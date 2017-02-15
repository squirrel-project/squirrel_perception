#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <squirrel_object_perception_msgs/Classification.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/GetSaliency3DSymmetry.h>
#include <robotino_msgs/LookAtPanTilt.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>
#include <squirrel_object_perception_msgs/SegmentsToObjects.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationInit.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationOnce.h>
#include <squirrel_object_perception_msgs/Recognize.h>
#include <squirrel_planning_knowledge_msgs/UpdateObjectService.h>
#include <squirrel_planning_knowledge_msgs/AddObjectService.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sstream>
#include <algorithm>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include "mongodb_store/message_store.h"

#define DEFAULT_RECOGNIZER_TOPIC_ "/squirrel_recognizer/squirrel_recognize_objects"

class Object
{

public:
    squirrel_object_perception_msgs::SceneObject sceneObject;
    std_msgs::Int32MultiArray point_indices;
};

class LookForObjectsAction
{
protected:

    typedef pcl::PointXYZRGB PointT;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<squirrel_object_perception_msgs::LookForObjectsAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    squirrel_object_perception_msgs::LookForObjectsFeedback feedback_;
    squirrel_object_perception_msgs::LookForObjectsResult result_;
    // create needed variables
    sensor_msgs::PointCloud2 scene;
    bool success;
    std::vector<Object> objects;
    std::vector<Object>::iterator objectIterator;
    // just for now
    std::vector<squirrel_object_perception_msgs::RecognizeResponse> recognized_object;
    std::vector<std_msgs::Int32MultiArray> cluster_indices;
    int id_cnt_;

    mongodb_store::MessageStoreProxy message_store;
    ros::Publisher markerPublisher;

    // Recognition topic
    std::string recognizer_topic_;
    void set_publish_feedback(std::string phase, std::string status, int percent)
    {
        this->feedback_.current_phase = phase;
        this->feedback_.current_status = status;
        this->feedback_.percent_completed = percent;
        this->as_.publishFeedback(this->feedback_);
        return;
    }

    std::string get_unique_object_id()
    {
        std::stringstream ss;
        ss << this->id_cnt_;
        std::string str = ss.str();
        this->id_cnt_++;
        return (std::string("object") + str);
    }

    bool do_recognition(squirrel_object_perception_msgs::SceneObject &object)
    {
        if (!ros::service::waitForService(recognizer_topic_, ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Recognize>(recognizer_topic_);
	std::cout << "Recognizer topic: " << recognizer_topic_ << std::endl;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(scene, *cloud);

        pcl::PointCloud<PointT>::Ptr segmented_object(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(object.cloud, *segmented_object);
	
	if (segmented_object->points.size() > 0) {
		transformPointCloud(segmented_object, segmented_object->header.frame_id, "/map");

        	PointT min_p, max_p;
        	pcl::getMinMax3D(*segmented_object, min_p, max_p);

        	std::cout << "Size from Segmenter: " << "X(" << min_p.x << ";" << max_p.x << ")" <<
                     " Y(" << min_p.y << ";" << max_p.y << ")" <<
                     " Z(" << min_p.z << ";" << max_p.z << ")";

        	//TODO maybe add some buffer to the min/max points if segmentation method was not accurate
        	pcl::PassThrough<PointT> pass;
        	pass.setKeepOrganized(true);
        	pass.setFilterFieldName("x");
        	pass.setFilterLimits(min_p.x, max_p.x);
        	pass.setInputCloud(cloud);
        	pass.filter(*cloud);
        	pass.setFilterFieldName("y");
        	pass.setFilterLimits(min_p.y, max_p.y);
        	pass.setInputCloud(cloud);
        	pass.filter(*cloud);
        	pass.setFilterFieldName("z");
        	pass.setFilterLimits(min_p.z, max_p.z);
        	pass.setInputCloud(cloud);
        	pass.filter(*cloud);
	}
        
	squirrel_object_perception_msgs::Recognize srv;
        pcl::toROSMsg(*cloud, srv.request.cloud);
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", recognizer_topic_.c_str());
            if (srv.response.ids.size() >0)
            {
               	 this->recognized_object.push_back(srv.response);
		 int ind = most_confident_class(srv.response.confidences);
               	 object.category = srv.response.ids.at(ind).data; //this is only ok, when just one object gets recognized
               	 object.cloud = srv.response.model_clouds.at(ind);
               	 object.cloud.header.frame_id = srv.request.cloud.header.frame_id;
               	 transformPointCloud(object.cloud, object.cloud.header.frame_id, "/map");
               	 std::cout << "Category: " << object.category << std::endl;
               	 object.pose = transform(srv.response.centroids.at(ind).x, srv.response.centroids.at(ind).y, srv.response.centroids.at(ind).z,
               	                         srv.request.cloud.header.frame_id, "/map").pose;
               	 //TODO: transform BBox from Recognizer to BCylinder for SceneObject
               	 //std::cout << "Position from Recognizer in map-frame (" << object.pose.position.x << "; "
               	 //             << object.pose.position.y << "; " << object.pose.position.z << "; " << std::endl;
		compareToDB(object);
		
		return true;
	    } else {
		std::cout << "could not recognize an object!" << std::endl;
		return true;
	    }
        }
        else
        {
            return false;
        }
    }

    geometry_msgs::PoseStamped transform(double x, double y, double z, const std::string &from, const std::string &to)
    {
        geometry_msgs::PoseStamped before, after;

        before.pose.position.x = x;
        before.pose.position.y = y;
        before.pose.position.z = z;
        before.pose.orientation.x = 0;
        before.pose.orientation.y = 0;
        before.pose.orientation.z = 0;
        before.pose.orientation.w = 1;
        before.header.frame_id = from;
        try
        {
            tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose(to, before, after);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
        return after;
    }

    //transforms a whole point cloud and saves the result in the same object
    void transformPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_cluster, const std::string &from, const std::string &to) {
        try
        {
            tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
            pcl_ros::transformPointCloud(to, *cloud_cluster, *cloud_cluster, tf_listener);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
    }

    //transforms a whole point cloud and saves the result in the same object
    void transformPointCloud(sensor_msgs::PointCloud2 &cloud_cluster, const std::string &from, const std::string &to) {
        try
        {
            tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
            pcl_ros::transformPointCloud(to, cloud_cluster, cloud_cluster, tf_listener);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
    }


    bool compareToDB(squirrel_object_perception_msgs::SceneObject sceneObject) {
        std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > sceneObjects_results;
        message_store.query<squirrel_object_perception_msgs::SceneObject>(sceneObjects_results);

        //TODO think about a good way when to update objects.
        BOOST_FOREACH(boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> sceneObject_db, sceneObjects_results) {
            if (isSame(sceneObject, *sceneObject_db)) { //update
                if (sceneObject.category == "unknown" && sceneObject_db->category != "unknown") {
                    //do nothing, the object in the DB was already categorized
                } else {
                    sceneObject_db->pose = sceneObject.pose;
                    sceneObject_db->category = sceneObject.category;
                    sceneObject_db->cloud = sceneObject.cloud;
                    sceneObject_db->bounding_cylinder = sceneObject.bounding_cylinder;
                    result_.objects_updated.push_back(*sceneObject_db);
	    	    std::cout << "Recognized object got updated in DB" << std::endl;
                }
                return true;
            }
        }
        //new object - add it to DB
        result_.objects_added.push_back(sceneObject);
	std::cout << "Recognized object got added to DB" << std::endl;
        return true;
    }

    //check for overlapping bounding cylinders
    bool isSame(const squirrel_object_perception_msgs::SceneObject& sceneObject, const squirrel_object_perception_msgs::SceneObject& sceneObject_db)
    {
        double size1 = std::sqrt(std::pow(sceneObject.bounding_cylinder.diameter/2,2) +
                                 std::pow(sceneObject.bounding_cylinder.diameter/2,2) +
                                 std::pow(sceneObject.bounding_cylinder.height/2,2));
        double size2 = std::sqrt(std::pow(sceneObject_db.bounding_cylinder.diameter/2,2) +
                                 std::pow(sceneObject_db.bounding_cylinder.diameter/2,2) +
                                 std::pow(sceneObject_db.bounding_cylinder.height/2,2));;

        geometry_msgs::Point p1 = sceneObject.pose.position;
        geometry_msgs::Point p2 = sceneObject_db.pose.position;
        geometry_msgs::Point d;
        d.x = p1.x - p2.x;
        d.y = p1.y - p2.y;
        d.z = p1.z - p2.z;
        if(sqrt(d.x*d.x + d.y*d.y + d.z*d.z) < std::min(size1/2., size2/2.))
            return true;
        else
            return false;
    }


    int most_confident_class(std::vector<float> r)
    {
	float  max_val=0.0;
	int ind = 0;
	for (size_t i = 0; i < r.size(); i++) {
		if (r.at(i) > max_val) {
			max_val = r.at(i);
			ind = i;
		}
	
	}
	return ind;
    }

public:

    LookForObjectsAction(ros::NodeHandle &nh, std::string name) :
        as_(nh_, name, boost::bind(&LookForObjectsAction::executeCB, this, _1), false),
        action_name_(name),
        message_store(nh_)
    {
	nh_ = nh;
        id_cnt_ = 1;
        as_.start();
        success = false;
	recognizer_topic_ = DEFAULT_RECOGNIZER_TOPIC_;

	if(nh_.getParam ( "recognizer_topic", recognizer_topic_ ))
            ROS_INFO("Listening to recognizer topic on %s", recognizer_topic_.c_str());
        else
            ROS_WARN("Recognizer topic not specified!");
    }

    ~LookForObjectsAction(void)
    {
    }

    void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
    {

	objects.clear();
	
    	sensor_msgs::PointCloud2ConstPtr sceneConst;
        ROS_INFO("%s: executeCB started", action_name_.c_str());

        sleep(2); // HACK: Michael Zillich


        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted(result_);
        }

        // get data from depth camera
        sceneConst = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_, ros::Duration(20));

        squirrel_object_perception_msgs::SceneObject sceneObject;

	if (sceneConst != NULL)
        {
            scene = *sceneConst;
	    sceneConst.reset();
            ROS_INFO("%s: Received data", action_name_.c_str());
            std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
            if (goal->look_for_object == squirrel_object_perception_msgs::LookForObjectsGoal::CHECK) {
                //get lump size from DB and filter cloud for segmentation to cut off unnecessary parts
                ROS_INFO("Checking out a lump");


                if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(goal->id, results)) {
                    if(results.size()<1) { // no results
                        ROS_INFO("There is nothing in the Database with ID %s. Use the whole scene for recognition", (goal->id).c_str());
                    } else {
                        sceneObject = *results.at(0);
                        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                        pcl::fromROSMsg(scene, *cloud);
			PointT min_p, max_p;

			//if lump was already segmented once
                        if (sceneObject.cloud.width > 0) {
                            ROS_INFO("We already have a cloud of the lump");
                            pcl::PointCloud<PointT>::Ptr lump(new pcl::PointCloud<PointT>);
                            pcl::fromROSMsg(sceneObject.cloud, *lump);

                            transformPointCloud(lump, lump->header.frame_id, "/map");
                            pcl::getMinMax3D(*lump, min_p, max_p);

                        } else if (sceneObject.bounding_cylinder.diameter != 0.0) {
                            ROS_INFO("Use bounding cylinder to crop point cloud");
                            min_p.x = sceneObject.pose.position.x - sceneObject.bounding_cylinder.diameter/2;
                            max_p.x = sceneObject.pose.position.x + sceneObject.bounding_cylinder.diameter/2;
                            min_p.y = sceneObject.pose.position.y - sceneObject.bounding_cylinder.diameter/2;
                            max_p.y = sceneObject.pose.position.y + sceneObject.bounding_cylinder.diameter/2;
                            min_p.z = 0;
                            max_p.z = sceneObject.bounding_cylinder.height;

                            std::cout << "Pose x: " << sceneObject.pose.position.x << "; y: " << sceneObject.pose.position.y << std::endl;
			}



			transformPointCloud(cloud, cloud->header.frame_id, "/map");

                        std::cout << "Size: " << "X(" << min_p.x << ";" << max_p.x << ")" <<
                                     " Y(" << min_p.y << ";" << max_p.y << ")" <<
                                     " Z(" << min_p.z << ";" << max_p.z << ")" << std::endl;


                        //TODO maybe add some buffer to the min/max points if segmentation method was not accurate
                        pcl::PassThrough<PointT> pass;
                        pass.setKeepOrganized(true);
                        pass.setFilterFieldName("x");
                        pass.setFilterLimits(min_p.x-0.10, max_p.x+0.10);
                        pass.setInputCloud(cloud);
                        pass.filter(*cloud);
                        pass.setFilterFieldName("y");
                        pass.setFilterLimits(min_p.y-0.10, max_p.y+0.10);
                        pass.setInputCloud(cloud);
                        pass.filter(*cloud);
                        pass.setFilterFieldName("z");
                        pass.setFilterLimits(min_p.z-0.05, max_p.z+0.05);
                        pass.setInputCloud(cloud);
			pass.filter(*cloud);


                        pcl::toROSMsg(*cloud, scene);
                    }
                }
		else {
			ROS_INFO("There is nothing in the Database with ID %s. Use the whole scene for recognition", (goal->id).c_str()); 
		}
            }
        }
        else
        {
            ROS_INFO("squirrel_object_perception: Did not receive any data from the camera");
            result_.result_status = "Unable to get data from the camera.";
            as_.setAborted(result_);
            success = false;
            return;
        }
	
       success = do_recognition(sceneObject);
	
        if(success)
        {
            //result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
        else
        {
            result_.result_status = "Some error has occured.";
            as_.setAborted(result_);
        }

    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "squirrel_look_for_objects_in_hand");
    ros::NodeHandle n("~");
    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    LookForObjectsAction lookforobjects(n, ros::this_node::getName());
    ROS_INFO("%s: ready...", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
