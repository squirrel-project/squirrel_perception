#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <squirrel_object_perception_msgs/Classification.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/GetSaliency3DSymmetry.h>
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
    sensor_msgs::Image saliency_map;
    std::vector<Object> objects;
    std::vector<Object>::iterator objectIterator;
    // just for now
    std::vector<squirrel_object_perception_msgs::RecognizeResponse> recognized_object;
    std::vector<std_msgs::Int32MultiArray> cluster_indices;
    int id_cnt_;

    mongodb_store::MessageStoreProxy message_store;
    ros::Publisher markerPublisher;
    visualization_msgs::Marker zyl_marker;
    std::vector<int32_t> vis_marker_ids;

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
        if (!ros::service::waitForService("/squirrel_recognizer/squirrel_recognize_objects", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Recognize>("/squirrel_recognizer/squirrel_recognize_objects");

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(scene, *cloud);

        pcl::PointCloud<PointT>::Ptr segmented_object(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(object.cloud, *segmented_object);
	
        pcl::PCDWriter writer;
        //writer.write<PointT>("/home/squirrel/edith_rec_test/before_recognize.pcd", *segmented_object, false);

	transformPointCloud(segmented_object, segmented_object->header.frame_id, "/kinect_depth_optical_frame");

        PointT min_p, max_p;
        pcl::getMinMax3D(*segmented_object, min_p, max_p);

        std::cout << "Size from Segmenter: " << "X(" << min_p.x << ";" << max_p.x << ")" <<
                     " Y(" << min_p.y << ";" << max_p.y << ")" <<
                     " Z(" << min_p.z << ";" << max_p.z << ")";

        //TODO maybe add some buffer to the min/max points if segmentation method was not accurate
        pcl::PassThrough<PointT> pass;
        pass.setKeepOrganized(true);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_p.x-0.05, max_p.x+0.05);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(min_p.y-0.05, max_p.y+0.05);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_p.z-0.05, max_p.z+0.05);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);
	
        //writer.write<PointT>("/home/edith/edith_rec_test/cutted.pcd", *cloud, false);
        
	squirrel_object_perception_msgs::Recognize srv;
        pcl::toROSMsg(*cloud, srv.request.cloud);
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_recognizer/squirrel_recognize_objects");
            if (srv.response.ids.size() > 0) { 
                this->recognized_object.push_back(srv.response);
            	object.category = srv.response.ids.at(0).data; //this is only ok, when just one object gets recognized
            	object.cloud = srv.response.model_clouds.at(0);
                object.cloud.header.frame_id = srv.request.cloud.header.frame_id;
            	transformPointCloud(object.cloud, object.cloud.header.frame_id, "/map");
            	std::cout << "Category: " << object.category << std::endl;
            	object.pose = transform(srv.response.centroids.at(0).x, srv.response.centroids.at(0).y, srv.response.centroids.at(0).z,
            	                        "/kinect_depth_optical_frame", "/map").pose;
            	//TODO: transform BBox from Recognizer to BCylinder for SceneObject
            	return true;
	    } else {
		return false;
	    }
        }
        else
        {
            return false;
        }
    }

    bool setup_visualization()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_visualization_init", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentVisualizationInit>("/squirrel_segmentation_visualization_init");
        squirrel_object_perception_msgs::SegmentVisualizationInit srv;
        srv.request.cloud = (this->scene);
        srv.request.saliency_map = this->saliency_map;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_visualization_init");
            return true;
        }
        else
        {
            return false;
        }
    }

    bool run_visualization_once()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_visualization_once", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentVisualizationOnce>("/squirrel_segmentation_visualization_once");
        squirrel_object_perception_msgs::SegmentVisualizationOnce srv;
        srv.request.clusters_indices = this->cluster_indices;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_visualization_once");
            return true;
        }
        else
        {
            return false;
        }
    }

    bool get_saliency_map()
    {
        if (!ros::service::waitForService("/squirrel_attention_itti", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::GetSaliency3DSymmetry>("/squirrel_attention_itti");
        squirrel_object_perception_msgs::GetSaliency3DSymmetry srv;
        srv.request.cloud = (this->scene);
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_attention_itti");
            this->saliency_map = srv.response.saliency_map;
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/squirrel_attention_itti");
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
                }
                return true;
            }
        }
        //new object - add it to DB
        result_.objects_added.push_back(sceneObject);
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

    bool add_object_to_db(squirrel_object_perception_msgs::SceneObject sceneObject)
    {
        if (!ros::service::waitForService("/kcl_rosplan/add_object", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_planning_knowledge_msgs::AddObjectService>("/kcl_rosplan/add_object");
        squirrel_planning_knowledge_msgs::AddObjectService srv;
        srv.request.object.header = sceneObject.header;
        srv.request.object.header.frame_id = "/map";
        srv.request.object.id = sceneObject.id;
        srv.request.object.category = sceneObject.category;
        srv.request.object.pose = transform(sceneObject.pose.position.x, sceneObject.pose.position.y, sceneObject.pose.position.z, sceneObject.header.frame_id, "/map").pose;
        transformPointCloud(sceneObject.cloud, sceneObject.cloud.header.frame_id, "/map");
        srv.request.object.cloud = sceneObject.cloud;
        srv.request.object.bounding_cylinder = sceneObject.bounding_cylinder;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/kcl_rosplan/add_object");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/kcl_rosplan/add_object");
            return false;
        }
    }

    void visualizeObject(squirrel_object_perception_msgs::SceneObject sceneObject) {
        zyl_marker.header.frame_id = "map";
        zyl_marker.header.stamp = ros::Time();
        zyl_marker.ns = "object_marker";
        zyl_marker.id = std::atoi(sceneObject.id.substr(6, std::string::npos).c_str());
        zyl_marker.lifetime = ros::Duration();
        zyl_marker.type = visualization_msgs::Marker::CYLINDER;
        zyl_marker.action = visualization_msgs::Marker::ADD;
        zyl_marker.pose.position.x = sceneObject.pose.position.x;
        zyl_marker.pose.position.y = sceneObject.pose.position.y;
        zyl_marker.pose.position.z = sceneObject.pose.position.z;
        zyl_marker.pose.orientation.x = 0.0;
        zyl_marker.pose.orientation.y = 0.0;
        zyl_marker.pose.orientation.z = 0.0;
        zyl_marker.pose.orientation.w = 1.0;
        zyl_marker.scale.x = sceneObject.bounding_cylinder.diameter;
        zyl_marker.scale.y = sceneObject.bounding_cylinder.diameter;
        zyl_marker.scale.z = sceneObject.bounding_cylinder.height;
        zyl_marker.color.r = 0.1;
        zyl_marker.color.g = 0.1;
        zyl_marker.color.b = 0.9;
        zyl_marker.color.a = 0.4;

        markerPublisher.publish(zyl_marker);
        vis_marker_ids.push_back(zyl_marker.id);
    }

    bool setup_segmentation()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_incremental_init", ros::Duration(5.0)))
            return false;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(this->scene, *cloud);
    //pcl::io::savePCDFileBinary("/home/squirrel/edith_rec_test/before_segmentation.pcd", *cloud);

        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentInit>("/squirrel_segmentation_incremental_init");
        squirrel_object_perception_msgs::SegmentInit srv;
        srv.request.cloud = (this->scene);
        //srv.request.saliency_map = this->saliency_map;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_incremental_init");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/squirrel_segmentation_incremental_init");
            return false;
        }
    }

    bool run_segmentation_once()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_incremental_once", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");
        squirrel_object_perception_msgs::SegmentOnce srv;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_incremental_once");
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/squirrel_segmentation_incremental_once");
            return false;
        }

        this->cluster_indices = srv.response.clusters_indices;


        for(int i=0; srv.response.poses.size(); i++) {
            ROS_INFO("appending object");
            Object obj;
            obj.sceneObject.category = "unknown";
            obj.sceneObject.id = get_unique_object_id();
            obj.sceneObject.header = srv.response.poses[i].header;
            obj.point_indices = srv.response.clusters_indices[i];
            obj.sceneObject.cloud = srv.response.points[i];
            obj.sceneObject.pose = srv.response.poses[i].pose;

            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(obj.sceneObject.cloud, *cloud);

            PointT min_p, max_p;
            pcl::getMinMax3D(*cloud, min_p, max_p);

            //it is in kinect-optical-frame! (x=z, y=x, z=y)
            double x_diam = double(max_p.x - min_p.x + 1);
            double y_diam = double(max_p.y - min_p.y + 1);
            double z_diam = double(max_p.z - min_p.z + 1);

            double diam = std::sqrt(std::pow(x_diam,2) + std::pow(z_diam,2));

            obj.sceneObject.bounding_cylinder.diameter = diam;
            obj.sceneObject.bounding_cylinder.height = y_diam;
            this->objects.push_back(obj);
            return true;
        }
    }

    bool update_object_in_db(squirrel_object_perception_msgs::SceneObject sceneObject)
    {
        if (!ros::service::waitForService("/kcl_rosplan/update_object", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_planning_knowledge_msgs::UpdateObjectService>("/kcl_rosplan/update_object");
        squirrel_planning_knowledge_msgs::UpdateObjectService srv;
        srv.request.object.header = sceneObject.header;
        srv.request.object.header.frame_id = "/map";
        srv.request.object.id = sceneObject.id;
        srv.request.object.category = sceneObject.category;
        srv.request.object.pose = transform(sceneObject.pose.position.x, sceneObject.pose.position.y, sceneObject.pose.position.z, sceneObject.header.frame_id, "/map").pose;
        transformPointCloud(sceneObject.cloud, sceneObject.cloud.header.frame_id, "/map");
        srv.request.object.cloud = sceneObject.cloud;

        if (client.call(srv))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    std::string most_confident_class(squirrel_object_perception_msgs::Classification classification)
    {
        std::vector<float>::iterator max;
        max = std::max_element(classification.confidence.begin(), classification.confidence.end());
        int max_index = std::distance(classification.confidence.begin(), max);
        return classification.class_type[max_index].data;
    }

    bool run_classifier()
    {
        if (!ros::service::waitForService("/squirrel_classify", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Classify>("/squirrel_classify");
        squirrel_object_perception_msgs::Classify srv;
        // TODO: check data
        std::vector<std_msgs::Int32MultiArray> point_indices;
        point_indices.push_back(this->objects.back().point_indices);
        srv.request.cloud = (this->scene);
        srv.request.clusters_indices = point_indices;
        if (client.call(srv))
        {
            if (srv.response.class_results.size() == 1)
            {
                this->objects.back().sceneObject.category = this->most_confident_class(srv.response.class_results[0]);
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

public:

    LookForObjectsAction(std::string name) :
        as_(nh_, name, boost::bind(&LookForObjectsAction::executeCB, this, _1), false),
        action_name_(name),
        message_store(nh_)
    {
        id_cnt_ = 1;
        as_.start();
        success = false;

        markerPublisher = nh_.advertise<visualization_msgs::Marker>("visualization_segm_objects", 0);
    }

    ~LookForObjectsAction(void)
    {
    }

    void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
    {
	
    	sensor_msgs::PointCloud2ConstPtr sceneConst;
        ROS_INFO("%s: executeCB started", action_name_.c_str());

        sleep(2); // HACK: Michael Zillich

        for (std::vector<int>::iterator it = vis_marker_ids.begin() ; it != vis_marker_ids.end(); ++it) {
            zyl_marker.id = *it;
            zyl_marker.ns = "object_marker";
            zyl_marker.action = visualization_msgs::Marker::DELETE;
            markerPublisher.publish(zyl_marker);
        }


        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted(result_);
        }

        // get data from depth camera
        sceneConst = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_, ros::Duration(5));

	if (sceneConst != NULL)
        {
            scene = *sceneConst;
	    sceneConst.reset();
            ROS_INFO("%s: Received data", action_name_.c_str());
            if (goal->look_for_object == squirrel_object_perception_msgs::LookForObjectsGoal::CHECK) {
                //get lump size from DB and filter cloud for segmentation to cut off unnecessary parts
                ROS_INFO("Checking out a lump");

                squirrel_object_perception_msgs::SceneObject sceneObject;

                std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
                if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(goal->id, results)) {
                    if(results.size()<1) { // no results
                        ROS_INFO("There is nothing in the Database with ID %s. Use the whole scene for segmentation", (goal->id).c_str());
                    } else {
                        sceneObject = *results.at(0);
                        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                        pcl::fromROSMsg(scene, *cloud);

                        pcl::PointCloud<PointT>::Ptr lump(new pcl::PointCloud<PointT>);
                        pcl::fromROSMsg(sceneObject.cloud, *lump);

                        transformPointCloud(lump, lump->header.frame_id, "/kinect_depth_optical_frame");
			
                        PointT min_p, max_p;
                        pcl::getMinMax3D(*lump, min_p, max_p);

                        std::cout << "Size from DB: " << "X(" << min_p.x << ";" << max_p.x << ")" <<
                                     " Y(" << min_p.y << ";" << max_p.y << ")" <<
                                     " Z(" << min_p.z << ";" << max_p.z << ")";

                        //TODO maybe add some buffer to the min/max points if segmentation method was not accurate
                        /*pcl::PassThrough<PointT> pass;
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
                        pass.filter(*cloud); */

                        pcl::toROSMsg(*cloud, scene);
                    }
                }
		else {
			ROS_INFO("There is nothing in the Database with ID %s. Use the whole scene for segmentation", (goal->id).c_str()); 
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

        //TODO we do not use attention right now
        /*if (!get_saliency_map())
    {
        result_.result_status = "unable to get saliency map";
        as_.setAborted(result_);
        return;
    }*/

        if (!setup_segmentation())
        {
            result_.result_status = "unable to initialze segmentation";
            as_.setAborted(result_);
            return;
        }

        //not supported by popout_segmentation
        /*if (!setup_visualization())
    {
        result_.result_status = "unable to initialze visualization";
        as_.setAborted(result_);
        return;
    }*/


        // TODO: find a reasonable number of times to run here
        for(int i=0; i<1; i++)
        {
            run_segmentation_once();
            //run_visualization_once();
        }
        if (objects.size() < 1)
        {
            result_.result_status = "No objects classified";
            as_.setAborted(result_);
            return;
        }
        for(objectIterator = objects.begin(); objectIterator != objects.end(); objectIterator++)
        {
            do_recognition((*objectIterator).sceneObject);
            success = compareToDB((*objectIterator).sceneObject);
            visualizeObject((*objectIterator).sceneObject);
            //success = add_object_to_db((*objectIterator).sceneObject);
            if (!success)
                break;
        }
	/*squirrel_object_perception_msgs::SceneObject sceneObjectTemp;
	sceneObjectTemp.cloud = scene;
	sceneObjectTemp.category = "unknown";
	sceneObjectTemp.id = "object1";
	do_recognition(sceneObjectTemp);*/
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
    ros::init(argc, argv, "look_for_objects");
    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    LookForObjectsAction lookforobjects(ros::this_node::getName());
    ros::spin();

    return 0;
}
