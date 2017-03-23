#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
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

class RecognizeObjectsAction
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
    bool called_cam_service;
    // just for now
    sensor_msgs::PointCloud2ConstPtr sceneConst;

    mongodb_store::MessageStoreProxy message_store;


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


    bool do_recognition()
    {
        if (!ros::service::waitForService(recognizer_topic_, ros::Duration(5.0)))
            return false;
        squirrel_object_perception_msgs::SceneObject object;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Recognize>(recognizer_topic_);
        squirrel_object_perception_msgs::Recognize srv;
        srv.request.cloud = scene;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", recognizer_topic_.c_str());
            if (srv.response.ids.size() > 0) {
                for (int i= 0; i < srv.response.ids.size(); i++) {
                    object.category = srv.response.ids.at(i).data;
                    object.cloud = srv.response.models_cloud.at(i);
                    object.cloud.header.frame_id = srv.request.cloud.header.frame_id;
                    transformPointCloud(object.cloud, object.cloud.header.frame_id, "/map");
                    std::cout << "Category: " << object.category << std::endl;
                    object.pose = transform(srv.response.centroid.at(i).x, srv.response.centroid.at(i).y, srv.response.centroid.at(i).z,
                                            srv.request.cloud.header.frame_id, "/map").pose;
                    //TODO: transform BBox from Recognizer to BCylinder for SceneObject
                    //std::cout << "Position from Recognizer in map-frame (" << object.pose.position.x << "; "
                    //             << object.pose.position.y << "; " << object.pose.position.z << "; " << std::endl;
                    //compareToDB(sceneObject);
                    result_.objects_added.push_back(object);
                }
                return true;
            } else {
                std::cout << "could not recognize an object!" << std::endl;
                return false;
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
                    ROS_INFO("Object in DB with that pose exists already and is categorized");
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

    void getPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (!called_cam_service) { //this is a little hack
            std::cout << "Waiting for point cloud." << std::endl;
            sceneConst = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_, ros::Duration(20));
            std::cout << "Received point cloud." << std::endl;
            scene = *sceneConst;
            called_cam_service = true;
        }
    }

public:

    RecognizeObjectsAction(ros::NodeHandle &nh, std::string name) :
        as_(nh_, name, boost::bind(&RecognizeObjectsAction::executeCB, this, _1), false),
        action_name_(name),
        message_store(nh_)
    {
        nh_ = nh;
        as_.start();
        success = false;

        recognizer_topic_ = DEFAULT_RECOGNIZER_TOPIC_;
        if(nh_.getParam ( "recognizer_topic", recognizer_topic_ ))
            ROS_INFO("Listening to recognizer topic on %s", recognizer_topic_.c_str());
        else
            ROS_WARN("Recognizer topic not specified!");
    }

    ~RecognizeObjectsAction(void)
    {
    }

    void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
    {
        called_cam_service = false;
        result_.objects_added.clear();
        result_.objects_updated.clear();

        ROS_INFO("%s: executeCB started", action_name_.c_str());

        sleep(2); // HACK: Michael Zillich
        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted(result_);
        }

        // get data from depth camera
        ROS_INFO("Try to subscribe to point cloud");
        ros::Subscriber sub_pc = nh_.subscribe ("/kinect/depth_registered/points", 1, &RecognizeObjectsAction::getPointCloud, this);
        ros::Rate loop_rate (1);
        // poll until we did receive a point cloud
        sleep(3);
        while(!called_cam_service)
        {
            ros::spinOnce ();
            loop_rate.sleep ();
        }
        sub_pc.shutdown();


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
                ROS_INFO("There is nothing in the Database with ID %s. Use the whole scene for segmentation", (goal->id).c_str());
            }
        }

        success = do_recognition();

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
    ros::init(argc, argv, "squirrel_recognize_objects");
    ros::NodeHandle n ("~");
    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    RecognizeObjectsAction recognizeobjects(n, ros::this_node::getName());
    ROS_INFO("%s: ready...", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
