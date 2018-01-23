#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <squirrel_planning_knowledge_msgs/UpdateObjectService.h>
#include <squirrel_planning_knowledge_msgs/AddObjectService.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <squirrel_view_controller_msgs/LookAtPosition.h>
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
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>
#include <squirrel_object_perception_msgs/BBox.h>
#include <v4r_object_recognition_msgs/recognize.h>

#define DEFAULT_RECOGNIZER_TOPIC_ "/recognition_service/recognize"


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
    int unique_id;
    sensor_msgs::PointCloud2ConstPtr sceneConst;
    ros::Publisher markerPublisher;
    ros::ServiceClient segm_init_client;
    ros::ServiceClient segm_once_client;
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
        ros::ServiceClient client = nh_.serviceClient<v4r_object_recognition_msgs::recognize>(recognizer_topic_);
        v4r_object_recognition_msgs::recognize srv;
        srv.request.cloud = scene;
        std::cout << "***********************************Scene Frame: " << scene.header.frame_id << " *******************************" << std::endl;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", recognizer_topic_.c_str());
            if (srv.response.ids.size() > 0) {
                for (int i= 0; i < srv.response.ids.size(); i++) {
                    object.header.frame_id = "/map";
                    object.header.stamp = ros::Time::now();
                    object.category = srv.response.ids.at(i).data;
                    object.category = "battery";
                    object.cloud = srv.response.models_cloud.at(i);
                    object.cloud.header.frame_id = srv.request.cloud.header.frame_id;
                    transformPointCloud(object.cloud, object.cloud.header.frame_id, "/map");
                    std::cout << "Category: " << object.category << std::endl;
                    object.pose = transform(srv.response.centroid.at(i).x, srv.response.centroid.at(i).y, srv.response.centroid.at(i).z,
                                            srv.request.cloud.header.frame_id, "/map").pose;

                    //the wizard was called
                    if (srv.response.confidence.size() !=0 &&srv.response.confidence.at(i) == 1.0) {
                        squirrel_object_perception_msgs::SegmentOnce::Response segm_result;
                        bool seg_ok = do_segmentation(scene, segm_result);
                        if (!seg_ok) {
                            result_.used_wizard = true;
                        } else {
                            try
                            {
                                tf_listener.waitForTransform(segm_result.poses[0].header.frame_id, "/map", ros::Time::now(), ros::Duration(1.0));
                                tf_listener.transformPose("/map", segm_result.poses[0], segm_result.poses[0]);
                            }
                            catch (tf::TransformException& ex)
                            {
                                ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
                            }
                            object.pose = segm_result.poses[0].pose;

                            pcl::PointCloud<PointT>::Ptr cloud_segm(new pcl::PointCloud<PointT>);
                            pcl::fromROSMsg(segm_result.points[0], *cloud_segm);
                            transformPointCloud(cloud_segm, cloud_segm->header.frame_id, "/map");
                            PointT min_p, max_p;
                            pcl::getMinMax3D(*cloud_segm, min_p, max_p);
                            object.bounding_cylinder.height = max_p.z;
                            object.bounding_cylinder.diameter = std::sqrt((max_p.x - min_p.x)* (max_p.x - min_p.x) +
                                    (max_p.y - min_p.y) * (max_p.y - min_p.y));
                        }
                    }
                    else {
                        //transform bounding box into bounding cylinder
                        squirrel_object_perception_msgs::BBox bbox;
			bbox.point = srv.response.bbox.at(i).points;
                        transform_bbox(bbox, "/kinect_depth_optical_frame", "/map");
                        double max_z = std::numeric_limits<float>::min();
                        for (int i = 0; i < bbox.point.size(); i++) {
                            if (bbox.point.at(i).z > max_z) {
                                max_z = bbox.point.at(i).z;
                            }
                        }
                        object.bounding_cylinder.height = max_z;

                        float max_dist = std::numeric_limits<float>::min();
                        for (int p1 = 0; p1 < bbox.point.size(); p1++) {
                            geometry_msgs::Point32 point1 = bbox.point.at(p1);
                            for (int p2 = 0; p2 < bbox.point.size(); p2++) {
                                geometry_msgs::Point32 point2 = bbox.point.at(p2);
                                float dist = (point1.x - point2.x) * (point1.x - point2.x) +
                                        (point1.y - point2.y) * (point1.y - point2.y);
                                dist = std::sqrt(dist);
                                if (max_dist < dist) {
                                    max_dist = dist;
                                }
                            }
                        }
                        object.bounding_cylinder.diameter = max_dist;
                        visualizeBCylinder(object);
                    }
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

    bool transform_bbox(squirrel_object_perception_msgs::BBox &bbox, const std::string &from, const std::string &to) {
        geometry_msgs::PoseStamped before, after;

        for (int i = 0; i < bbox.point.size(); i++) {
            before.pose.position.x = bbox.point[i].x;
            before.pose.position.y = bbox.point[i].y;
            before.pose.position.z = bbox.point[i].z;
            before.pose.orientation.x = 0;
            before.pose.orientation.y = 0;
            before.pose.orientation.z = 0;
            before.pose.orientation.w = 1;
            before.header.frame_id = from;
            try
            {
                tf_listener.waitForTransform(from, to, ros::Time(0), ros::Duration(0.2));
                tf_listener.transformPose(to, before, after);
                bbox.point[i].x = after.pose.position.x;
                bbox.point[i].y = after.pose.position.y;
                bbox.point[i].z = after.pose.position.z;
            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
                return false;
            }
        }
        return true;
    }

    bool do_segmentation(const sensor_msgs::PointCloud2 &cloud, squirrel_object_perception_msgs::SegmentOnce::Response &segm_result) {
        squirrel_object_perception_msgs::SegmentInit srv_init;
        srv_init.request.cloud = cloud;
        if (segm_init_client.call(srv_init)) {
            ROS_INFO("Called segmentation init");
        }
        else {
            ROS_ERROR("Failed to call segmentation init!");
        }
        squirrel_object_perception_msgs::SegmentOnce srv_once;
        if (segm_once_client.call(srv_once)) {
            ROS_INFO("Called segmentation once");
            segm_result = srv_once.response;
            return true;
        }
        else {
            ROS_ERROR("Failed to call segmentation once!");
        }
        return false;
    }

    void visualizeBCylinder(squirrel_object_perception_msgs::SceneObject obj) {
        visualization_msgs::Marker zyl_marker;
        zyl_marker.header.frame_id = obj.header.frame_id;
        zyl_marker.header.stamp = ros::Time();
        zyl_marker.ns = "zylinder_marker";
        zyl_marker.id = unique_id;
        unique_id += 1;
        zyl_marker.lifetime = ros::Duration();
        zyl_marker.type = visualization_msgs::Marker::CYLINDER;
        zyl_marker.action = visualization_msgs::Marker::ADD;
        zyl_marker.pose.position.x = obj.pose.position.x;
        zyl_marker.pose.position.y = obj.pose.position.y;
        zyl_marker.pose.position.z = obj.pose.position.z;
        zyl_marker.pose.orientation.x = 0.0;
        zyl_marker.pose.orientation.y = 0.0;
        zyl_marker.pose.orientation.z = 0.0;
        zyl_marker.pose.orientation.w = 1.0;
        zyl_marker.scale.x = obj.bounding_cylinder.diameter;
        zyl_marker.scale.y = obj.bounding_cylinder.diameter;
        zyl_marker.scale.z = obj.bounding_cylinder.height;
        zyl_marker.color.r = 1.0;
        zyl_marker.color.g = 0.9;
        zyl_marker.color.b = 0.1;
        zyl_marker.color.a = 0.4;

        markerPublisher.publish(zyl_marker);
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
            sleep(1);
            std::cout << "Waiting for point cloud." << std::endl;
            sceneConst = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_);
            std::cout << "Received point cloud." << std::endl;
            scene = *sceneConst;
            called_cam_service = true;
        }
    }

    bool moveCameraToPose(geometry_msgs::PoseStamped pose) {
        ros::ServiceClient client = nh_.serviceClient<squirrel_view_controller_msgs::LookAtPosition>("/squirrel_view_controller/look_at_position");

        squirrel_view_controller_msgs::LookAtPosition srv;
        srv.request.target.header.frame_id= pose.header.frame_id;
        srv.request.target.pose.position.x = pose.pose.position.x;
        srv.request.target.pose.position.y = pose.pose.position.y;
        srv.request.target.pose.position.z = pose.pose.position.z;
        srv.request.target.pose.orientation.x = 0.0;
        srv.request.target.pose.orientation.y = 0.0;
        srv.request.target.pose.orientation.z = 0.0;
        srv.request.target.pose.orientation.w = 1.0;

        if(client.call(srv)) {
            ROS_INFO("Moved camera to new position");
            return true;
        } else {
            ROS_INFO("Did NOT move camera to new position");
            return false;
        }
    }

    bool moveCameraToDefault() {
        //move back to default position
        ros::ServiceClient resetCamera_client = nh_.serviceClient<std_srvs::Empty>("/squirrel_view_controller/reset_positions");

        std_srvs::Empty e;
        if (!resetCamera_client.call(e))
        {
            ROS_ERROR("Failed to move camera back to default tilt position");
            return false;
        }

        return true;
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
        unique_id = 0;
        result_.used_wizard = false;
        segm_init_client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentInit>("/squirrel_segmentation_incremental_init");
        segm_once_client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");
        markerPublisher = nh_.advertise<visualization_msgs::Marker>("rec_box_marker", 0);
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

        success = moveCameraToPose(goal->look_at_pose);
        if (!success) {
            result_.result_status = "Could not move camera to default position";
            as_.setAborted(result_);
            return;
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

                    //TODO check if cloud should be transformed back in kinect_frame

                    pcl::toROSMsg(*cloud, scene);
                }
            }
            else {
                ROS_INFO("There is nothing in the Database with ID %s. Use the whole scene for segmentation", (goal->id).c_str());
            }
        }

        success = do_recognition();
        moveCameraToDefault();

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
