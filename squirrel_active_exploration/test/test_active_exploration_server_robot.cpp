#include <ros/ros.h>
#include <octomap_ros/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/ActiveExplorationNBV.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "squirrel_active_exploration/io_utils.h"

using namespace std;
using namespace pcl;
using namespace octomap;

// Default definitions, should be replaced by program arguments
#define _VARIANCE 0.5
#define _TREE_DEPTH 14  // 14 => resolution of 0.1m, initial tree depth is 16 => 0.025m
#define _CAMERA_HEIGHT 0.75 // height of camera above robot base
#define _ROBOT_RADIUS 0.22
#define _DISTANCE_FROM_CENTER 2
#define _NUM_LOCATIONS 10
#define _TREE_RESOLUTION 0.1
#define _BASE "/base_link"
#define _MAP "/map"
#define _CAMERA_TOPIC "/kinect/depth_registered/points"
#define _CAMERA_FRAME "/kinect_depth_optical_frame"
#define _MARKER_NAME "active_exploration_waypoints"

PointCloud<PointT> transform_cloud(sensor_msgs::PointCloud2ConstPtr scene);
bool get_pose(Eigen::Vector4f &pose);
bool get_octree(const sensor_msgs::PointCloud2 &cloud_msg, const Eigen::Vector4f &pose, const double &tree_resolution,
                octomap_msgs::Octomap &oc_msg);
vector<int> get_valid_indices(const squirrel_object_perception_msgs::SegmentOnce &seg_srv, const PointCloud<PointT> &cloud);
void publish_markers(ros::NodeHandle &node_handle, const vector<geometry_msgs::Point> &locations, const int &nbv_ix);

// Run Main
int main(int argc, char **argv)
{
    ros::init (argc, argv ,"test_active_exploration_server_robot");
    ros::NodeHandle n("~");

    // Get the parameters
    string train_dir;
    double variance = _VARIANCE;
    double camera_height = _CAMERA_HEIGHT;
    double robot_radius = _ROBOT_RADIUS;
    double distance_from_center = _DISTANCE_FROM_CENTER;
    int num_locations = _NUM_LOCATIONS;
    double tree_resolution = _TREE_RESOLUTION;
    bool occlusions = true;
    bool working_classifier = false;
    // Read the input if it exists
    n.getParam ("train_dir", train_dir);
    n.getParam ("variance", variance);
    n.getParam ("camera_height", camera_height);
    n.getParam ("robot_radius", robot_radius);
    n.getParam ("distance_from_center", distance_from_center);
    n.getParam ("num_locations", num_locations);
    n.getParam ("tree_resolution", tree_resolution);
    n.getParam ("occlusions", occlusions);
    n.getParam ("working_classifier", working_classifier);
    // Print out the input
    ROS_INFO("test_active_exploration_server_robot : input parameters");
    cout << "Variance = " << variance << endl;
    cout << "Camera height = " << camera_height << endl;
    cout << "Robot radius = " << robot_radius << endl;
    cout << "Distance from center = " << distance_from_center << endl;
    cout << "Num locations = " << num_locations << endl;
    cout << "Tree resolution = " << tree_resolution << endl;
    if (occlusions)
        cout << "Occlusions = TRUE" << endl;
    else
        cout << "Occlusions = FALSE" << endl;
    if (working_classifier)
        cout << "Classifier = TRUE" << endl;
    else
        cout << "Classifier = FALSE" << endl;
    // If classifier is not working, then needs a valid training directory
    if (!working_classifier)
    {
        if (train_dir.size() == 0)
        {
            ROS_ERROR("test_active_exploration_server_robot : classifier is assumed not working, you must enter a training directory");
            return EXIT_FAILURE;
        }
        // Add backslash
        train_dir = add_backslash(train_dir);
    }


    // Create the service clients
    ros::ServiceClient nbv_client = n.serviceClient<squirrel_object_perception_msgs::ActiveExplorationNBV>("/squirrel_active_exploration");
    squirrel_object_perception_msgs::ActiveExplorationNBV nbv_srv;
    ros::ServiceClient seg_init_client = n.serviceClient<squirrel_object_perception_msgs::SegmentInit>("/squirrel_segmentation_incremental_init");;
    squirrel_object_perception_msgs::SegmentInit seg_init_srv;
    ros::ServiceClient seg_client = n.serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");
    squirrel_object_perception_msgs::SegmentOnce seg_srv;
    ros::ServiceClient classify_client = n.serviceClient<squirrel_object_perception_msgs::Classify>("/squirrel_classify");
    squirrel_object_perception_msgs::Classify classify_srv;

    // To pass to the service
    Eigen::Vector4f pose;
    sensor_msgs::PointCloud2 cloud_msg;

    while (ros::ok)
    {
        // Get the point cloud
        sensor_msgs::PointCloud2ConstPtr scene = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(_CAMERA_TOPIC,
                                                                                                      n, ros::Duration(50));
        // Transform the point cloud
        PointCloud<PointT> cloud = transform_cloud(scene);
        // Convert to ros message
        pcl::toROSMsg(cloud, cloud_msg);

        // Get the pose
        if (!get_pose(pose))
        {
            ROS_ERROR("test_active_exploration_server_robot : could not get the pose");
            return EXIT_FAILURE;
        }
        // Add camera height
        pose[2] += camera_height;

        // Get octree
        octomap_msgs::Octomap oc_msg;
        if (!get_octree(cloud_msg, pose, tree_resolution, oc_msg))
        {
            ROS_ERROR("test_active_exploration_server_robot : could not get the octree");
            return EXIT_FAILURE;
        }

        // Segment  --  CHANGE TO ANY SEGMENTATION THAT RETURNS STRUCTURE vector<std_msgs::Int32MultiArray>
        seg_init_srv.request.cloud = *scene;
        if (!seg_init_client.call(seg_init_srv))
        {
            ROS_ERROR("test_active_exploration_server_robot : could not call the segmentation initialisation service");
            return EXIT_FAILURE;
        }
        // Segment once
        vector<std_msgs::Int32MultiArray> clusters_indices;
        vector<vector<int> > segs;
        while (seg_client.call(seg_srv))
        {
            vector<int> s = get_valid_indices(seg_srv, cloud);
            if (s.size() > 0)
            {
                segs.push_back(s);
            }
        }
        // Set the cluster indices
        clusters_indices.resize(segs.size());
        for (size_t i = 0; i < segs.size(); ++i)
        {
            clusters_indices[i].data = segs[i];
        }
        ROS_INFO("test_active_exploration_server_robot : successfully segmented the scene");

        // Classify  --  CHANGE TO ANY CLASSIFICATION THAT RETURNS STRUCTURE vector<squirrel_object_perception_msgs::Classification>
        vector<squirrel_object_perception_msgs::Classification> class_results;
        if (working_classifier)
        {
            classify_srv.request.cloud = cloud_msg;
            classify_srv.request.clusters_indices = clusters_indices;
            if (!classify_client.call(classify_srv))
            {
                ROS_ERROR("test_active_exploration_server_robot : could not call the classification service");
                return EXIT_FAILURE;
            }
            class_results = classify_srv.response.class_results;
        }
        else
        {
            // Fake classification because it does not seem to work
            ROS_WARN("test_active_exploration_server_robot : fake classification!");
            for (size_t i = 0; i < clusters_indices.size(); ++i)
            {
                squirrel_object_perception_msgs::Classification c;
                std_msgs::String str;
                str.data = "apple/";
                c.class_type.push_back(str);
                str.data = "bottle/";
                c.class_type.push_back(str);
                str.data = "spray_bottle/";
                c.class_type.push_back(str);
                c.confidence.push_back(0.2);
                c.confidence.push_back(0.3);
                c.confidence.push_back(0.5);
                str.data = train_dir + "apple//3a92a256ad1e060ec048697b91f69d2/esf/pose_0.txt";
                c.pose.push_back(str);
                str.data = train_dir + "bottle//1cf98e5b6fff5471c8724d5673a063a6/esf/pose_0.txt";
                c.pose.push_back(str);
                str.data = train_dir + "spray_bottle//9b9a4bb5550f00ea586350d6e78ecc7/esf/pose_0.txt";
                c.pose.push_back(str);
                class_results.push_back(c);
            }
        }

        // Set the fields in the service
        nbv_srv.request.robot_pose.position.x = pose[0];
        nbv_srv.request.robot_pose.position.y = pose[1];
        nbv_srv.request.robot_pose.position.z = pose[2];
        nbv_srv.request.variance = variance;
        nbv_srv.request.occlusions = occlusions;
        nbv_srv.request.cloud = cloud_msg;
        nbv_srv.request.map = oc_msg;
        nbv_srv.request.clusters_indices = clusters_indices;
        nbv_srv.request.class_results = class_results;
        nbv_srv.request.camera_height = camera_height;
        nbv_srv.request.robot_radius = robot_radius;
        nbv_srv.request.distance_from_center = distance_from_center;
        nbv_srv.request.num_locations = num_locations;

        // Call service
        ROS_INFO("test_active_exploration_server_robot : calling next best view service");
        if (!nbv_client.call(nbv_srv))
        {
            ROS_ERROR("test_active_exploration_server_robot : could not call the next best view service");
            return EXIT_FAILURE;
        }
        // Print out the best index
        cout << endl;
        ROS_INFO("Next best view index is %i", nbv_srv.response.nbv_ix);
        // Print out the locations and their utilities
        cout << "locations and utilities:" << endl;
        for (size_t i = 0; i < nbv_srv.response.generated_locations.size(); ++i)
        {
            cout << "[" << nbv_srv.response.generated_locations[i].x << " "
                 << nbv_srv.response.generated_locations[i].y << " "
                 << nbv_srv.response.generated_locations[i].z << "] -> "
                 << nbv_srv.response.utilities[i] << endl;
        }

        // Publish markers for inspection
        publish_markers(n, nbv_srv.response.generated_locations, nbv_srv.response.nbv_ix);

        // -- ROBOT MOVES TO WAYPOINT AND THIS REPEATS
        ROS_WARN("Robot moves to next waypoint ...");
        cout << "Hit enter when ready to make next observation" << endl;
        cin.ignore();
    }

    // End
    ros::shutdown();
    return EXIT_SUCCESS;
}

PointCloud<PointT> transform_cloud(sensor_msgs::PointCloud2ConstPtr scene)
{
    // Transform
    tf::StampedTransform stamped_transform;
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform(_CAMERA_FRAME, _MAP, ros::Time(0), ros::Duration(5.0));
    tf_listener.lookupTransform (_CAMERA_FRAME, _MAP, ros::Time(0), stamped_transform);
    tf::Vector3 t = stamped_transform.getOrigin();
    tf::Matrix3x3 r = stamped_transform.getBasis();
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf(0,0) = r[0][0];
    tf(0,1) = r[0][1];
    tf(0,2) = r[0][2];
    tf(0,3) = t[0];
    tf(1,0) = r[1][0];
    tf(1,1) = r[1][1];
    tf(1,2) = r[1][2];
    tf(1,3) = t[1];
    tf(2,0) = r[2][0];
    tf(2,1) = r[2][1];
    tf(2,2) = r[2][2];
    tf(2,3) = t[2];
    Eigen::Matrix4f transform;
    // Invert the transform
    transform = tf.inverse();
    // Transform the point cloud
    PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*scene, pcl_pc2);
    PointCloud<PointT> cloud;
    fromPCLPointCloud2(pcl_pc2, cloud);
    transformPointCloud(cloud, cloud, transform);
    int nan_count = 0, valid_count = 0;
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        if (isnan(cloud.points[i].x))
            nan_count++;
        else
            valid_count++;
    }
    ROS_INFO("get_cloud : read cloud -> total = %lu, nan = %i, valid = %i", cloud.size(), nan_count, valid_count);

    return cloud;
}

bool get_pose(Eigen::Vector4f &pose)
{
    PointCloud<PointT> robot_pos;
    robot_pos.resize(1);
    robot_pos.points[0].x = 0.0;
    robot_pos.points[0].y = 0.0;
    robot_pos.points[0].z = 0.0;
    PointCloud<PointT> robot_map_pos;
    if (!frame_to_frame(_BASE, _MAP, robot_pos, robot_map_pos))
    {
        ROS_ERROR("get_pose : could not transform robot location to map frame");
        return false;
    }
    else
    {
        ROS_INFO("get_pose : robot location is [%.2f, %.2f, %.2f]",
                 robot_map_pos.points[0].x, robot_map_pos.points[0].y, robot_map_pos.points[0].z);
        pose[0] = robot_map_pos.points[0].x;
        pose[1] = robot_map_pos.points[0].y;
        pose[2] = robot_map_pos.points[0].z;
        pose[3] = 0.0;
    }

    return true;
}

bool get_octree(const sensor_msgs::PointCloud2 &cloud_msg, const Eigen::Vector4f &pose, const double &tree_resolution,
                octomap_msgs::Octomap &oc_msg)
{
    // Create an octree with the point cloud input
    OcTree tree (tree_resolution);
    // Add the point cloud
    // Otherwise insert the point cloud
    point3d pos (pose[0], pose[1], pose[2]);
    octomap::Pointcloud o_cloud;
    pointCloud2ToOctomap(cloud_msg, o_cloud);
    tree.insertPointCloud(o_cloud, pos);

    // Octomap - Only works with binary conversion!
    if (!octomap_msgs::binaryMapToMsg(tree, oc_msg))
    {
        ROS_ERROR("get_octree : could not convert the octomap");
        return false;
    }

    return true;
}

vector<int> get_valid_indices(const squirrel_object_perception_msgs::SegmentOnce &seg_srv, const PointCloud<PointT> &cloud)
{
    vector<int> indices;
    // Check if has data
    if (seg_srv.response.clusters_indices[0].data.size() == 0)
    {
        ROS_WARN("get_valid_indices : segment is empty");
        return indices;
    }
    // Check has valid points
    vector<int> ss = seg_srv.response.clusters_indices[0].data;
    PointCloud<PointT> seg_cloud;
    copyPointCloud(cloud, ss, seg_cloud);
    ss.clear();
    // For each point
    for (size_t i = 0; i < seg_cloud.size(); ++i)
    {
        if (!isnan(seg_cloud.points[i].x) && !isnan(seg_cloud.points[i].y) && !isnan(seg_cloud.points[i].z))
        {
            indices.push_back(seg_srv.response.clusters_indices[0].data[i]);
        }
    }
    return indices;
}

void publish_markers(ros::NodeHandle &node_handle, const vector<geometry_msgs::Point> &locations, const int &nbv_ix)
{
    // Publish the markers
    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::MarkerArray>(_MARKER_NAME, 1000);
    visualization_msgs::MarkerArray marker_array_msg;

    marker_array_msg.markers.resize(locations.size()+1);
    for (size_t i = 0; i < locations.size(); ++i)
    {
        marker_array_msg.markers[i].header.frame_id = "map";
        marker_array_msg.markers[i].header.stamp = ros::Time();
        marker_array_msg.markers[i].ns = "test_active_exploration_server";
        marker_array_msg.markers[i].id = i;
        marker_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array_msg.markers[i].pose.position.x = locations[i].x;
        marker_array_msg.markers[i].pose.position.y = locations[i].y;
        marker_array_msg.markers[i].pose.position.z = locations[i].z;
        marker_array_msg.markers[i].pose.orientation.x = 0.0;
        marker_array_msg.markers[i].pose.orientation.y = 0.0;
        marker_array_msg.markers[i].pose.orientation.z = 0.0;
        marker_array_msg.markers[i].pose.orientation.w = 1.0;
        marker_array_msg.markers[i].scale.x = 0.8;
        marker_array_msg.markers[i].scale.y = 0.8;
        marker_array_msg.markers[i].scale.z = 0.8;
        marker_array_msg.markers[i].color.a = 1.0;
        marker_array_msg.markers[i].color.r = 0.0;
        marker_array_msg.markers[i].color.g = 1.0;  // Green
        marker_array_msg.markers[i].color.b = 0.0;
    }
    // Add best location
    int sz = locations.size();
    marker_array_msg.markers[sz].header.frame_id = "map";
    marker_array_msg.markers[sz].header.stamp = ros::Time();
    marker_array_msg.markers[sz].ns = "test_active_exploration_server";
    marker_array_msg.markers[sz].id = locations.size();
    marker_array_msg.markers[sz].type = visualization_msgs::Marker::SPHERE;
    marker_array_msg.markers[sz].action = visualization_msgs::Marker::ADD;
    marker_array_msg.markers[sz].pose.position.x = locations[nbv_ix].x;
    marker_array_msg.markers[sz].pose.position.y = locations[nbv_ix].y;
    marker_array_msg.markers[sz].pose.position.z = locations[nbv_ix].z;
    marker_array_msg.markers[sz].pose.orientation.x = 0.0;
    marker_array_msg.markers[sz].pose.orientation.y = 0.0;
    marker_array_msg.markers[sz].pose.orientation.z = 0.0;
    marker_array_msg.markers[sz].pose.orientation.w = 1.0;
    marker_array_msg.markers[sz].scale.x = 1.0;
    marker_array_msg.markers[sz].scale.y = 1.0;
    marker_array_msg.markers[sz].scale.z = 1.0;
    marker_array_msg.markers[sz].color.a = 1.0;
    marker_array_msg.markers[sz].color.r = 1.0;  // Pink
    marker_array_msg.markers[sz].color.g = 0.0;
    marker_array_msg.markers[sz].color.b = 1.0;

    marker_pub.publish(marker_array_msg);
    ros::spinOnce();
}
