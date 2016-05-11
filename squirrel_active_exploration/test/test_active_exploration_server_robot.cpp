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
#define _ROBOT_HEIGHT 0.75
#define _ROBOT_RADIUS 0.22
#define _DISTANCE_FROM_CENTER 2
#define _NUM_LOCATIONS 10
#define _TREE_RESOLUTION 0.1
#define _BASE "/base_link"
#define _MAP "/map"

// Run Main
int main(int argc, char **argv)
{
    ros::init (argc, argv ,"test_active_exploration_server");
    ros::NodeHandle n("~");

    // Get the parameters
    double variance = _VARIANCE;
    double robot_height = _ROBOT_HEIGHT;
    double robot_radius = _ROBOT_RADIUS;
    double distance_from_center = _DISTANCE_FROM_CENTER;
    int num_locations = _NUM_LOCATIONS;
    double tree_resolution = _TREE_RESOLUTION;
    bool occlusions = true;
    // Read the input if it exists
    n.getParam ("variance", variance);
    n.getParam ("robot_height", robot_height);
    n.getParam ("robot_radius", robot_radius);
    n.getParam ("distance_from_center", distance_from_center);
    n.getParam ("num_locations", num_locations);
    n.getParam ("tree_resolution", tree_resolution);
    n.getParam ("occlusions", occlusions);
    // Print out the input
    ROS_INFO("test_active_exploration_server : input parameters");
    cout << "Variance = " << variance << endl;
    cout << "Robot height = " << robot_height << endl;
    cout << "Robot radius = " << robot_radius << endl;
    cout << "Distance from center = " << distance_from_center << endl;
    cout << "Num locations = " << num_locations << endl;
    cout << "Tree resolution = " << tree_resolution << endl;
    if (occlusions)
        cout << "Occlusions = TRUE" << endl;
    else
        cout << "Occlusions = FALSE" << endl;

    // To pass to the service
    Eigen::Vector4f pose;
    sensor_msgs::PointCloud2 cloud_msg;

    // Get the point cloud
    sensor_msgs::PointCloud2ConstPtr scene = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points",
                                                                                                  n, ros::Duration(50));
    // Transform
    tf::StampedTransform stamped_transform;
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("/kinect_depth_optical_frame", "/map", ros::Time(0), ros::Duration(5.0));
    tf_listener.lookupTransform ("/kinect_depth_optical_frame", "/map", ros::Time(0), stamped_transform);
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
    transform = tf.inverse();  // inverse the transform
    // Transform the point cloud
    PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*scene, pcl_pc2);
    PointCloud<PointT> cloud;
    fromPCLPointCloud2(pcl_pc2, cloud);
    transformPointCloud(cloud, cloud, transform);
    // Save point cloud
    string f = "/home/squirrel/tim_cloud.pcd";
    io::savePCDFileBinary (f, cloud);
    cout << "The points in the cloud" << endl;
    int nan_count = 0, valid_count = 0;
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        //cout << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << endl;
        if (isnan(cloud.points[i].x))
            nan_count++;
        else
            valid_count++;
    }
    cout << "total = " << cloud.size() << ", nan = " << nan_count << ", valid = " << valid_count << endl;
    // Convert back to ros message
    pcl::toROSMsg(cloud, cloud_msg);

    // Get the pose
    PointCloud<PointT> robot_pos;
    robot_pos.resize(1);
    robot_pos.points[0].x = 0.0;
    robot_pos.points[0].y = 0.0;
    robot_pos.points[0].z = 0.0;
    PointCloud<PointT> robot_map_pos;
    if (!frame_to_frame(_BASE, _MAP, robot_pos, robot_map_pos))
    {
        ROS_ERROR("test_active_exploration_server : could not transform robot location to map frame");
        return EXIT_FAILURE;
    }
    else
    {
        ROS_INFO("test_active_exploration_server : robot location is [%.2f, %.2f, %.2f]",
                 robot_map_pos.points[0].x, robot_map_pos.points[0].y, robot_map_pos.points[0].z);
        pose[0] = robot_map_pos.points[0].x;
        pose[1] = robot_map_pos.points[0].y;
        pose[2] = robot_map_pos.points[0].z + robot_height;  // Adding height here?
        pose[3] = 0.0;
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

    // Create an octree with the point cloud input
    OcTree tree (tree_resolution);
    // Add the point cloud
    // Otherwise insert the point cloud
    point3d pos (pose[0], pose[1], pose[2]);
    octomap::Pointcloud o_cloud;
    pointCloud2ToOctomap(cloud_msg, o_cloud);
    tree.insertPointCloud(o_cloud, pos);
    // Save octomap
    tree.writeBinary("/home/squirrel/tim_tree.bt");

    // Set the fields in the service
    nbv_srv.request.camera_pose.position.x = pose[0];
    nbv_srv.request.camera_pose.position.y = pose[1];
    nbv_srv.request.camera_pose.position.z = pose[2];
    nbv_srv.request.variance = variance;
    //nbv_srv.request.cloud = cloud_msg;  // CHANGED!
    nbv_srv.request.occlusions = occlusions;
    // Octomap - Only works with binary conversion!
    octomap_msgs::Octomap oc_msg;
    if (!octomap_msgs::binaryMapToMsg(tree, oc_msg))
    {
        ROS_ERROR("test_active_exploration_server : could not convert the octomap");
        return EXIT_FAILURE;
    }
    nbv_srv.request.map = oc_msg;

    // Convert octomap to cloud
    PointCloud<PointT> oc_cloud = octree_to_cloud(tree);
    PointT pt;
    pt.x = pose[0];
    pt.y = pose[1];
    pt.z = pose[2];
    oc_cloud.push_back(pt);
    f = "/home/squirrel/tim_oc_cloud.pcd";
    io::savePCDFileBinary (f, oc_cloud);

    // This is how the segmentation and classification SHOULD work
    sensor_msgs::Image in_image;
    // Load the image
    string image_file = "/home/squirrel/catkin_ws/src/squirrel_perception/squirrel_active_exploration/data/test45.png";
    cv::Mat image = cv::imread(image_file,-1);
    cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    // Convert OpenCV image to ROS message
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "saliency_map";
    cv_ptr->encoding = "mono8";
    cv_ptr->image = image;
    cv_ptr->toImageMsg(in_image);
    // Segment initialisation
    //seg_init_srv.request.saliency_map = in_image;
    seg_init_srv.request.cloud = *scene;
    if (!seg_init_client.call(seg_init_srv))
    {
        ROS_ERROR("test_active_exploration_server : could not call the segmentation initialisation service");
        return EXIT_FAILURE;
    }
    // Segment once
    PointCloud<PointT> segment_cloud;
    nbv_srv.request.clusters_indices.clear();
    vector<vector<int> > segs;
    int s = 0;
    while (seg_client.call(seg_srv))
    {
        // Get the points for the segment
        pcl_conversions::toPCL(seg_srv.response.points[0], pcl_pc2);
        PointCloud<PointT> pc;
        fromPCLPointCloud2(pcl_pc2, pc);
        transformPointCloud(pc, pc, transform);
        // Save point cloud
        if (pc.size() > 0)
        {
            segment_cloud.insert(segment_cloud.end(), pc.begin(), pc.end());
            vector<int> ss;
            int start_index = segment_cloud.size();
            int count = 0;
            for (size_t i = 0; i < pc.size(); ++i)
            {
                if (!isnan(pc.points[i].x))
                    ss.push_back(start_index + count);
            }
            if (ss.size() > 0)
            {
                segs.push_back(ss);
                f = "/home/squirrel/tim_seg_cloud_" + boost::lexical_cast<string>(s) + ".pcd";
                io::savePCDFileBinary (f, pc);
            }
            else
            {
                ROS_WARN("Segment %i has no valid points", s);
            }
        }
        else
        {
            ROS_WARN("Segment %i is empty", s);
        }
        ++s;

//        if (seg_srv.response.clusters_indices[0].data.size() > 0)
//        {
//            vector<int> ss = seg_srv.response.clusters_indices[0].data;
//            PointCloud<PointT> seg_cloud;
//            copyPointCloud(cloud, ss, seg_cloud);
//            cout << "Points in segment " << s << ":" << endl;
//            nan_count = 0;
//            valid_count = 0;
//            for (size_t i = 0; i < seg_cloud.size(); ++i)
//            {
//                //cout << seg_cloud.points[i].x << " " << seg_cloud.points[i].y << " " << seg_cloud.points[i].z << endl;
//                if (isnan(seg_cloud.points[i].x))
//                    nan_count++;
//                else
//                    valid_count++;
//            }
//            cout << "total = " << seg_cloud.size() << ", nan = " << nan_count << ", valid = " << valid_count << endl;
//            if (valid_count > 0)
//            {
//                segs.push_back(ss);
//                f = "/home/squirrel/tim_seg_cloud_" + boost::lexical_cast<string>(s) + ".pcd";
//                io::savePCDFileBinary (f, seg_cloud);
//            }
//            else
//            {
//                ROS_WARN("Segment %i has no valid points", s);
//            }
//        }
//        else
//        {
//            ROS_WARN("Segment %i is empty", s);
//        }
//        ++s;
    }

    nbv_srv.request.clusters_indices.resize(segs.size());
    for (size_t i = 0; i < segs.size(); ++i)
    {
        nbv_srv.request.clusters_indices[i].data = segs[i];
    }

    ROS_INFO("test_active_exploration_server : successfully segmented the scene");
    cin.ignore();


//    // Classify
//    classify_srv.request.cloud = cloud_msg;
//    classify_srv.request.clusters_indices = seg_srv.response.clusters_indices;
//    if (!classify_client.call(classify_srv))
//    {
//        ROS_ERROR("test_active_exploration_server::main : could not call the classification service");
//        return EXIT_FAILURE;
//    }
//    nbv_srv.request.class_results = classify_srv.response.class_results;

    // Fake classification because it does not seem to work
    ROS_WARN("test_active_exploration_server : fake classification!");
    vector<squirrel_object_perception_msgs::Classification> class_results;
    string squirrel_dir = "/home/squirrel/tim_data/training_set_3/training/";
    for (size_t i = 0; i < segs.size(); ++i)
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
        str.data = squirrel_dir + "apple//3a92a256ad1e060ec048697b91f69d2/esf/pose_0.txt";
        c.pose.push_back(str);
        str.data = squirrel_dir + "bottle//1cf98e5b6fff5471c8724d5673a063a6/esf/pose_0.txt";
        c.pose.push_back(str);
        str.data = squirrel_dir + "spray_bottle//9b9a4bb5550f00ea586350d6e78ecc7/esf/pose_0.txt";
        c.pose.push_back(str);
        class_results.push_back(c);
    }
    nbv_srv.request.class_results = class_results;

    // Set the cloud to match the segments
    //pcl::toROSMsg(segment_cloud, cloud_msg);
    nbv_srv.request.cloud = cloud_msg;


//    // --- Test 1: given locations
//    if (multiple_locations)
//    {
//        vector<geometry_msgs::Point> locations;
//        //for (size_t i = 0; i < poses.size(); ++i)
//        for (size_t i = 0; i < 4; ++i)
//        {
//            geometry_msgs::Point p;
//            p.x = poses[i][0];
//            p.y = poses[i][1];
//            p.z = poses[i][2];
//            locations.push_back(p);
//        }
//        nbv_srv.request.locations = locations;
//        // Call the service
//        ROS_INFO("test_active_exploration_server : calling next best view service with given locations");
//        if (!nbv_client.call(nbv_srv))
//        {
//            ROS_ERROR("test_active_exploration_server : could not call the next best view service");
//            return EXIT_FAILURE;
//        }
//        // Print out the best index
//        cout << endl;
//        ROS_INFO("Next best view index is %i", nbv_srv.response.nbv_ix);
//        // Print out the locations and their utilities
//        cout << "locations and utilities:" << endl;
//        for (size_t i = 0; i < nbv_srv.response.generated_locations.size(); ++i)
//        {
//            cout << "[" << nbv_srv.response.generated_locations[i].x << " "
//                 << nbv_srv.response.generated_locations[i].y << " "
//                 << nbv_srv.response.generated_locations[i].z << "] -> "
//                 << nbv_srv.response.utilities[i] << endl;
//        }
//    }

    // --- Test 2: without locations (they must be generated)
    nbv_srv.request.locations.clear();
    nbv_srv.request.robot_height = robot_height;
    nbv_srv.request.robot_radius = robot_radius;
    nbv_srv.request.distance_from_center = distance_from_center;
    nbv_srv.request.num_locations = num_locations;
    ROS_INFO("test_active_exploration_server : calling next best view service without given locations");
    if (!nbv_client.call(nbv_srv))
    {
        ROS_ERROR("test_active_exploration_server : could not call the next best view service");
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
    // Publish the markers
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("active_exploration_waypoints", 1000);
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array_msg;

    marker_array_msg.markers.resize(nbv_srv.response.generated_locations.size()+1);
    for (size_t i = 0; i < nbv_srv.response.generated_locations.size(); ++i)
    {
        marker_array_msg.markers[i].header.frame_id = "map";
        marker_array_msg.markers[i].header.stamp = ros::Time();
        marker_array_msg.markers[i].ns = "test_active_exploration_server";
        marker_array_msg.markers[i].id = i;
        marker_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array_msg.markers[i].pose.position.x = nbv_srv.response.generated_locations[i].x;
        marker_array_msg.markers[i].pose.position.y = nbv_srv.response.generated_locations[i].y;
        marker_array_msg.markers[i].pose.position.z = nbv_srv.response.generated_locations[i].z;
        marker_array_msg.markers[i].pose.orientation.x = 0.0;
        marker_array_msg.markers[i].pose.orientation.y = 0.0;
        marker_array_msg.markers[i].pose.orientation.z = 0.0;
        marker_array_msg.markers[i].pose.orientation.w = 1.0;
        marker_array_msg.markers[i].scale.x = 0.8;
        marker_array_msg.markers[i].scale.y = 0.8;
        marker_array_msg.markers[i].scale.z = 0.8;
        marker_array_msg.markers[i].color.a = 1.0;
        marker_array_msg.markers[i].color.r = 0.0;
        marker_array_msg.markers[i].color.g = 1.0;
        marker_array_msg.markers[i].color.b = 0.0;
    }
    // Add best location
    int sz = nbv_srv.response.generated_locations.size();
    marker_array_msg.markers[sz].header.frame_id = "map";
    marker_array_msg.markers[sz].header.stamp = ros::Time();
    marker_array_msg.markers[sz].ns = "test_active_exploration_server";
    marker_array_msg.markers[sz].id = nbv_srv.response.generated_locations.size();
    marker_array_msg.markers[sz].type = visualization_msgs::Marker::SPHERE;
    marker_array_msg.markers[sz].action = visualization_msgs::Marker::ADD;
    marker_array_msg.markers[sz].pose.position.x = nbv_srv.response.generated_locations[nbv_srv.response.nbv_ix].x;
    marker_array_msg.markers[sz].pose.position.y = nbv_srv.response.generated_locations[nbv_srv.response.nbv_ix].y;
    marker_array_msg.markers[sz].pose.position.z = nbv_srv.response.generated_locations[nbv_srv.response.nbv_ix].z;
    marker_array_msg.markers[sz].pose.orientation.x = 0.0;
    marker_array_msg.markers[sz].pose.orientation.y = 0.0;
    marker_array_msg.markers[sz].pose.orientation.z = 0.0;
    marker_array_msg.markers[sz].pose.orientation.w = 1.0;
    marker_array_msg.markers[sz].scale.x = 1.0;
    marker_array_msg.markers[sz].scale.y = 1.0;
    marker_array_msg.markers[sz].scale.z = 1.0;
    marker_array_msg.markers[sz].color.a = 1.0;
    marker_array_msg.markers[sz].color.r = 1.0;
    marker_array_msg.markers[sz].color.g = 0.0;
    marker_array_msg.markers[sz].color.b = 1.0;

    while(ros::ok)
    {
        pub_marker.publish(marker_array_msg);
        ros::spinOnce();
    }


    // Send the next best location as a waypoint
    // TODO

    // End
    ros::shutdown();
    return EXIT_SUCCESS;
}
