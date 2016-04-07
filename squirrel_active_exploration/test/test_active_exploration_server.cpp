#include <ros/ros.h>
#include <octomap_ros/conversions.h>
#include <squirrel_object_perception_msgs/Segment.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/ActiveExplorationNBV.h>
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

// Function declarations
bool load_data(const string &data_name, const bool &reverse_transforms, vector<Eigen::Vector4f> &poses,
               vector<PointCloud<PointT> > &clouds, vector<Eigen::Matrix4f> &transforms, vector<vector<vector<int> > > &indices);

// Run Main
int main(int argc, char **argv)
{
    ros::init (argc, argv ,"test_active_exploration_server");
    ros::NodeHandle n("~");

    // Get the parameters
    string data_name = "";
    double variance = _VARIANCE;
    double robot_height = _ROBOT_HEIGHT;
    double robot_radius = _ROBOT_RADIUS;
    double distance_from_center = _DISTANCE_FROM_CENTER;
    int num_locations = _NUM_LOCATIONS;
    double tree_resolution = _TREE_RESOLUTION;
    bool reverse_transforms = false;
    bool occlusions = true;
    bool multiple_locations = false;
    // Read the input if it exists
    if (!n.getParam ("data_name", data_name))
    {
        ROS_ERROR("test_active_exploration_server::main : you must enter a filename!");
        return EXIT_FAILURE;
    }
    n.getParam ("variance", variance);
    n.getParam ("robot_height", robot_height);
    n.getParam ("robot_radius", robot_radius);
    n.getParam ("distance_from_center", distance_from_center);
    n.getParam ("num_locations", num_locations);
    n.getParam ("tree_resolution", tree_resolution);
    n.getParam ("reverse_transforms", reverse_transforms);
    n.getParam ("occlusions", occlusions);
    // Print out the input
    ROS_INFO("test_active_exploration_server : input parameters");
    cout << "Data name = " << data_name << endl;
    cout << "Variance = " << variance << endl;
    cout << "Robot height = " << robot_height << endl;
    cout << "Robot radius = " << robot_radius << endl;
    cout << "Distance from center = " << distance_from_center << endl;
    cout << "Num locations = " << num_locations << endl;
    cout << "Tree resolution = " << tree_resolution << endl;
    if (reverse_transforms)
        cout << "Reverse transforms = TRUE" << endl;
    else
        cout << "Reverse transforms = FALSE" << endl;
    if (occlusions)
        cout << "Occlusions = TRUE" << endl;
    else
        cout << "Occlusions = FALSE" << endl;

    // Load the clouds from the file
    vector<Eigen::Vector4f> poses;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<vector<vector<int> > > indices;
    if (!load_data(data_name, reverse_transforms, poses, clouds, transforms, indices))
    {
        ROS_ERROR("test_active_exploration_server::main : could not load the data");
        return EXIT_FAILURE;
    }
    // Get a single pose, cloud and transform
    Eigen::Vector4f pose;
    PointCloud<PointT> cloud;
    Eigen::Matrix4f transform;
    vector<vector<int> > segs;
    if (poses.size() == 0)
    {
        ROS_ERROR("test_active_exploration_server::main : iposes is empty");
        return EXIT_FAILURE;
    }
    else if (poses.size() == 1)
    {
        pose = poses[0];
        cloud = clouds[0];
        transform = transforms[0];
        segs = indices[0];
    }
    else
    {
        multiple_locations = true;
        int r = int_rand(0, poses.size()-1);
        if (r < 0 || r >= poses.size())
        {
            ROS_ERROR("test_active_exploration_server::main : invalid random number %i for poses of size %lu", r, poses.size());
            return EXIT_FAILURE;
        }
        // Otherwise get the elements
        pose = poses[r];
        cloud = clouds[r];
        transform = transforms[r];
        segs = indices[r];
    }

    // Transform the point cloud
    transformPointCloud(cloud, cloud, transform);
    // Get the pose from the transformed point cloud
    pose = extract_camera_position (cloud);
    pose = transform_eigvec(pose, transform);

    // Create the service clients
    ros::ServiceClient nbv_client = n.serviceClient<squirrel_object_perception_msgs::ActiveExplorationNBV>("/squirrel_active_exploration");
    squirrel_object_perception_msgs::ActiveExplorationNBV nbv_srv;
    ros::ServiceClient seg_client = n.serviceClient<squirrel_object_perception_msgs::Segment>("/squirrel_segmentation");;
    squirrel_object_perception_msgs::Segment seg_srv;
    ros::ServiceClient classify_client = n.serviceClient<squirrel_object_perception_msgs::Classify>("/squirrel_classify");
    squirrel_object_perception_msgs::Classify classify_srv;

    // Convert the point cloud to a ros message type
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    // Create an octree with the point cloud input
    OcTree tree (tree_resolution);
    // Add the point cloud
    // Otherwise insert the point cloud
    point3d pos (pose[0], pose[1], pose[2]);
    octomap::Pointcloud o_cloud;
    pointCloud2ToOctomap(cloud_msg, o_cloud);
    tree.insertPointCloud(o_cloud, pos);

    // Set the fields in the service
    nbv_srv.request.camera_pose.position.x = pose[0];
    nbv_srv.request.camera_pose.position.y = pose[1];
    nbv_srv.request.camera_pose.position.z = pose[2];
    nbv_srv.request.variance = variance;
    nbv_srv.request.cloud = cloud_msg;
    nbv_srv.request.occlusions = occlusions;
    // Octomap - Only works with binary conversion!
//    vector<int8_t> map_data;
//    if (!octomap_msgs::fullMapToMsgData(tree, map_data))
//    {
//        ROS_ERROR("test_active_exploration_server::main : could not convert the octomap");
//        return EXIT_FAILURE;
//    }
//    nbv_srv.request.map.id = "OcTree";  // always!
//    nbv_srv.request.map.binary = false; // used full map conversion
//    nbv_srv.request.map.resolution = tree_resolution;
//    nbv_srv.request.map.data = map_data;
    octomap_msgs::Octomap oc_msg;
    if (!octomap_msgs::binaryMapToMsg(tree, oc_msg))
    {
        ROS_ERROR("test_active_exploration_server::main : could not convert the octomap");
        return EXIT_FAILURE;
    }
    nbv_srv.request.map = oc_msg;

    // This is how the segmentation and classification SHOULD work
//    // Segment
//    seg_srv.request.cloud = cloud_msg;
//    if (!seg_client.call(seg_srv))
//    {
//        ROS_ERROR("test_active_exploration_server::main : could not call the segmentation service");
//        return EXIT_FAILURE;
//    }
//    nbv_srv.request.clusters_indices = seg_srv.response.clusters_indices;
//    // Classify
//    classify_srv.request.cloud = cloud_msg;
//    classify_srv.request.clusters_indices = seg_srv.response.clusters_indices;
//    if (!classify_client.call(classify_srv))
//    {
//        ROS_ERROR("test_active_exploration_server::main : could not call the classification service");
//        return EXIT_FAILURE;
//    }
//    nbv_srv.request.class_results = classify_srv.response.class_results;

    // Fake segmentation and classification because they do not seem to work
    vector<std_msgs::Int32MultiArray> clusters_indices;
    vector<squirrel_object_perception_msgs::Classification> class_results;
    string squirrel_dir = "/home/tpat8946/ros_ws/squirrel/src/squirrel_perception/squirrel_active_exploration/data/training_set_3/training/";
    for (size_t i = 0; i < segs.size(); ++i)
    {
        std_msgs::Int32MultiArray s;
        s.data = segs[i];
        clusters_indices.push_back(s);
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
    nbv_srv.request.clusters_indices = clusters_indices;
    nbv_srv.request.class_results = class_results;


    // --- Test 1: given locations
    if (multiple_locations)
    {
        vector<geometry_msgs::Point> locations;
        //for (size_t i = 0; i < poses.size(); ++i)
        for (size_t i = 0; i < 4; ++i)
        {
            geometry_msgs::Point p;
            p.x = poses[i][0];
            p.y = poses[i][1];
            p.z = poses[i][2];
            locations.push_back(p);
        }
        nbv_srv.request.locations = locations;
        // Call the service
        ROS_INFO("test_active_exploration_server::main : calling next best view service with given locations");
        if (!nbv_client.call(nbv_srv))
        {
            ROS_ERROR("test_active_exploration_server::main : could not call the next best view service");
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
    }

    // --- Test 2: without locations (they must be generated)
    nbv_srv.request.locations.clear();
    nbv_srv.request.robot_height = robot_height;
    nbv_srv.request.robot_radius = robot_radius;
    nbv_srv.request.distance_from_center = distance_from_center;
    nbv_srv.request.num_locations = num_locations;
    ROS_INFO("test_active_exploration_server::main : calling next best view service without given locations");
    if (!nbv_client.call(nbv_srv))
    {
        ROS_ERROR("test_active_exploration_server::main : could not call the next best view service");
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

    // End
    ros::shutdown();
    return EXIT_SUCCESS;
}

bool load_data(const string &data_name, const bool &reverse_transforms, vector<Eigen::Vector4f> &poses,
               vector<PointCloud<PointT> > &clouds, vector<Eigen::Matrix4f> &transforms, vector<vector<vector<int> > > &indices)
{
    // Clear the vectors
    poses.clear();
    clouds.clear();
    transforms.clear();
    indices.clear();

    // If this is a single file, then load it
    if (data_name.size() > 4 && data_name.substr(data_name.size()-4) == ".pcd")
    {
        ROS_INFO("test_active_exploration_server::load_data : single file %s", data_name.c_str());
        // Single point cloud input
        PointCloud<PointT> cloud;
        if (io::loadPCDFile<PointT> (data_name.c_str(), cloud) == -1)
        {
            ROS_ERROR("test_active_exploration_server::load_data : could not load point cloud %s", data_name.c_str());
            return false;
        }
        // Get the directory from the filename
        string cloud_path_name;
        string cloud_filename;
        if (!split_filename(data_name, cloud_path_name, cloud_filename))
        {
            ROS_ERROR("test_active_exploration_server::load_data : could not get path and filename from string %s", data_name.c_str());
            return false;
        }
        // Get the transform
        Eigen::Vector4f pose;
        PointCloud<PointT> transformed_cloud;
        Eigen::Matrix4f transform;
        if (!transform_cloud_from_file(cloud_path_name, cloud_filename, cloud, transformed_cloud, pose, transform))
        {
            ROS_ERROR("test_active_exploration_server::load_data : error transforming point cloud from file %s", data_name.c_str());
            return false;
        }
        else
        {
            // Add to vectors
            poses.push_back(pose);
            clouds.push_back(cloud);
            // Reverse transform
            Eigen::Matrix4f transform_inv = transform;
            if (reverse_transforms)
                transform_inv = transform.inverse();
            transform = transform_inv;
            transforms.push_back(transform);
        }
        // Get the segment indices
        // First get the index number of the point cloud
        // Get the dot
        size_t dot = data_name.find_last_of('.');
        if (dot == string::npos)
        {
            ROS_ERROR("test_active_exploration_server::load_data : could not read extension of file %s", cloud_filename.c_str());
            return false;
        }
        // Get the index name
        int ix = -1;
        size_t underscore = cloud_filename.find_last_of('_');
        if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
        {
            int str_len = dot - underscore;
            string str_ix = cloud_filename.substr(underscore+1,str_len-1);
            ix = atoi(str_ix.c_str());
        }
        else
        {
            ROS_ERROR("test_active_exploration_server::load_data : could not read the index in file %s", cloud_filename.c_str());
            return false;
        }
        // Append zeros to front
        string ix_str = boost::lexical_cast<string>(ix);
        while (ix_str.size() < 10)
            ix_str = "0" + ix_str;
        string segment_indices_str;
        vector<vector<int> > segment_indices;
        int seg_count = 0;
        // Load the segment indices for each segment associated to this cloud
        while (true)
        {
            // Create the count string
            string count_str = boost::lexical_cast<string>(seg_count);
            while (count_str.size() < 2)
                count_str = "0" + count_str;
            segment_indices_str = add_backslash(cloud_path_name) + _INDICES_PREFIX + count_str + "_" + ix_str + ".pcd";
            // If valid file then load it
            if (boost::filesystem::exists(segment_indices_str))
            {
                PointCloud<IndexPoint> in_cloud;
                if (io::loadPCDFile<IndexPoint>(segment_indices_str.c_str(), in_cloud) == -1)
                {
                    ROS_WARN("test_active_exploration_server::load_data : could not read index file");
                    break;
                }
                else
                {
                    // Append the point cloud indices
                    vector<int> in_indices;
                    in_indices.resize(in_cloud.points.size());
                    for (size_t j = 0; j < in_cloud.points.size(); ++j)
                        in_indices[j] = in_cloud.points[j].idx;
                    segment_indices.push_back(in_indices);
                }
            }
            // Otherwise finish
            else
            {
                break;
            }
            // Next segment
            ++seg_count;
        }
        indices.push_back(segment_indices);
    }
    // Otherwise it is a directory and must load all files in the directory
    else
    {
        if (!load_test_directory_with_segment_indices(data_name, reverse_transforms, poses, clouds, transforms, indices))
        {
            ROS_ERROR("test_active_exploration_server::load_data : could not load the data from the directory %s", data_name.c_str());
            return false;
        }
        // This can also work without getting the segment indices if segmentation was working
//        if (!load_test_directory(data_name, reverse_transforms, poses, clouds, transforms))
//        {
//            ROS_ERROR("test_active_exploration_server::load_data : could not load the data from the directory %s", data_name.c_str());
//            return false;
//        }
    }

    // Return success
    return true;
}
