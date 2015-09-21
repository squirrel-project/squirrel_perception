#include "squirrel_active_exploration/active_exploration.h"

using namespace std;
using namespace pcl;

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");

    ActiveExploration *exp (new ActiveExploration());
    exp->initialize(argc, argv);
    exp->set_table_height_threshold(0.5);

    // First point cloud
    ROS_INFO("Loading first point cloud ...");
    string cloud_name = "/home/tpat8946/Data/TUW/TUW_dynamic_dataset_icra15/test_set/set_00007/00000.pcd";
    string trans_name = "/home/tpat8946/Data/TUW/TUW_dynamic_dataset_icra15/test_set/set_00007/transformation_00000.txt";
    bool reverse_transform = false;
    string image_name = "/home/tpat8946/ros_ws/squirrel_active_exploration/src/squirrel_active_exploration/data/test45.png";
    // Load the point cloud
    PointCloud<PointT>::Ptr cloud;
    cloud.reset(new PointCloud<PointT>);
    if (io::loadPCDFile<PointT> (cloud_name.c_str(), *cloud) == -1)
    {
        ROS_ERROR("TEST_update (main) : could not read cloud filename");
        return EXIT_FAILURE;
    }
    // Load the transformation matrix
    Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
    // Read the pose from the file name
    if (!read_tf_file(trans_name, tr))
        ROS_WARN("TEST_update (main) : could not read transform file %s", trans_name.c_str());
    if (reverse_transform)
        tr = tr.inverse();
    // Transform the point cloud
    transformPointCloud(*cloud, *cloud, tr);
    // Convert to ROS msg
    sensor_msgs::PointCloud2 in_cloud;
    toROSMsg(*cloud, in_cloud);
    // Load the image
    cv::Mat image = cv::imread(image_name,-1);
    cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    // Convert OpenCV image to ROS message
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "saliency_map";
    cv_ptr->encoding = "mono8";
    cv_ptr->image = image;
    sensor_msgs::Image in_image;
    cv_ptr->toImageMsg(in_image);

    // Process the point cloud
    ROS_INFO("Processing first point cloud ...");
    if (!exp->process(in_cloud, in_image))
    {
        ROS_ERROR("TEST_update (main) : error when processing point cloud");
        return EXIT_FAILURE;
    }
    // Viualize the segments
    if(!exp->visualize())
    {
        ROS_ERROR("TEST_update (main) : error when visualizing the point cloud segments");
        return EXIT_FAILURE;
    }

    // Second point cloud
    ROS_INFO("Loading second point cloud ...");
    cloud_name = "/home/tpat8946/Data/TUW/TUW_dynamic_dataset_icra15/test_set/set_00007/00001.pcd";
    trans_name = "/home/tpat8946/Data/TUW/TUW_dynamic_dataset_icra15/test_set/set_00007/transformation_00001.txt";
    // Load the point cloud
    cloud.reset(new PointCloud<PointT>);
    if (io::loadPCDFile<PointT> (cloud_name.c_str(), *cloud) == -1)
    {
        ROS_ERROR("TEST_update (main) : could not read cloud filename");
        return EXIT_FAILURE;
    }
    // Load the transformation matrix
    tr = Eigen::Matrix4f::Identity();
    // Read the pose from the file name
    if (!read_tf_file(trans_name, tr))
        ROS_WARN("TEST_update (main) : could not read transform file %s", trans_name.c_str());
    if (reverse_transform)
        tr = tr.inverse();
    // Transform the point cloud
    transformPointCloud(*cloud, *cloud, tr);
    // Convert to ROS msg
    toROSMsg(*cloud, in_cloud);

    // Update the hypotheses
    ROS_INFO("Updating hypotheses ...");
    if (!exp->update_hypotheses(in_cloud, in_image))
    {
        ROS_ERROR("TEST_update (main) : error when processing point cloud");
        return EXIT_FAILURE;
    }
    if(!exp->visualize())
    {
        ROS_ERROR("TEST_update (main) : error when visualizing the point cloud segments");
        return EXIT_FAILURE;
    }

    // Shutdown and exit
    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}

