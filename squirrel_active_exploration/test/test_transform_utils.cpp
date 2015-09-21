#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include "squirrel_active_exploration/transform_utils.h"

using namespace std;
using namespace pcl;

// Run Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "unit_test_transform_utils");
    ros::NodeHandle node("~");
    srand (time(NULL));

//    /* === TEST 1 === */
//    // Load a point cloud
//    string cloud_name = "/home/tpat8946/ros_ws/squirrel_active_exploration/src/squirrel_active_exploration/data/training_small/apple/118644ba80aa5048ac59fb466121cd17/esf/view_1.pcd";
//    PointCloud<PointT> target;
//    if (io::loadPCDFile<PointT> (cloud_name.c_str(), target) == -1)
//    {
//        ROS_WARN("TEST_transform_utils (main) : could not load point cloud %s", cloud_name.c_str());
//        return EXIT_FAILURE;
//    }
//    // Small change to the point cloud
//    Eigen::Matrix4f original_transform = Eigen::Matrix4f::Identity();
//    float theta = 3 * (float)rand() / (float)RAND_MAX; // random angle
//    float x = 5.0;// * (float)rand() / (float)RAND_MAX;
//    float y = 5.0;// * (float)rand() / (float)RAND_MAX;
//    float z = 5.0;// * (float)rand() / (float)RAND_MAX;
//    original_transform(0,0) = cos(theta);
//    original_transform(0,1) = -sin(theta);
//    original_transform(1,0) = sin(theta);
//    original_transform(1,1) = cos(theta);
//    original_transform(0,3) = x;
//    original_transform(1,3) = y;
//    original_transform(2,3) = z;
//    cout << "Original Transform" << endl;
//    cout << original_transform << endl;
//    // Transform the point cloud
//    PointCloud<PointT> source;
//    transformPointCloud(target, source, original_transform);
//    // Apply a small scale change
//    Eigen::Matrix4f scale_transform = Eigen::Matrix4f::Identity();
//    scale_transform.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * 0.5;
//    transformPointCloud(source, source, scale_transform);
//    cout << "Scale Transform" << endl;
//    cout << scale_transform << endl;
//    cout << "Combined Transform" << endl;
//    cout << scale_transform * original_transform << endl;

//    string save_file = "source.pcd";
//    io::savePCDFileASCII (save_file, source);
//    save_file = "target.pcd";
//    io::savePCDFileASCII (save_file, target);

//    // Recover the inverse transform by aligning the source to the target
//    Eigen::Matrix4f recovered_transform;
//    double score;
//    if (!transform_cloud_to_cloud(source, target, recovered_transform, score))
//    {
//        ROS_WARN("TEST_transform_utils (main) : could not transform point cloud");
//        return EXIT_FAILURE;
//    }
//    cout << "Recovered Transform" << endl;
//    cout << recovered_transform << endl;
//    cout << "Recovered Transform (inverse)" << endl;
//    cout << recovered_transform.inverse() << endl;
//    save_file = "recovered.pcd";
//    transformPointCloud(source, source, recovered_transform);
//    io::savePCDFileASCII (save_file, source);



    /* === TEST 2 === */
    PointCloud<PointT> segment;
    io::loadPCDFile("/home/tpat8946/Data/TUW/test_alignment/segment.pcd", segment);
    PointCloud<PointT> instance_view;
    io::loadPCDFile("/home/tpat8946/Data/TUW/test_alignment/instance_view.pcd", instance_view);
    // Perform icp to transform instance_view cloud to segment cloud
    Eigen::Matrix4f transform;
    double score;
    transform_cloud_to_cloud(instance_view, segment, transform, score);
    transformPointCloud(instance_view, instance_view, transform);
    io::savePCDFileASCII("/home/tpat8946/source.pcd", segment);
    io::savePCDFileASCII("/home/tpat8946/target.pcd", instance_view);
    io::savePCDFileASCII("/home/tpat8946/transformed.pcd", instance_view);

//    // Scale
//    double s = 0.231153;
//    Eigen::Matrix4f T_scale = Eigen::Matrix4f::Identity();
//    T_scale.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * s;
//    PointCloud<PointT> transformed_view;
//    transformPointCloud(instance_view, transformed_view, T_scale);
//    io::savePCDFileASCII("/home/tpat8946/scaled.pcd", transformed_view);
//    // Center
//    Eigen::Vector4f pose;
//    compute3DCentroid (transformed_view, pose);
//    Eigen::Matrix4f ctf = Eigen::Matrix4f::Identity();
//    ctf(0,3) = -pose[0];
//    ctf(1,3) = -pose[1];
//    ctf(2,3) = -pose[2];
//    transformPointCloud(transformed_view, transformed_view, ctf);
//    io::savePCDFileASCII("/home/tpat8946/centred.pcd", transformed_view);
//    // Rotate
//    double theta = 4.39823;
//    int rix = 0;
//    Eigen::Matrix4f rot = rotation_matrix(theta)[rix];
//    transformPointCloud(transformed_view, transformed_view, rot);
//    io::savePCDFileASCII("/home/tpat8946/rotated.pcd", transformed_view);
//    // ICP
//    Eigen::Matrix4f icp_tf = Eigen::Matrix4f::Identity();
//    icp_tf(0,0) = 0.846744;
//    icp_tf(0,1) = 0.446652;
//    icp_tf(0,2) = -0.28901;
//    icp_tf(0,3) = 0.0355938;
//    icp_tf(1,0) = -0.52684;
//    icp_tf(1,1) = 0.779488;
//    icp_tf(1,2) = -0.338876;
//    icp_tf(1,3) = -0.0548714;
//    icp_tf(2,0) = 0.0739205;
//    icp_tf(2,1) = 0.439203;
//    icp_tf(2,2) = 0.895341;
//    icp_tf(2,3) = 0.0737792;
//    icp_tf(3,0) = 0;
//    icp_tf(3,1) = 0;
//    icp_tf(3,2) = 0;
//    icp_tf(3,3) = 1;
//    transformPointCloud(transformed_view, transformed_view, icp_tf);
//    io::savePCDFileASCII("/home/tpat8946/icp_aligned.pcd", transformed_view);

//    // Transforms together
//    Eigen::Matrix4f final_tf = icp_tf * rot * ctf * T_scale;
//    transformPointCloud(instance_view, transformed_view, final_tf);
//    io::savePCDFileASCII("/home/tpat8946/final.pcd", transformed_view);

    ros::shutdown();
    return EXIT_SUCCESS;
}
