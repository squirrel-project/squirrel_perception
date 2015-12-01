#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "math_utils.h"
#include "file_utils.h"
#include "io_utils.h"
#include "pc_utils.h"
//#include "active_exploration_utils.h"

#define _NUM_ROTATIONS_ICP 12

//// Forward declaration
//class Pose;

typedef pcl::PointXYZRGB PointT;

bool frame_to_frame_tf(const std::string &frame1, const std::string &frame2, Eigen::Matrix4f &transform);

bool frame_to_frame(const pcl::PointCloud<PointT> &in_cloud, pcl::PointCloud<PointT> &out_cloud,
                    const std::string &frame1, const std::string &frame2);

bool frame_to_frame(const std::string &frame1, const std::string &frame2,
                    const pcl::PointCloud<PointT> &in_cloud, pcl::PointCloud<PointT> &out_cloud);

bool frame_to_frame(const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud,
                    const std::string &frame1, const std::string &frame2);

bool frame_to_frame(const std::string &frame1, const std::string &frame2,
                    const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud);

bool transform_cloud_from_file(const std::string &filename, const pcl::PointCloud<PointT> &in_cloud,
                               pcl::PointCloud<PointT> &out_cloud, Eigen::Vector4f &camera_pose, Eigen::Matrix4f &transform);

bool transform_cloud_from_file(const std::string &filename, const pcl::PointCloud<PointT> &in_cloud,
                               pcl::PointCloud<PointT> &out_cloud, Eigen::Vector4f &camera_pose);

bool transform_cloud_from_file(const std::string &filename, const pcl::PointCloud<PointT> &in_cloud,
                               pcl::PointCloud<PointT> &out_cloud, pcl::PointCloud<PointT> &camera_pose, Eigen::Matrix4f &transform);

bool transform_cloud_from_file(const std::string &filename, const pcl::PointCloud<PointT> &in_cloud,
                               pcl::PointCloud<PointT> &out_cloud, pcl::PointCloud<PointT> &camera_pose);

bool transform_cloud_from_file(const std::string &dir, const std::string &filename, const pcl::PointCloud<PointT> &in_cloud,
                               pcl::PointCloud<PointT> &out_cloud, Eigen::Vector4f &camera_pose, Eigen::Matrix4f &transform);

bool transform_cloud_from_file(const std::string &dir, const std::string &filename, const pcl::PointCloud<PointT> &in_cloud,
                               pcl::PointCloud<PointT> &out_cloud, Eigen::Vector4f &camera_pose);

bool transform_cloud_from_file(const std::string &dir, const std::string &filename, const pcl::PointCloud<PointT> &in_cloud,
                               pcl::PointCloud<PointT> &out_cloud, pcl::PointCloud<PointT> &camera_pose, Eigen::Matrix4f &transform);

bool transform_cloud_from_file(const std::string &dir, const std::string &filename, const pcl::PointCloud<PointT> &in_cloud,
                               pcl::PointCloud<PointT> &out_cloud, pcl::PointCloud<PointT> &camera_pose);

bool transform_cloud_to_cloud(const pcl::PointCloud<PointT> &source, const pcl::PointCloud<PointT> &target,
                              Eigen::Matrix4f &transform, double &score);

//bool transform_cloud_to_cloud(const pcl::PointCloud<PointT> &source, const Eigen::Vector4f &source_pose,
//                              const pcl::PointCloud<PointT> &target, const Eigen::Vector4f &target_pose,
//                              Eigen::Matrix4f &transform, double &score);

//bool transform_cloud_to_cloud(const pcl::PointCloud<PointT> &source, const Eigen::Vector4f &source_pose,
//                              const pcl::PointCloud<PointT> &target,
//                              Eigen::Matrix4f &transform, double &score);

//bool transform_cloud_to_cloud(const pcl::PointCloud<PointT> &source,
//                              const pcl::PointCloud<PointT> &target, const Eigen::Vector4f &target_pose,
//                              Eigen::Matrix4f &transform , double &score);

//bool transform_cloud_to_cloud(const pcl::PointCloud<PointT> &source,
//                              const pcl::PointCloud<PointT> &target,
//                              Eigen::Matrix4f &transform, double &score);

bool transform_cloud_to_cloud_nonrigid(const pcl::PointCloud<PointT> &source, const pcl::PointCloud<PointT> &target,
                                       Eigen::Matrix4f &transform, double &score, double &scale);

std::vector<Eigen::Matrix4f> rotation_matrix(const double &theta);

Eigen::Vector4f transform_eigvec(const Eigen::Vector4f &vec, const Eigen::Matrix4f &transform);

pcl::PointCloud<PointT> transform_eigvec_to_cloud(const Eigen::Vector4f &vec, const Eigen::Matrix4f &transform);

Eigen::Vector4f pcl_to_eig(const pcl::PointCloud<PointT> &in_cloud);

pcl::PointCloud<PointT> eig_to_pcl(const Eigen::Vector4f &in_eigen);

bool downsample_point_cloud(const pcl::PointCloud<PointT> &in_cloud, pcl::PointCloud<PointT> &out_cloud);

#endif // TRANSFORM_UTILS_H
