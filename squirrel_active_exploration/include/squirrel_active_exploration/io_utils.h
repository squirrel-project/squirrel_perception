#ifndef IO_UTILS_H
#define IO_UTILS_H

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>

#include <cstdlib>
#include <boost/tuple/tuple.hpp>
#include <dirent.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

#include "math_utils.h"
#include "transform_utils.h"

#define _CLOUD_PREFIX "cloud_"
#define _INDICES_PREFIX "object_indices_"

typedef pcl::PointXYZRGB PointT;

struct IndexPoint
{
    int idx;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint, (int, idx, idx) )

struct pcl_CloudTX
{
    Eigen::Vector4f _camera_position;
    pcl::PointCloud<PointT> _cloud;
    Eigen::Matrix4f _transform;
    int _ix;
};

struct ros_CloudTX
{
    Eigen::Vector4f _camera_position;
    sensor_msgs::PointCloud2 _cloud;
    Eigen::Matrix4f _transform;
    int _ix;
};

bool read_tf_file(const std::string &filename, Eigen::Matrix4f &tr);

bool write_to_file(const std::string &filename, const double &val);

bool write_to_file(const std::string &filename, const float &val);

bool load_test_directory(const std::string &dir, const bool &invert_transform,
                         std::vector<Eigen::Vector4f> &poses, std::vector<pcl::PointCloud<PointT> > &clouds,
                         std::vector<Eigen::Matrix4f> &transforms, std::vector<int> &ix_order);

bool load_test_directory(const std::string &dir, const bool &invert_transform,
                         std::vector<Eigen::Vector4f> &poses, std::vector<pcl::PointCloud<PointT> > &clouds, std::vector<Eigen::Matrix4f> &transforms);

bool load_test_directory(const std::string &dir, const bool &invert_transform,
                         std::vector<Eigen::Vector4f> &poses, std::vector<sensor_msgs::PointCloud2 > &clouds,
                         std::vector<Eigen::Matrix4f> &transforms, std::vector<int> &ix_order);

bool load_test_directory(const std::string &dir, const bool &invert_transform,
                         std::vector<Eigen::Vector4f> &poses, std::vector<sensor_msgs::PointCloud2 > &clouds, std::vector<Eigen::Matrix4f> &transforms);

bool load_test_directory(const std::string &dir, const bool &invert_transform, std::vector<pcl_CloudTX> &pose_cloud_tf);

bool load_test_directory(const std::string &dir, const bool &invert_transform, std::vector<ros_CloudTX> &pose_cloud_tf);

bool load_test_directory_with_segment_indices(const std::string &dir, const bool &invert_transform,
                                              std::vector<Eigen::Vector4f> &poses, std::vector<pcl::PointCloud<PointT> > &clouds,
                                              std::vector<Eigen::Matrix4f> &transforms, std::vector<std::vector<std::vector<int> > > &indices,
                                              const int &index = -1);

bool load_model_training_directory(const std::string &dir, const bool &invert_transform,
                                   std::vector<Eigen::Vector4f> &poses, std::vector<pcl::PointCloud<PointT> > &clouds,
                                   std::vector<Eigen::Matrix4f> &transforms, std::vector<std::vector<int> > &indices);

int get_next_write_index(const std::string &dir, const std::string &prefix);

#endif // IO_UTILS_H
