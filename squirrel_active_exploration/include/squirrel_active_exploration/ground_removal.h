#ifndef GROUND_REMOVAL_H
#define GROUND_REMOVAL_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

typedef pcl::PointXYZRGB PointT;

bool get_ground_plane(const pcl::PointCloud<PointT>::Ptr &in_cloud, std::vector<int> &indices);

bool get_ground_plane(const pcl::PointCloud<PointT> &in_cloud, std::vector<int> &indices);

bool get_ground_plane(const pcl::PointCloud<PointT>::Ptr &in_cloud, std::vector<int> &indices,
                      pcl::PointCloud<PointT> &ground, pcl::PointCloud<PointT> &remaining);

bool get_ground_plane(const pcl::PointCloud<PointT> &in_cloud, std::vector<int> &indices,
                      pcl::PointCloud<PointT> &ground, pcl::PointCloud<PointT> &remaining);


#endif // GROUND_REMOVAL_H
