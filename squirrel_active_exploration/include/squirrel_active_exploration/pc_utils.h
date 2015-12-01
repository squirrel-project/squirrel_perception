#ifndef PC_UTILS_H
#define PC_UTILS_H

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>

#include <octomap/octomap.h>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include "math_utils.h"
#include "octomap_utils.h"

#define _RADIUS 2
#define _DOWNSAMPLE_FOR_VIS_CHECKING 0.05
#define _DOWNSAMPLE_START_RESOLUTION 0.05
#define _ICP_POINT_MAX 50
#define _ICP_POINT_MIN 10
#define _ICP_NUM_TRIES 4

typedef pcl::PointXYZRGB PointT;

// Convenient structure to handle pointclouds
struct PCD
{
    pcl::PointCloud<PointT>::Ptr cloud;
    std::string f_name;

    PCD() : cloud (new pcl::PointCloud<PointT>())
    {}
};

struct PCDComparator
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
{
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

//template<typename T>
//Eigen::Matrix4f get_transfrom_to_origin_frame(const pcl::PointCloud<T> &cloud);

//template<typename T>
//void organize_point_cloud(const pcl::PointCloud<T> &cloud, pcl::PointCloud<T> &cloud_organized);

Eigen::Vector4f extract_camera_position(const pcl::PointCloud<PointT> &cloud, const double &radius = _RADIUS);

bool icp(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &transform, double &score,
         const bool &downsample = true);

bool find_planes(const pcl::PointCloud<PointT> &cloud, std::vector<int> &plane_indices);

#endif // PC_UTILS_H
