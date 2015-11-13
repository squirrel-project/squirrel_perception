#include "squirrel_active_exploration/ground_removal.h"

using namespace std;
using namespace pcl;

bool get_ground_plane(const PointCloud<PointT>::Ptr &in_cloud, vector<int> &indices)
{
    SACSegmentation<PointT> seg;
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (in_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        printf(ANSI_COLOR_RED  "ERROR ground_removal::get_ground_plane : failed to get ground plane"  ANSI_COLOR_RESET "\n");
        return false;
    }
    indices = inliers->indices;
    return true;
}

bool get_ground_plane(const PointCloud<PointT> &in_cloud, vector<int> &indices)
{
    PointCloud<PointT>::Ptr cloud_ptr (new PointCloud<PointT>(in_cloud));
    return get_ground_plane(cloud_ptr, indices);
}

bool get_ground_plane(const PointCloud<PointT>::Ptr &in_cloud, vector<int> &indices, PointCloud<PointT> &ground, PointCloud<PointT> &remaining)
{
    if (!get_ground_plane(in_cloud, indices))
    {
        printf(ANSI_COLOR_RED  "ERROR ground_removal::get_ground_plane : failed to call get ground plane"  ANSI_COLOR_RESET "\n");
        return false;
    }

    PointIndices::Ptr inliers (new PointIndices);
    inliers->indices = indices;

    // Extract the planar inliers from the input cloud
    ExtractIndices<PointT> extract;
    extract.setInputCloud (in_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (ground);

    // Extract the planer outliers
    extract.setNegative (true);
    extract.filter (remaining);

    return true;
}

bool get_ground_plane(const PointCloud<PointT> &in_cloud, vector<int> &indices, PointCloud<PointT> &ground, PointCloud<PointT> &remaining)
{
    PointCloud<PointT>::Ptr cloud_ptr (new PointCloud<PointT>(in_cloud));
    return get_ground_plane(cloud_ptr, indices, ground, remaining);
}
