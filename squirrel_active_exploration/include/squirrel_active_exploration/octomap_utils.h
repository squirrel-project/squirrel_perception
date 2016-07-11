#ifndef OCTOMAP_UTILS_H
#define OCTOMAP_UTILS_H

#include <ros/ros.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <boost/filesystem.hpp>
#include <sys/types.h>
#include <sys/stat.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <dirent.h>

#include "ground_removal.h"
#include "file_utils.h"

#define _GROUND_THRESH 0.25
#define _CONFIG_FILE "config"
#define _TREE_KEYS_FILE "tree_keys"
#define _VIEW_KEYS_FILE "view_keys"

typedef pcl::PointXYZRGB PointT;

typedef std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> keys_box;

struct robot_parameters
{
    double height;
    double outer_range;
    double inner_range;
    double radius;
};

// Comparator structure for using Eigen::Vector4f as a map key
struct compare_octree_key_struct
{
    bool operator()(const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const
    {
        if (lhs.k[0] < rhs.k[0]) return true;
        if (rhs.k[0] < lhs.k[0]) return false;
        // Otherwise, lhs.k[0] == rhs.k[0]
        if (lhs.k[1] < rhs.k[1]) return true;
        if (rhs.k[1] < lhs.k[1]) return false;
        // Otherwise, lhs.k[1] == rhs.k[1]
        if (lhs.k[2] < rhs.k[2]) return true;
        if (rhs.k[2] < lhs.k[2]) return false;
        return false;
    }
};

// Comparator function for using octomap::OcTree as a map key
bool compare_octree_key(const octomap::OcTreeKey &lhs, const octomap::OcTreeKey &rhs);

bool equal_octree_key(const octomap::OcTreeKey &lhs, const octomap::OcTreeKey &rhs);

// Comparator function for using octomap::OcTree as a map key
bool compare_pair_double_octree_key(const std::pair<double,octomap::OcTreeKey> &lhs,
                                    const std::pair<double,octomap::OcTreeKey> &rhs);

// Comparator function for sorting by the size of KeyRay
bool compare_keys_box(const std::pair<std::vector<octomap::OcTreeKey>,octomap::point3d> &lhs,
                      const std::pair<std::vector<octomap::OcTreeKey>,octomap::point3d> &rhs);

/* === OCTOMAP FUNCTIONS === */

bool octree_to_occupied_free_clouds(const octomap::OcTree &tree, pcl::PointCloud<PointT> &occupied, pcl::PointCloud<PointT> &free,
                                    const unsigned int &depth = 0);

bool get_octree_ground(const pcl::PointCloud<PointT> &in_cloud, std::vector<int> &indices,
                       pcl::PointCloud<PointT> &ground, pcl::PointCloud<PointT> &remaining);

pcl::PointCloud<PointT> octree_to_cloud(const octomap::OcTree &tree, const unsigned int &depth = 0);

double octree_ground_height(const octomap::OcTree &tree, const unsigned int &depth = 0);

double octree_resolution_at_depth(const octomap::OcTree &tree, const unsigned int &depth = 0);

std::vector<octomap::point3d> voxels_from_keys(const octomap::OcTree &tree, const std::vector<octomap::OcTreeKey> &keys);

std::vector<octomap::OcTreeKey> visible_voxels(const octomap::OcTree &tree, const octomap::point3d &location, const robot_parameters &robot,
                                               const double &zmin, const double &zheight, const unsigned int &depth = 0);

std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> >
compute_visibility(const octomap::OcTree &tree, const std::vector<octomap::point3d> &locations, const robot_parameters &robot,
                   const double &zmin, const double &zheight, const unsigned int &depth = 0);

void valid_on_ground(const octomap::OcTree &tree, const double &robot_radius,
                     const double &zmin, const double &zmax, const double &floor_height,
                     const octomap::point3d &location, bool &valid, const bool &check_floor = true);

void valid_on_ground(const octomap::OcTree &tree, const double &robot_radius, const double &zmin, const double &zmax, const double &floor_height,
                     std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> > &locations, const bool &check_floor = true);

std::vector<octomap::point3d> exhaustive_coverage_set(const octomap::OcTree &tree, const robot_parameters &robot, const double &step,
                                                      const unsigned int &depth);

bool compute_visible_locations(const octomap::OcTree &tree, const robot_parameters &robot, const double &step, std::vector<octomap::OcTreeKey> &tree_keys,
                               std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> > &valid_locations, const unsigned int &depth);

std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> >
optimize_coverage_set(const std::vector<octomap::OcTreeKey> &tree_keys,
                      const std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> > &locations,
                      const double &range, const int &max_iters = -1);

void update_keys(const std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> &location, const double &range,
                 std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> > &other_locations);

void remove_keys(const std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> &location, std::vector<octomap::OcTreeKey> &key_map);

std::vector<octomap::point3d> get_coverage_locations(const octomap::OcTree &tree, const std::string &dir, const robot_parameters &robot,
                                                     const double &step, const int &max_iters = -1, const unsigned int &depth = 0);

/* === VISUALIZATION === */

void octree_visualize(const octomap::OcTree &tree, const robot_parameters &robot, const unsigned int &depth = 0);

void octree_visualize_grid(const octomap::OcTree &tree, const robot_parameters &robot, const double &step, const unsigned int &depth = 0);

void octree_visualize_location(const octomap::OcTree &tree, const robot_parameters &robot, const unsigned int &depth = 0);

void octree_visualize_minimal_overlap(const octomap::OcTree &tree, const std::string &dir, const robot_parameters &robot, const double &step,
                                      const int &max_iters, const unsigned int &depth = 0);

void octree_visualize_segments(const octomap::OcTree &tree, const std::vector<std::vector<octomap::OcTreeKey> > &segments);

void visualize_coverage(const octomap::OcTree &tree, const robot_parameters &robot, const double &step, std::vector<octomap::point3d> &locations,
                        const unsigned int &depth = 0);

void run_view_locations(pcl::visualization::PCLVisualizer *viewer, std::vector<octomap::point3d> &coverage_path,
                        std::vector<pcl::PointCloud<PointT> > &path_vis, const double &range, const double &zmin, const double &zmax);

void run_viewer(pcl::visualization::PCLVisualizer *viewer);

/* === IO === */

int save_visible_locations(const std::string &dir, const std::vector<octomap::OcTreeKey> &tree_keys,
                           const std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> > &locations,
                           const robot_parameters &robot, const double &step, const unsigned int &depth);

void write_keys_box(const std::string &filename, const std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> > &vec);

void write_config_file(const std::string &filename, const unsigned int &depth, const robot_parameters &robot, const double &step);

std::vector<std::pair<std::vector<octomap::OcTreeKey>, octomap::point3d> > read_keys_box(const std::string &filename);

void read_config_file(const std::string &filename, unsigned int &depth, robot_parameters &robot, double &step);

bool load_precomputed_data(const std::string &dir, const unsigned int depth, const robot_parameters &robot, const double &step, int &ix);

std::vector<int> get_all_config_files(const std::string &dir);

octomap::OcTree get_tree_from_filename(const std::string &str, bool &return_is_directory);

std::string append_backslash_to_path(const std::string &str);

#endif // OCTOMAP_UTILS_H
