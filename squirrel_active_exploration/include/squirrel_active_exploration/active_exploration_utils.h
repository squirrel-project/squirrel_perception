#ifndef ACTIVE_EXPLORATION_UTILS_H
#define ACTIVE_EXPLORATION_UTILS_H

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

#include "squirrel_active_exploration/transform_utils.h"
#include "squirrel_active_exploration/pcl_conversions.h"
#include <squirrel_object_perception_msgs/Segment.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/EntropyMap.h>
#include <squirrel_object_perception_msgs/EntropyMapViz.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationInit.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationOnce.h>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <boost/tuple/tuple.hpp>

#include "math_utils.h"
#include "octomap_utils.h"
#include "pc_utils.h"

#define _NULL_FILE "null"
#define _INSTANCE_VIEW_FILENAME "view_"
#define _EPS 0.05
#define _CONFIDENCE_THRESHOLD 0.9

typedef pcl::PointXYZRGB PointT;

/* === DEFINITIONS === */

enum SIM_TYPE
{
    WORST_TO_BEST_ENTROPY = 0,
    BEST_TO_WORST_ENTROPY = 1,
    WORST_TO_BEST_PROB = 2,
    BEST_TO_WORST_PROB = 3,
    RANDOM = 4,
    NEAREST_AREA = 5,
    NEAREST_MIN_ENTROPY = 6,
    NEAREST_MAX_CLASS_PROB = 7,
    MAX_AREA = 8,
    MAX_AREA_UNOCCLUDED = 9,
    MIN_CLASS_ENTROPY = 10,
    MIN_CLASS_ENTROPY_UNOCCLUDED = 11,
    MAX_CLASS_PROB = 12,
    MAX_CLASS_PROB_UNOCCLUDED = 13,
    MIN_VIEW_CLASSIFICATION_ENTROPY = 14,
    MAX_VIEW_CLASSIFICATION_PROB = 15
};

class Pose
{
public:
    Pose()
    {
        valid = false;
    }

    Pose(const Eigen::Vector4f &centroid, const Eigen::Vector4f bb_min, const Eigen::Vector4f bb_max)
        : _centroid (centroid),
          _bb_min (bb_min),
          _bb_max (bb_max)
    {
        valid = true;
    }

    Eigen::Vector4f get_centroid()
    {
        return _centroid;
    }

    Eigen::Vector4f get_centroid() const
    {
        return _centroid;
    }

    Eigen::Vector4f get_bb_min()
    {
        return _bb_min;
    }

    Eigen::Vector4f get_bb_min() const
    {
        return _bb_min;
    }

    Eigen::Vector4f get_bb_max()
    {
        return _bb_max;
    }

    Eigen::Vector4f get_bb_max() const
    {
        return _bb_max;
    }

    bool is_valid()
    {
        return valid;
    }

    bool is_valid() const
    {
        return valid;
    }

private:
    Eigen::Vector4f _centroid;
    Eigen::Vector4f _bb_min;
    Eigen::Vector4f _bb_max;
    bool valid;
};

struct InstLookUp
{
    std::string _dir;
    std::string _class_type;
    std::string _instance_name;
    std::string _ix;
};

struct EntMap
{
    pcl::PointCloud<PointT> _instance_cloud;
    std::vector<int> _ixs;
    std::vector<pcl::PointCloud<PointT> > _clouds;
    std::vector<Eigen::Matrix4f> _transforms;
    std::vector<Eigen::Vector4f> _centroids;
    std::vector<Eigen::Vector4f> _camera_poses;
    std::vector<Eigen::Quaternionf> _camera_orientations;
    std::vector<double> _surface_areas;
    std::vector<double> _class_entropies;
    std::vector<double> _recognition_probabilities;
    std::string _octree_file;
    bool _valid;
};

struct InstToMapTF
{
    Eigen::Matrix4f _transform;
    double _score;
};

struct Hypothesis
{
    std::vector<std::vector<int> > _segments;
    std::vector<std::vector<octomap::OcTreeKey> > _octree_keys;
    std::vector<squirrel_object_perception_msgs::Classification> _class_estimates;
    std::vector<Pose> _poses;
    std::vector<std::vector<InstLookUp> > _instance_directories;
    std::vector<std::vector<InstToMapTF> > _transforms;
    std::vector<std::vector<EntMap> > _emaps;
    std::vector<double> _entropies;
    std::vector<int> _entropy_ranking;
};
namespace active_exploration_utils
{
    /* === SEGMENTATION === */

    bool segment(const pcl::PointCloud<PointT> &cloud, ros::ServiceClient &seg_client, std::vector<std::vector<int> > &segments);

    bool segment(const pcl::PointCloud<PointT> &cloud, const std::vector<std::vector<int> > &segment_indices,
                 std::vector<std::vector<int> > &segments);

    bool filter_segments(const octomap::OcTree &tree, const pcl::PointCloud<PointT> &transformed_cloud, const Eigen::Vector4f &position,
                         const double &max_object_distance, const double &min_object_height, const double &max_object_height, const double &min_object_length, const double &max_object_length, std::vector<std::vector<int> > &segments,
                         std::vector<int> &ground_indices, std::vector<octomap::OcTreeKey> &ground_keys);

    bool is_valid_segment(const pcl::PointCloud<PointT> &cloud, const double &min_object_height, const double &max_object_height,
                          const double &min_object_length, const double &max_object_length, const int &ix = -1);

    bool is_valid_segment(const pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f &transform,
                          const double &min_object_height, const double &max_object_height,
                          const double &min_object_length, const double &max_object_length, const int &ix = -1);

    bool extract_segment_octree_keys(const octomap::OcTree &tree, const pcl::PointCloud<PointT> &transformed_cloud,
                                     const std::vector<std::vector<int> > &segments,
                                     std::vector<std::vector<octomap::OcTreeKey> > &segment_octree_keys);

    /* === POSE ESTIMATION / OBJECT TRACKING === */

    bool estimate_pose(const pcl::PointCloud<PointT> &transformed_cloud, const std::vector<std::vector<int> > &segments, std::vector<Pose> &poses);

    /* === CLASSIFICATION === */

    bool classify(const pcl::PointCloud<PointT> &transformed_cloud, const std::vector<std::vector<int> > &segments, ros::ServiceClient &class_client,
                  std::vector<squirrel_object_perception_msgs::Classification> &class_estimates,
                  std::vector<std::vector<InstLookUp> > &instance_directories,
                  std::vector<std::vector<InstToMapTF> > &instances_to_map_tfs);

    std::vector<squirrel_object_perception_msgs::Classification> fix_path_names(std::vector<squirrel_object_perception_msgs::Classification> &class_estimates);

    bool extract_instance_directories(const std::vector<squirrel_object_perception_msgs::Classification> &class_estimates,
                                      std::vector<std::vector<InstLookUp> > &instance_directories);

    bool transform_instances_to_map(const pcl::PointCloud<PointT> &transformed_cloud, const std::vector<std::vector<int> > &segments,
                                    const std::vector<std::vector<InstLookUp> > &instance_directories,
                                    std::vector<std::vector<InstToMapTF> > &transforms);

    bool transform_instances_to_map(const pcl::PointCloud<PointT> &transformed_cloud, const std::vector<std::vector<int> > &segments,
                                    const std::vector<std::vector<InstLookUp> > &instance_directories,
                                    std::vector<std::vector<Eigen::Matrix4f> > &transforms);

    /* === GET ENTROPY MAPS === */

    bool retrieve_entropy_maps(const std::vector<std::vector<int> > &segments, const std::vector<std::vector<InstLookUp> > &instance_directories,
                               ros::ServiceClient &em_client, std::vector<std::vector<EntMap> > &entropy_maps);

    /* === COMPUTE ENTROPY === */

    bool compute_entropy(const std::vector<squirrel_object_perception_msgs::Classification> &class_estimates, std::vector<double> &entropies);

    bool rank_entropy(const std::vector<double> &entropies, std::vector<int> &entropy_ranking);

    /* === UPDATE HYPOTHESES === */

    bool update_hypotheses(const std::vector<std::vector<int> > &overlaps, const octomap::OcTree &tree,
                           const Hypothesis &previous_hyp, const Hypothesis &current_hyp,
                           std::vector<std::vector<int> > &updated_associations, Hypothesis &result_hyp);

    bool merge_segments(const std::vector<int> &first, const std::vector<int> &second, std::vector<int> &output);

    bool merge_estimates(const squirrel_object_perception_msgs::Classification &first, const squirrel_object_perception_msgs::Classification &second,
                         const double &min_eps, squirrel_object_perception_msgs::Classification &output);

    bool merge_segment_octree_keys(const octomap::OcTree &tree, const std::vector<octomap::OcTreeKey> &first, const std::vector<octomap::OcTreeKey> &second,
                                   std::vector<octomap::OcTreeKey> &output);

    bool merge_poses(const Pose &first, const Pose &second, Pose &output);

    /* === OVERLAP CHECKING === */

    bool bounding_box_overlap(const Eigen::Vector4f &min1, const Eigen::Vector4f &max1, const Eigen::Vector4f &min2, const Eigen::Vector4f &max2);

    bool voxels_overlap(const std::vector<octomap::OcTreeKey> &first, const std::vector<octomap::OcTreeKey> &second, const double &threshold);

    std::vector<std::vector<int> > segment_overlap(const std::vector<Pose> &first, const std::vector<std::vector<octomap::OcTreeKey> > &first_keys,
                                                   const std::vector<Pose> &second, const std::vector<std::vector<octomap::OcTreeKey> > &second_keys,
                                                   const double &threshold);

    std::vector<std::vector<int> > segment_overlap(const std::vector<Pose> &first, const std::vector<Pose> &second, const double &threshold);

    std::vector<std::vector<int> > segment_overlap(const std::vector<std::vector<octomap::OcTreeKey> > &first_keys,
                                                   const std::vector<std::vector<octomap::OcTreeKey> > &second_keys,
                                                   const double &threshold);

    /* === PLANNING === */

    bool next_best_view(int &next_best_index, const octomap::OcTree &tree, const Hypothesis &hypothesis, const SIM_TYPE &sim,
                        const std::vector<Eigen::Vector4f> &map_locations, const double &variance, const bool &do_visualize = false);

    int nearest_next_best_view(const std::vector<squirrel_object_perception_msgs::Classification> &class_estimates,
                               const std::vector<Eigen::Vector4f> &map_locations, const std::vector<int> &seg_indices,
                               const std::vector<std::vector<std::pair<std::vector<pcl::PointCloud<PointT> >,std::vector<Eigen::Vector4f> > > > &model_views,
                               const std::vector<std::vector<std::vector<double> > > &scaled_model_utilities,
                               const std::vector<double> &uncertainty_weight);

    int gaussian_weighted_next_best_view(const octomap::OcTree &tree, const Hypothesis &hypothesis, const std::vector<Eigen::Vector4f> &map_locations,
                                         const std::vector<int> &seg_indices,
                                         const std::vector<std::vector<std::pair<std::vector<pcl::PointCloud<PointT> >,std::vector<Eigen::Vector4f> > > > &model_views,
                                         const std::vector<std::vector<std::vector<double> > > &scaled_model_utilities,
                                         const std::vector<double> &uncertainty_weight, const double &variance, const bool &unoccluded,
                                         const bool &do_visualize = false);

    int extracted_point_cloud_next_best_view(const std::vector<Eigen::Vector4f> &map_locations, const std::vector<int> &seg_indices,
                                             const std::vector<double> &uncertainty_weight, const SIM_TYPE &sim);

    double location_utility(const Eigen::Vector4f &location, const std::vector<Eigen::Vector4f> &training_locations,
                            const std::vector<double> &training_utilities, const double &variance);

    pcl::PointCloud<PointT> get_expected_point_cloud(const Eigen::Vector4f &location_in_map, const EntMap &emap, const int &emap_ix,
                                                     const Eigen::Matrix4f instance_to_map_tfs);

    std::vector<int> get_visible_points_in_cloud(const octomap::OcTree &tree, const Eigen::Vector4f &origin, const pcl::PointCloud<PointT> &cloud,
                                                 const std::map<octomap::OcTreeKey,std::vector<int>,compare_octree_key_struct> &keys_to_points,
                                                 const std::vector<octomap::OcTreeKey> &ignore_keys = std::vector<octomap::OcTreeKey>());

    double percentage_visible_points(const octomap::OcTree &tree, const Eigen::Vector4f &location, const InstLookUp &inst_dir, const EntMap &emap,
                                     const Eigen::Matrix4f &transform, const std::vector<octomap::OcTreeKey> &ignore_keys,
                                     pcl::PointCloud<PointT> &expected_cloud, std::vector<int> &visible_indices);

    std::vector<int> keys_to_point_indices(const octomap::OcTree &tree, const std::vector<octomap::OcTreeKey> &keys,
                                           const std::map<octomap::OcTreeKey,std::vector<int>,compare_octree_key_struct> &map_key_to_points,
                                           const std::vector<octomap::OcTreeKey> &ignore_keys = std::vector<octomap::OcTreeKey>());

    /* === TRANSFORMATION === */

    bool transform_pose(const Pose &in, Pose &out, const Eigen::Matrix4f &transform);

    bool transform_pose(const std::vector<Pose> &in, std::vector<Pose> &out, const Eigen::Matrix4f &transform);

    /* === DATA TYPE CONVERSION === */

    bool fromROSMsg(const squirrel_object_perception_msgs::EntropyMap &in, EntMap &out);

    /* === VISUALIZATION === */

    bool set_visualization_image(const pcl::PointCloud<PointT> &cloud, const sensor_msgs::Image &image, ros::ServiceClient &viz_init_client);

    bool visualize_segmentation(const octomap::OcTree &tree, const Hypothesis &hypothesis,
                                const std::map<octomap::OcTreeKey,std::vector<int>,compare_octree_key_struct> &map_key_to_points,
                                const std::vector<octomap::OcTreeKey> &current_ground_keys, ros::ServiceClient &viz_client);

    bool visualize_background(const pcl::PointCloud<PointT> &cloud, const std::vector<std::vector<int> > &segments, const std::vector<int> &planes,
                              const std::vector<int> &invalids, ros::ServiceClient &viz_client, const float &ground_height = std::numeric_limits<float>::infinity());

    bool visualize_planning_world(const pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f &transform, const std::vector<std::vector<int> > &segments,
                                  const std::vector<std::vector<InstLookUp> > &instance_directories,
                                  const std::vector<std::vector<InstToMapTF> > &instance_to_map_tfs, const std::vector<std::vector<EntMap> > &emaps,
                                  const Eigen::Vector4f &best_location, const std::vector<Eigen::Vector4f> &map_locations, const SIM_TYPE &sim);

    bool visualize_in_model_frame(const pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f &transform, const Eigen::Vector4f &location,
                                  const EntMap &emap, const Eigen::Matrix4f &instance_to_map_tf, const Eigen::Matrix4f &instance_to_model_tf);

    bool visualize_segment_overlap(const octomap::OcTree &tree, const std::vector<std::vector<int> > &overlaps,
                                   const std::map<octomap::OcTreeKey,std::vector<int>,compare_octree_key_struct> &map_key_to_points,
                                   const std::vector<octomap::OcTreeKey> &current_ground_keys,
                                   const pcl::PointCloud<PointT> &transformed_cloud, const std::vector<std::vector<int> > &segments,
                                   const std::vector<Pose> &poses, const std::vector<std::vector<octomap::OcTreeKey> > &octree_keys,
                                   const pcl::PointCloud<PointT> &previous_cloud, const std::vector<std::vector<int> > &previous_segments,
                                   const std::vector<Pose> &previous_poses, const std::vector<std::vector<octomap::OcTreeKey> > &previous_octree_keys);

    bool visualize_expected_clouds(const pcl::PointCloud<PointT> &transformed_cloud, const Eigen::Vector4f &location,
                                   const std::vector<std::vector<int> > &segments, const std::vector<int> &seg_indices,
                                   const std::vector<std::vector<InstLookUp> > &instance_directories, const std::vector<std::vector<EntMap> > &emaps,
                                   const std::vector<std::vector<InstToMapTF> > &instance_to_map_tfs,
                                   const std::vector<std::vector<pcl::PointCloud<PointT> > > &expected_clouds,
                                   const std::vector<std::vector<std::vector<int> > > &visible_indices);

    void run_view_segment(pcl::visualization::PCLVisualizer *viewer, const pcl::PointCloud<PointT> &transformed_cloud,
                          const std::vector<std::vector<int> > &segments, const std::vector<std::vector<pcl::PointCloud<PointT> > > &instance_clouds,
                          const std::vector<std::vector<pcl::PointCloud<PointT> > > &view_clouds,
                          const std::vector<std::vector<pcl::PointCloud<PointT> > > &camera_clouds,
                          const std::vector<std::vector<std::pair<int,int> > > &view_indices);

    void run_view_expected_clouds(pcl::visualization::PCLVisualizer *viewer, const pcl::PointCloud<PointT> &transformed_cloud,
                                  const Eigen::Vector4f &location, const std::vector<std::vector<int> > segments, const std::vector<int> &seg_indices,
                                  const std::vector<std::vector<pcl::PointCloud<PointT> > > &instance_clouds,
                                  const std::vector<std::vector<pcl::PointCloud<PointT> > > &expected_clouds,
                                  const std::vector<std::vector<std::vector<int> > > &visible_indices);

    void run_viewer(pcl::visualization::PCLVisualizer *viewer);
}

/* === IO === */

std::vector<int> load_entropy_order_file(const std::string &entropy_file, const std::string &test_dir, const SIM_TYPE &sim);

bool load_views_limit_from_file(const std::string &views_file, const std::string &test_dir, int &num_views);


#endif // ACTIVE_EXPLORATION_UTILS_H
