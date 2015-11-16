#ifndef ACTIVE_EXPLORATION_H
#define ACTIVE_EXPLORATION_H

#include <ros/ros.h>

#include <cstdlib>
#include <boost/tuple/tuple.hpp>
#include <numeric>

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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include "squirrel_active_exploration/pcl_conversions.h"
#include <squirrel_object_perception_msgs/Segment.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationInit.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationOnce.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_active_exploration/EntropyMap.h>
#include <squirrel_active_exploration/EntropyMapViz.h>
#include <octomap_ros/conversions.h>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <octomap/octomap.h>

#include "active_exploration_utils.h"
#include "math_utils.h"
#include "transform_utils.h"
#include "io_utils.h"
#include "octomap_utils.h"
#include "pc_utils.h"

#define _SAVE_DIRECTORY "/home/tpat8946/Data/TUW/Results"
#define _RESULTS_FILE "results.txt"
#define _PROB_FILE_PREFIX "probabilities_"
#define _PROB_FILE_EXT ".txt"
#define _CLOUD_FILE_PREFIX "cloud_"
#define _R_RATE 10
#define _N_TIMEOUT 5
#define _MAX_OBJECT_DISTANCE 1.5
#define _MIN_OBJECT_HEIGHT 0.075
#define _MAX_OBJECT_HEIGHT 1.0
#define _MIN_OBJECT_LENGTH 0.01
#define _MAX_OBJECT_LENGTH 1.0
#define _TABLE_HEIGHT_THRESHOLD 0.5
#define _VOXEL_OVERLAP_THRESHOLD 0.25
#define _DEFAULT_TREE_RESOLUTION 0.05

typedef pcl::PointXYZRGB PointT;

class ActiveExploration
{
public:
    /* === CONSTRUCTORS, DESTRUCTORS, OPERATORS, INITIALIZATION, CLEAR === */

    ActiveExploration();

    ActiveExploration(const ActiveExploration &that);

    ~ActiveExploration();

    ActiveExploration& operator=(const ActiveExploration &that);

    bool initialize(int argc, char **argv);

    bool initialize(ros::NodeHandle *node);

    void clear();

    void clear_sensor_input();

    void clear_hypotheses();

    /* === CALLBACKS === */

    void point_cloud_callback(const sensor_msgs::PointCloud2Ptr &msg);

    void image_callback(const sensor_msgs::ImagePtr &msg);

    /* === IO === */

    bool read_input(const std::string &cloud_name, const std::string &transform_name, const std::string &saliency_name);

    bool data_from_sensor(const Eigen::Matrix4f &transform = Eigen::Matrix4f::Identity());

    bool valid_input();

    bool valid_cloud();

    bool valid_image();

    /* === PROCESS === */

    bool process();

    bool process(const std::vector<std::vector<int> > &segment_indices);

    bool process(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, const Eigen::Matrix4f &transform,
                 const std::vector<std::vector<int> > &segment_indices = std::vector<std::vector<int> >());

    bool process(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image,
                 const std::vector<std::vector<int> > &segment_indices = std::vector<std::vector<int> >());

    bool process(const ros_CloudTX &ctx, const sensor_msgs::Image &image,
                 const std::vector<std::vector<int> > &segment_indices = std::vector<std::vector<int> >());

    bool process_current_cloud();

    /* === SEGMENTATION === */

    bool segment();

    bool segment(const std::vector<std::vector<int> > &segment_indices);

    bool filter_segments();

    bool extract_segment_octree_keys();

    /* === POSE ESTIMATION / OBJECT TRACKING === */

    bool estimate_pose();

    /* === CLASSIFICATION === */

    bool classify();

    /* === GET ENTROPY MAPS === */

    bool retrieve_entropy_maps();

    bool retrieve_entropy_maps(std::vector<std::vector<EntMap> > &entropy_maps);

    /* === COMPUTE ENTROPY === */

    bool compute_entropy();

    bool rank_entropy();

    /* === UPDATE HYPOTHESES === */

    bool update_hypotheses();

    bool update_hypotheses(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, const Eigen::Matrix4f transform,
                           const std::vector<std::vector<int> > &segment_indices = std::vector<std::vector<int> >());

    bool update_hypotheses(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image,
                           const std::vector<std::vector<int> > &segment_indices = std::vector<std::vector<int> >());

    bool update_hypotheses(const ros_CloudTX &ctx, const sensor_msgs::Image &image,
                           const std::vector<std::vector<int> > &segment_indices = std::vector<std::vector<int> >());

    /* === PLANNING === */

    bool plan(int &next_best_index, const SIM_TYPE &sim, const double &variance,
              const std::vector<Eigen::Vector4f> &map_locations = std::vector<Eigen::Vector4f>());

    /* === VISUALIZATION === */

    bool initialize_visualization();

    bool visualize();

    bool visualize_background(const std::vector<int> &planes, const std::vector<int> &invalids,
                              const float &ground_height = std::numeric_limits<float>::infinity());

    bool visualize_planning_world(const Eigen::Vector4f &best_location, const std::vector<Eigen::Vector4f> &map_locations, const SIM_TYPE &sim);

    bool visualize_in_model_frame(const Eigen::Vector4f &location, const EntMap &emap, const Eigen::Matrix4f &instance_to_map_tf,
                                  const Eigen::Matrix4f &instance_to_model_tf);

    bool visualize_segment_overlap(const pcl::PointCloud<PointT> &previous_cloud, const std::vector<std::vector<int> > &previous_segments,
                                   const std::vector<Pose> &previous_poses, const std::vector<std::vector<octomap::OcTreeKey> > &previous_keys,
                                   const std::vector<std::vector<int> > &overlaps);

    bool visualize_expected_clouds(const Eigen::Vector4f &location, const std::vector<int> &seg_indices,
                                   const std::vector<std::vector<pcl::PointCloud<PointT> > > &expected_clouds,
                                   const std::vector<std::vector<std::vector<int> > > &visible_indices);

    /* === CHECK DATA === */

    bool check_vector_sizes();

    /* === SETTERS === */

    void set_data(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, const Eigen::Matrix4f &transform);

    void set_table_height_threshold(const double &table_height_threshold);

    void turn_on_visualization();

    void turn_off_visualization();

    void turn_on_saving(const int &num_objects = -1, const int &num_classes = -1);

    void turn_off_saving();

    /* === GETTERS === */

    ros::NodeHandle* get_ros_node_handle();
    ros::NodeHandle* get_ros_node_handle() const;

    Eigen::Vector4f get_position();
    Eigen::Vector4f get_position() const;

    pcl::PointCloud<PointT> get_cloud();
    pcl::PointCloud<PointT> get_cloud() const;

    pcl::PointCloud<PointT> get_transformed_cloud();
    pcl::PointCloud<PointT> get_transformed_cloud() const;

    sensor_msgs::PointCloud2 get_current_cloud();
    sensor_msgs::PointCloud2 get_current_cloud() const;

    sensor_msgs::Image get_image();
    sensor_msgs::Image get_image() const;

    Eigen::Matrix4f get_transform();
    Eigen::Matrix4f get_transform() const;

    sensor_msgs::Image get_current_image();
    sensor_msgs::Image get_current_image() const;

    sensor_msgs::Image get_saliency_map();
    sensor_msgs::Image get_saliency_map() const;

    sensor_msgs::Image get_current_saliency_map();
    sensor_msgs::Image get_current_saliency_map() const;

    Eigen::Matrix4f get_current_transform();
    Eigen::Matrix4f get_current_transform() const;

    bool get_flag_received_cloud();
    bool get_flag_received_cloud() const;

    bool get_flag_received_image();
    bool get_flag_received_image() const;

    bool get_flag_received_transform();
    bool get_flag_received_transform() const;

    std::vector<std::vector<int> > get_segments();
    std::vector<std::vector<int> > get_segments() const;

    std::vector<std::vector<octomap::OcTreeKey> > get_segment_octree_keys();
    std::vector<std::vector<octomap::OcTreeKey> > get_segment_octree_keys() const;

    std::vector<Pose> get_poses();
    std::vector<Pose> get_poses() const;

    std::vector<squirrel_object_perception_msgs::Classification> get_class_estimates();
    std::vector<squirrel_object_perception_msgs::Classification> get_class_estimates() const;

    std::vector<std::vector<InstLookUp> > get_instance_directories();
    std::vector<std::vector<InstLookUp> > get_instance_directories() const;

    std::vector<std::vector<EntMap> > get_emaps();
    std::vector<std::vector<EntMap> > get_emaps() const;

    std::vector<std::vector<InstToMapTF> > get_instances_to_map_tfs();
    std::vector<std::vector<InstToMapTF> > get_instances_to_map_tfs() const;

    std::vector<double> get_entropies();
    std::vector<double> get_entropies() const;

    std::vector<int> get_entropy_ranking();
    std::vector<int> get_entropy_ranking() const;

    int get_num_clouds();
    int get_num_clouds() const;

    std::string get_save_directory();
    std::string get_save_directory() const;

    double get_max_object_distance();
    double get_max_object_distance() const;

    double get_min_object_height();
    double get_min_object_height() const;

    double get_max_object_height();
    double get_max_object_height() const;

    double get_min_object_length();
    double get_min_object_length() const;

    double get_max_object_length();
    double get_max_object_length() const;

    double get_table_height_threshold();
    double get_table_height_threshold() const;

    octomap::OcTree get_octree();
    octomap::OcTree get_octree() const;

    int get_expected_num_objects();
    int get_expected_num_objects() const;

    int get_expected_num_classes();
    int get_expected_num_classes() const;

    double get_voxel_overlap_threshold();
    double get_voxel_overlap_threshold() const;

private:
    /* === FUNCTIONS === */

    /* === FIX STRINGS === */

    bool fix_path_names();

    /* === IO === */

    bool initialize_results_file();

    bool initialize_object_details_file();

    bool save_octree();

    bool save_results();

    bool save_object_details(const std::vector<std::vector<int> > &associations = std::vector<std::vector<int> >(), const int &num_previous = -1);

    /* === UPDATE HYPOTHESES === */

    bool update_hypotheses(const std::vector<std::vector<octomap::OcTreeKey> > &pre_octree_keys,
                           const std::vector<squirrel_object_perception_msgs::Classification> &pre_class_estimates,
                           const std::vector<Pose> &pre_poses,
                           const std::vector<std::vector<InstLookUp> > &pre_instance_directories,
                           const std::vector<std::vector<InstToMapTF> > &pre_transforms,
                           const std::vector<std::vector<int> > &segment_indices,
                           std::vector<std::vector<int> > &overlaps);

    bool update_class_estimates(const std::vector<std::vector<int> > &overlaps,
                                const std::vector<std::vector<octomap::OcTreeKey> > &octree_keys,
                                const std::vector<squirrel_object_perception_msgs::Classification> &class_estimates,
                                const std::vector<Pose> &poses,
                                const std::vector<std::vector<InstLookUp> > &instance_directories,
                                const std::vector<std::vector<InstToMapTF> > &transforms,
                                std::vector<std::vector<int> > &updated_associations);

    bool merge_segments(const int &i, const std::vector<int> &seg);

    bool merge_segments(const std::vector<int> &first, const std::vector<int> &second, std::vector<int> &output);

    bool merge_estimates(const int &i, const squirrel_object_perception_msgs::Classification &est, const double &min_eps);

    bool merge_estimates(const squirrel_object_perception_msgs::Classification &first, const squirrel_object_perception_msgs::Classification &second,
                         const double &min_eps, squirrel_object_perception_msgs::Classification &output);

    bool merge_segment_octree_keys(const int &i, const std::vector<octomap::OcTreeKey> &keys);

    bool merge_segment_octree_keys(const std::vector<octomap::OcTreeKey> &first, const std::vector<octomap::OcTreeKey> &second,
                                   std::vector<octomap::OcTreeKey> &output);

    bool merge_poses(const int &i, const Pose &pose);

    bool merge_poses(const Pose &first, const Pose &second, Pose &output);

    /* === EXTRACT INFORMATION FROM FILE === */

    bool extract_instance_directories();

    bool transform_instances_to_map();

    bool transform_instances_to_map(std::vector<std::vector<InstToMapTF> > &transforms);

    bool transform_instances_to_map(std::vector<std::vector<Eigen::Matrix4f> > &transforms);

    /* === OCTREE === */

    bool add_to_octree();

    /* === LOCATION FROM POINT CLOUD === */

    void compute_position();

    /* === CHECK IF SEGMENT IS VALID === */

    bool is_valid_segment(const pcl::PointCloud<PointT> &cloud, const int &ix = -1);

    bool is_valid_segment(const pcl::PointCloud<PointT> &cloud, const Eigen::Matrix4f &transform, const int &ix = -1);

    /* === DATA TYPE CONVERSION === */

    bool fromROSMsg(const squirrel_active_exploration::EntropyMap &in, EntMap &out);

    /* === VISUALIZATION === */

    void run_view_segment(pcl::visualization::PCLVisualizer *viewer, const std::vector<std::vector<EntMap> > &emaps,
                          const std::vector<std::vector<Eigen::Matrix4f> > &itfs, const SIM_TYPE &sim, bool *exit_status);

    void run_view_expected_clouds(pcl::visualization::PCLVisualizer *viewer, const Eigen::Vector4f &location,
                                  const std::vector<int> &seg_indices, const std::vector<std::vector<pcl::PointCloud<PointT> > > &expected_clouds,
                                  const std::vector<std::vector<std::vector<int> > > &visible_indices,
                                  const std::vector<std::vector<EntMap> > &emaps, const std::vector<std::vector<Eigen::Matrix4f> > &itfs,
                                  bool *exit_status);

    /* === PLANNING === */

    bool next_best_view(int &next_best_index, const SIM_TYPE &sim, const std::vector<Eigen::Vector4f> &map_locations, const double &variance);

    int nearest_next_best_view(const std::vector<Eigen::Vector4f> &map_locations, const std::vector<int> &seg_indices,
                               const std::vector<std::vector<std::pair<std::vector<pcl::PointCloud<PointT> >,std::vector<Eigen::Vector4f> > > > &model_views,
                               const std::vector<std::vector<std::vector<double> > > &scaled_model_utilities,
                               const std::vector<double> &uncertainty_weight);

    int gaussian_weighted_next_best_view(const std::vector<Eigen::Vector4f> &map_locations, const std::vector<int> &seg_indices,
                                         const std::vector<std::vector<std::pair<std::vector<pcl::PointCloud<PointT> >,std::vector<Eigen::Vector4f> > > > &model_views,
                                         const std::vector<std::vector<std::vector<double> > > &scaled_model_utilities,
                                         const std::vector<double> &uncertainty_weight, const double &variance, const bool &unoccluded);

    int extracted_point_cloud_next_best_view(const std::vector<Eigen::Vector4f> &map_locations, const std::vector<int> &seg_indices,
                                             const std::vector<double> &uncertainty_weight, const SIM_TYPE &sim);

    /* === VARIABLES === */

    /* === ROS NODE HANDLE === */
    ros::NodeHandle *_n;

    /* === SUBSCRIBERS === */
    ros::Subscriber _cloud_sub; // subscriber for the point cloud from the sensor
    ros::Subscriber _image_sub;  // subscriber for the image from the sensor

    /* === PUBLISHERS === */

    /* === SERVICE CLIENTS === */
    ros::ServiceClient _seg_client;  // service client for segmentation
    squirrel_object_perception_msgs::Segment _seg_srv;  // message for segmentation service
    ros::ServiceClient _viz_init_client;  // service client for segmentation visualization initialization
    squirrel_object_perception_msgs::SegmentVisualizationInit _viz_init_srv; // message for visualization initialization
    ros::ServiceClient _viz_client;  // service client for segmentation visualization
    squirrel_object_perception_msgs::SegmentVisualizationOnce _viz_srv; // message for segmentation visualization
    ros::ServiceClient _class_client;  // service client for classification
    squirrel_object_perception_msgs::Classify _class_srv;  // message for classification service
    ros::ServiceClient _em_client;  // service client for entropy map
    squirrel_active_exploration::EntropyMap _em_srv;  // message for entropy map service
    ros::ServiceClient _em_viz_client;  // service client for entropy map visualization
    squirrel_active_exploration::EntropyMapViz _em_viz_srv;  // message for entropy map visualization service

    /* === POSITION, POINT CLOUDS, IMAGES AND TRANSFORMS === */
    Eigen::Vector4f _position;  // the current location
    pcl::PointCloud<PointT> _cloud;  // the point cloud
    pcl::PointCloud<PointT> _transformed_cloud;  // the transformed point cloud
    sensor_msgs::PointCloud2 _current_cloud;  // the current point cloud from the sensor
    std::vector<int> _current_ground_indices;  // the indices of the ground plane
    std::vector<octomap::OcTreeKey> _current_ground_keys;  // the octomap tree keys of the ground plans
    sensor_msgs::Image _image;  // the total image
    sensor_msgs::Image _current_image;  // the current image from the sensor
    sensor_msgs::Image _saliency_map;  // the total saliency map
    sensor_msgs::Image _current_saliency_map;  // the current saliency map for the image
    Eigen::Matrix4f _transform;  // the transformation to the map frame
    Eigen::Matrix4f _current_transform;  // the transformation of the current point cloud to the map frame
    std::map<octomap::OcTreeKey,std::vector<int>,compare_octree_key_struct> _map_key_to_points;  // a mapping from octree keys to the current point cloud
    bool _received_cloud;  // bool true if has received a point cloud from the sensor
    bool _received_image;  // bool true if has received an image from the sensor
    bool _received_transform;  // bool true if has received a transform for the current image

    /* === CURRENT PROCESSING RESULTS === */
    std::vector<std::vector<int> > _segments;  // vector of indices into the point cloud, each element is a segment
    std::vector<std::vector<octomap::OcTreeKey> > _segment_octree_keys;  // vector of octree keys into the octree for each segment
    std::vector<Pose> _poses;  // vector of poses for each segment
    std::vector<squirrel_object_perception_msgs::Classification> _class_estimates;  // vector of classification estimates for each segment
    std::vector<std::vector<InstLookUp> > _instance_directories;  // vector of directories for the matched instances in the classification
    std::vector<std::vector<InstToMapTF> > _instances_to_map_tfs;  // vector of transformations and scores from the instances to the map
    std::vector<std::vector<EntMap> > _emaps;  // vector of the entropy maps for each of the matched classes
    std::vector<double> _entropies;  // vector of entropies of the classification estimate for each segment
    std::vector<int> _entropy_ranking;  // vector of integers which rank the segment entropies from smallest to largest

    /* === OCTREE === */
    octomap::OcTree _tree;

    /* === MISCELLANEOUS === */
    int _num_clouds;
    std::string _save_dir;
    double _max_object_distance;
    double _min_object_height;
    double _max_object_height;
    double _min_object_length;
    double _max_object_length;
    double _table_height_threshold;
    bool _visualization_on;
    bool _saving_on;
    int _expected_num_objects;
    int _expected_num_classes;
    double _voxel_overlap_threshold;
};

#endif // ACTIVE_EXPLORATION_H
