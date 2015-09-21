#ifndef ENTROPY_MAP_H
#define ENTROPY_MAP_H

#include "ros/ros.h"
#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Pose.h"

#include <cstdlib>
#include <string>
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "squirrel_active_exploration/pcl_conversions.h"
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

#include <squirrel_object_perception_msgs/Classify.h>

#include "math_utils.h"
#include "octomap_utils.h"
#include "io_utils.h"
#include "pc_utils.h"
#include "squirrel_active_exploration/EntropyMap.h"
#include "squirrel_active_exploration/EntropyMapViz.h"

#define _VIEW_PREFIX "view_"
#define _TRANSFORM_PREFIX "pose_"
#define _CENTROID_PREFIX "centroid_"
#define _SURFACE_AREA_PREFIX "entropy_"
#define _CLASS_ENTROPY_PREFIX "class_entropy_"
#define _SELF_PROB_PREFIX "self_prob_"
#define _OCTREE_FILENAME "tree.ot"
#define _BINARY_OCTREE_FILENAME "tree.bt"

#define _TRAINING_RADIUS 4
#define _DOWNSAMPLE_FOR_OCTREE 0.005
#define _OCTREE_RESOLUTION 0.05

typedef pcl::PointXYZRGB PointT;

struct SensorPose
{
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
};




/* ****************************
   *** INSTANCE ENTROPY MAP ***
   ****************************
*/

class InstanceEntropyMap
{
public:
    InstanceEntropyMap();

    ~InstanceEntropyMap();

    bool initialize(const std::string &training_directory, const std::string &class_name,
                    const std::string &instance_name, const std::string &descriptor_name);

    void clear();

    bool load_data();

    bool is_valid_classification_data();

    bool compute_classification_data(ros::ServiceClient &class_client, squirrel_object_perception_msgs::Classify &class_srv);

    bool visualize(const int &view_ix = -1, const std::string &score_type = "");

    /* === GETTERS === */

    std::string get_training_directory();
    std::string get_training_directory() const;

    std::string get_class_name();
    std::string get_class_name() const;

    std::string get_instance_name();
    std::string get_instance_name() const;

    std::string get_descriptor_name();
    std::string get_descriptor_name() const;

    std::vector<int*> get_view_indices();
    std::vector<int*> get_view_indices() const;

    std::vector<pcl::PointCloud<PointT>::Ptr> get_point_clouds();
    std::vector<pcl::PointCloud<PointT>::Ptr> get_point_clouds() const;

    std::vector<Eigen::Matrix4f*> get_cloud_transforms();
    std::vector<Eigen::Matrix4f*> get_cloud_transforms() const;

    std::vector<Eigen::Vector4f*> get_cloud_centroids();
    std::vector<Eigen::Vector4f*> get_cloud_centroids() const;

    std::vector<SensorPose*> get_camera_poses();
    std::vector<SensorPose*> get_camera_poses() const;

    std::vector<double*> get_surface_area_proportions();
    std::vector<double*> get_surface_area_proportions() const;

    std::vector<double*> get_class_entropies();
    std::vector<double*> get_class_entropies() const;

    std::vector<double*> get_self_probabilities();
    std::vector<double*> get_self_probabilities() const;

    pcl::PointCloud<PointT>::Ptr get_combined_cloud();
    pcl::PointCloud<PointT>::Ptr get_combined_cloud() const;

    std::string get_octree_file();
    std::string get_octree_file() const;

    size_t size();
    size_t size() const;

private:
    std::string _training_directory;
    std::string _class_name;
    std::string _instance_name;
    std::string _descriptor_name;

    std::vector<int*> _view_indices;
    std::vector<pcl::PointCloud<PointT>::Ptr> _point_clouds;
    std::vector<Eigen::Matrix4f*> _cloud_transforms;
    std::vector<Eigen::Vector4f*> _cloud_centroids;
    std::vector<SensorPose*> _camera_poses;
    std::vector<double*> _surface_area_proportions;
    std::vector<double*> _class_entropies;
    std::vector<double*> _self_probabilities;

    pcl::PointCloud<PointT>::Ptr _combined_cloud;
    std::string _octree_file;

    /* === PRIVATE FUNCTIONS === */

    void run_viewer(pcl::visualization::PCLVisualizer *viewer);

    void run_view_instance(pcl::visualization::PCLVisualizer *viewer, const int &view_index, const std::string &score_type);
};



/* *************************
   *** CLASS ENTROPY MAP ***
   *************************
*/

class ClassEntropyMap
{
public:
    ClassEntropyMap();

    ~ClassEntropyMap();

    bool initialize(const std::string &training_directory, const std::string &class_name, const std::string &descriptor_name);

    void clear();

    bool load_data();

    InstanceEntropyMap* extract_map(const std::string &key);

    bool compute_classification_data(ros::ServiceClient &class_client, squirrel_object_perception_msgs::Classify &class_srv);

    /* === GETTERS === */

    std::string get_training_directory();
    std::string get_training_directory() const;

    std::string get_class_name();
    std::string get_class_name() const;

    std::string get_descriptor_name();
    std::string get_descriptor_name() const;

    std::map<std::string,InstanceEntropyMap*> get_instance_maps();
    std::map<std::string,InstanceEntropyMap*> get_instance_maps() const;

private:
    std::string _training_directory;
    std::string _class_name;
    std::string _descriptor_name;

    std::map<std::string,InstanceEntropyMap*> _instance_maps;
};




/* *******************
   *** ENTROPY MAP ***
   *******************
*/

class EntropyMap
{
public:
    EntropyMap();

    ~EntropyMap();

    bool initialize(int argc, char **argv);

    bool initialize(ros::NodeHandle *node);

    void clear();

    bool load_data();

    ClassEntropyMap* extract_class_map(const std::string &key);

    InstanceEntropyMap* extract_instance_map(const std::string &key);

    InstanceEntropyMap* extract_instance_map(const std::string &class_key, const std::string &instance_key);

    bool compute_classification_data();

    void inspect();

    /* === VISUALIZE === */

    //void visualize(const int &class_ix = -1, const int &instance_ix = -1, const int &view_ix = -1, const std::string &entropy_type = "");

    void visualize(const std::string &class_name, const std::string &instance_name, const int &view_index, const std::string &entropy_type);

    /* === GETTERS === */

    std::string get_training_directory();
    std::string get_training_directory() const;

    std::string get_descriptor_name();
    std::string get_descriptor_name() const;

    std::map<std::string,ClassEntropyMap*> get_class_maps();
    std::map<std::string,ClassEntropyMap*> get_class_maps() const;

    /* === ROS SERVICE CALLBACKS === */

    bool extract_entropy_map(squirrel_active_exploration::EntropyMap::Request &req,
                             squirrel_active_exploration::EntropyMap::Response &response);

    bool visualize_entropy_map(squirrel_active_exploration::EntropyMapViz::Request &req,
                               squirrel_active_exploration::EntropyMapViz::Response &response);

private:
    ros::NodeHandle *_n;  // ros node handle

    ros::ServiceClient _class_client;  // service client for classification
    squirrel_object_perception_msgs::Classify _class_srv;  // message for classification service

    ros::ServiceServer _service_entropy_map;  // offer the entropy map as a service
    ros::ServiceServer _service_entropy_map_visualize;  // offer to visualize an entropy map as a service

    std::string _training_directory;
    std::string _descriptor_name;
    bool _do_classification;
    bool _do_inspect;

    std::map<std::string,ClassEntropyMap*> _class_maps;
};

#endif // ENTROPY_MAP_H
