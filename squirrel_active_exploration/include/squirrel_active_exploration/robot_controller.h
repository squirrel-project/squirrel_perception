#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "squirrel_active_exploration/pcl_conversions.h"

#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/CreatePRM.h>
#include <rosplan_knowledge_msgs/AddWaypoint.h>
#include <rosplan_knowledge_msgs/Filter.h>
#include <std_srvs/Empty.h>

#include <move_base/move_base.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>

#include "math_utils.h"
#include "transform_utils.h"

#define _TF_ATTEMPTS 3
#define _KINECT "/kinect_depth_frame"
#define _BASE "/base_link"
#define _MAP "/map"

#define _SRV_ATTEMPTS 3
#define _MARKER_NS "waypoint_markers"
#define _MAX_DRIVE_STEP 0.4
#define _MIN_DIST_ERROR 0.05
#define _MAX_ANGLE_STEP 0.1
#define _MIN_ANGLE_ERROR 0.01
#define _SPLIT_ITERATIONS 3

typedef pcl::PointXYZRGB PointT;

class RobotController
{
public:
    RobotController();

    ~RobotController();

    bool initialize(int argc, char **argv);

    bool initialize(ros::NodeHandle *node);

    void clear();

//    /* === CALLBACKS === */

//    void octomap_callback(const octomap_msgs::Octomap &msg);

    /* === TRANSFORMS === */

    bool pointcloud_to_baselink(const pcl::PointCloud<PointT> &in_cloud, pcl::PointCloud<PointT> &out_cloud);

    bool pointcloud_to_baselink(const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud);

    bool baselink_to_map(const pcl::PointCloud<PointT> &in_cloud, pcl::PointCloud<PointT> &out_cloud);

    bool baselink_to_map(const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud);

    bool pointcloud_to_map(const pcl::PointCloud<PointT> &in_cloud, pcl::PointCloud<PointT> &out_cloud);

    bool pointcloud_to_map(const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud);

    Eigen::Matrix4f pointcloud_to_baselink_tf();

    Eigen::Matrix4f baselink_to_map_tf();

    Eigen::Matrix4f pointcloud_to_map_tf();

    /* === OCTOMAP === */

    bool retrieve_octomap();

    /* === CONTROL === */

    bool move_to_waypoint(const double &x, const double &y);

    bool move_to_waypoint(const double &x, const double &y) const;

    bool move_to_waypoint_slow(const double &x, const double &y);

    bool move_to_waypoint_slow(const double &x, const double &y) const;

    bool rotate(const double &x, const double &y);

    bool rotate(const double &x, const double &y) const;

    bool rotate_slow(const double &x, const double &y);

    bool rotate_slow(const double &x, const double &y) const;

    /* === ROBOT INFORMATION === */

    bool robot_position_in_map(Eigen::Vector4f &position);

    bool robot_position_in_map(Eigen::Vector4f &position) const;

    /* === VISUALIZATION === */

    void visualize(const pcl::PointCloud<PointT>::Ptr in_cloud);

    void visualize(const pcl::PointCloud<PointT> &in_cloud);

    void visualize(const sensor_msgs::PointCloud2Ptr in_cloud);

    void visualize(const sensor_msgs::PointCloud2 &in_cloud);

    /* === GETTERS === */

    ros::NodeHandle* get_ros_node_handle();
    ros::NodeHandle* get_ros_node_handle() const;

    octomap::OcTree* get_octree();
    octomap::OcTree* get_octree() const;

    bool transforms_available();
    bool transforms_available() const;

private:
    ros::NodeHandle *_n;  // ros node handle

    octomap::OcTree *_octree;  // octomap

    bool _received_octomap;
    bool _received_pc_to_bl_tf;
    bool _received_bl_to_mp_tf;
    bool _received_pc_to_mp_tf;
    bool _all_tfs;

    /* === PLANNING === */
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *_move_base_ac;
    ros::Publisher _marker_pub;
    int _wpt_id;

    /* === PRIVATE FUNCTIONS === */

    void listen_all_transforms();

    bool listen_transform(const std::string &frame1, const std::string &frame2);
};

// Website for octomap
// http://alufr-ros-pkg.googlecode.com/svn/tags/stacks/octomap_mapping/octomap_mapping-0.4.8/octomap_server/src/octomap_saver.cpp

#endif // ROBOT_CONTROLLER_H
