#include "squirrel_active_exploration/robot_controller.h"

using namespace std;
using namespace pcl;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

RobotController::RobotController()
    : _received_pc_to_bl_tf (false),
      _received_bl_to_mp_tf (false),
      _received_pc_to_mp_tf (false),
      _all_tfs (false),
      _wpt_id (0)
{
    _octree = NULL;
}

RobotController::~RobotController()
{
    // Delete the ros node handle
    if (_n)
        delete _n;
    // Delete the octree
    if (_octree)
        delete _octree;
    if (_move_base_ac)
        delete _move_base_ac;
    // Clear all data
    clear();
}

bool RobotController::initialize(int argc, char **argv)
{
    ROS_INFO("RobotController::initialize : starting");
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle *node (new ros::NodeHandle("~"));
    if (!initialize (node))
    {
        ROS_ERROR("RobotController::initialize : could not initialize with the node");
        return false;
    }
    clear();
}

bool RobotController::initialize(ros::NodeHandle *node)
{
    // Assign the ros node handle
    _n = node;

    // Look up transform from kinect to robot frame
    listen_all_transforms();
    if (_all_tfs)
    {
        ROS_INFO("RobotController::initialize : successfully got all TFs %s to %s", _KINECT, _BASE);
    }
    else
    {
        if (!_received_pc_to_bl_tf)
            ROS_WARN("RobotController::initialize : unsuccessfully got TF %s to %s", _KINECT, _BASE);
        if (!_received_bl_to_mp_tf)
            ROS_WARN("RobotController::initialize : unsuccessfully got TF %s to %s", _BASE, _MAP);
        if (!_received_pc_to_mp_tf)
            ROS_WARN("RobotController::initialize : unsuccessfully got TF %s to %s", _KINECT, _MAP);
    }

    //_nav_goal_pub = _n->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

    _move_base_ac = new MoveBaseClient("move_base", true);

    // Wait for the action server to come up
    while (!_move_base_ac->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("RobotController::initialize : waiting for the move_base action server to come up");
    }

    _n->advertise<visualization_msgs::Marker>("visualization_marker", 1);

    return true;
}

void RobotController::clear()
{
}

/* === TRANSFORMS === */

bool RobotController::pointcloud_to_baselink(const PointCloud<PointT> &in_cloud, PointCloud<PointT> &out_cloud)
{
    if (_received_pc_to_bl_tf)
    {
        return frame_to_frame(in_cloud, out_cloud, _KINECT, _BASE);
    }
    else
    {
        ROS_WARN("RobotController::pointcloud_to_baselink  : not yet established the TF, listeing again");
        int count = 0;
        while (count < _TF_ATTEMPTS && !_received_pc_to_bl_tf)
        {
            listen_transform(_KINECT, _BASE);
            if (_received_pc_to_bl_tf)
                _received_pc_to_bl_tf = true;
            else
                ROS_WARN("RobotController::pointcloud_to_baselink  : waiting for TF %s to %s [%u]", _KINECT, _BASE, count);
            ++count;
        }
        // If succesfully established the TF, call the function again
        if (_received_pc_to_bl_tf)
            return pointcloud_to_baselink(in_cloud, out_cloud);
        // Otherwise return false
        return false;
    }
}

bool RobotController::pointcloud_to_baselink(const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud)
{
    // Convert to pcl::PointCloud
    PointCloud<PointT> pcl_in_cloud;
    fromROSMsg(in_cloud, pcl_in_cloud);
    PointCloud<PointT> pcl_out_cloud;
    // Call the transform function
    if (!(pointcloud_to_baselink(pcl_in_cloud, pcl_out_cloud)))
    {
        ROS_ERROR("RobotController::pointcloud_to_baselink : error in transforming point cloud");
        return false;
    }
    // Convert back to ros message
    toROSMsg(pcl_out_cloud, out_cloud);
    return true;
}

bool RobotController::baselink_to_map(const PointCloud<PointT> &in_cloud, PointCloud<PointT> &out_cloud)
{
    if (_received_bl_to_mp_tf)
    {
        return frame_to_frame(in_cloud, out_cloud, _BASE, _MAP);
    }
    else
    {
        ROS_WARN("RobotController::baselink_to_map : not yet established the TF, listeing again");
        int count = 0;
        while (count < _TF_ATTEMPTS && !_received_pc_to_bl_tf)
        {
            listen_transform(_BASE, _MAP);
            if (_received_pc_to_bl_tf)
                _received_pc_to_bl_tf = true;
            else
                ROS_WARN("RobotController::baselink_to_map : waiting for TF %s to %s [%u]", _BASE, _MAP, count);
            ++count;
        }
        // If succesfully established the TF, call the function again
        if (_received_bl_to_mp_tf)
            return baselink_to_map(in_cloud, out_cloud);
        // Otherwise return false
        return false;
    }
}

bool RobotController::baselink_to_map(const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud)
{
    // Convert to pcl::PointCloud
    PointCloud<PointT> pcl_in_cloud;
    fromROSMsg(in_cloud, pcl_in_cloud);
    PointCloud<PointT> pcl_out_cloud;
    // Call the transform function
    if (!(baselink_to_map(pcl_in_cloud, pcl_out_cloud)))
    {
        ROS_ERROR("RobotController::baselink_to_map : error in transforming point cloud");
        return false;
    }
    // Convert back to ros message
    toROSMsg(pcl_out_cloud, out_cloud);
    return true;
}

bool RobotController::pointcloud_to_map(const PointCloud<PointT> &in_cloud, PointCloud<PointT> &out_cloud)
{
    if (_received_pc_to_mp_tf)
    {
        return frame_to_frame(in_cloud, out_cloud, _KINECT, _MAP);
    }
    else
    {
        ROS_WARN("RobotController::pointcloud_to_map : not yet established the TF, listeing again");
        int count = 0;
        while (count < _TF_ATTEMPTS && !_received_pc_to_bl_tf)
        {
            listen_transform(_KINECT, _MAP);
            if (_received_pc_to_bl_tf)
                _received_pc_to_bl_tf = true;
            else
                ROS_WARN("RobotController::pointcloud_to_map : waiting for TF %s to %s [%u]", _KINECT, _MAP, count);
            ++count;
        }
        // If succesfully established the TF, call the function again
        if (_received_pc_to_mp_tf)
            return pointcloud_to_map(in_cloud, out_cloud);
        // Otherwise return false
        return false;
    }
}

bool RobotController::pointcloud_to_map(const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud)
{
    // Convert to pcl::PointCloud
    PointCloud<PointT> pcl_in_cloud;
    fromROSMsg(in_cloud, pcl_in_cloud);
    PointCloud<PointT> pcl_out_cloud;
    // Call the transform function
    if (!(pointcloud_to_map(pcl_in_cloud, pcl_out_cloud)))
    {
        ROS_ERROR("RobotController::pointcloud_to_map : error in transforming point cloud");
        return false;
    }
    // Convert back to ros message
    toROSMsg(pcl_out_cloud, out_cloud);
    return true;
}

Eigen::Matrix4f RobotController::pointcloud_to_baselink_tf()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    if (!frame_to_frame_tf(_KINECT, _BASE, transform))
        ROS_ERROR("RobotController::pointcloud_to_baselink_tf : could not get transform from %s to %s", _KINECT, _BASE);
    return transform;
}

Eigen::Matrix4f RobotController::baselink_to_map_tf()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    if (!frame_to_frame_tf(_BASE, _MAP, transform))
        ROS_ERROR("RobotController::pointcloud_to_baselink_tf : could not get transform from %s to %s", _BASE, _MAP);
    return transform;
}

Eigen::Matrix4f RobotController::pointcloud_to_map_tf()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    if (!frame_to_frame_tf(_KINECT, _MAP, transform))
        ROS_ERROR("RobotController::pointcloud_to_baselink_tf : could not get transform from %s to %s", _KINECT, _MAP);
    return transform;
}

/* === OCTOMAP === */

bool RobotController::retrieve_octomap()
{
    octomap_msgs::GetOctomap::Request req;
    octomap_msgs::GetOctomap::Response resp;
    int count = 0;
    bool success = false;
    while (!success && count < _SRV_ATTEMPTS)
    {
        if (!ros::service::call("octomap_full", req, resp))
        {
            ROS_WARN("RobotController::retrieve_octomap : failed to get octomap");
            usleep(1000000);
        }
        else
        {
            ROS_INFO("RobotController::retrieve_octomap : retrieved octomap");
            success = true;
        }
        ++count;
    }
    // If could not get the octomap
    if (!success)
    {
        ROS_ERROR("RobotController::retrieve_octomap : could not retrieve the octomap");
        return false;
    }
    _octree = NULL;
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(resp.map);
    if (!tree)
    {
        ROS_ERROR("RobotController::retrieve_octomap : could not convert the octomap message");
        return false;
    }
    _octree = dynamic_cast<octomap::OcTree*>(tree);
    if (!_octree)
    {
        ROS_ERROR("RobotController::retrieve_octomap : could not convert octomap::AbstractOcTree to octomap::OcTree");
        return false;
    }

    delete tree;
    return true;
}

/* === CONTROL === */

bool RobotController::move_to_waypoint(const double &x, const double &y)
{
//    // Publish the marker to rviz
//    visualization_msgs::Marker marker;
//    marker.header.frame_id = "map";
//    marker.header.stamp = ros::Time();
//    marker.ns = _MARKER_NS;
//    marker.id = _wpt_id;
//    marker.type = visualization_msgs::Marker::SPHERE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = x;
//    marker.pose.position.y = y;
//    marker.pose.position.z = 0.1;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = orientation;
//    marker.pose.orientation.w = 1.0;
//    marker.scale.x = 2;
//    marker.scale.y = 2.1;
//    marker.scale.z = 2.1;
//    marker.color.a = 1.0; // Don't forget to set the alpha!
//    marker.color.r = 0.0;
//    marker.color.g = 1.0;
//    marker.color.b = 0.0;
//    _marker_pub.publish(marker);

    int max_iters = _SPLIT_ITERATIONS - 1;
    for (int i = 0; i <= max_iters; ++i)
    {
        PointCloud<PointT> map_pos;
        map_pos.resize(1);
        map_pos.points[0].x = x;
        map_pos.points[0].y = y;
        map_pos.points[0].z = 0.0;
        PointCloud<PointT> base_pos;
        if (!frame_to_frame(_MAP, _BASE, map_pos, base_pos))
        {
            ROS_ERROR("RobotController::move_to_waypoint : could not transform location to base_link frame");
            return false;
        }
        // First rotate to the new waypoint
        if (!rotate(x, y))
        {
            ROS_ERROR("RobotController::move_to_waypoint : could not transform location to base_link frame");
            return false;
        }

        // Second drive forward until the goal is reached
        double dist = sqrt(pow(base_pos.points[0].x,2) + pow(base_pos.points[0].y,2));
        if (i < max_iters)
            dist = dist / 2.0;
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = dist;
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("RobotController::move_to_waypoint: driving forward to waypoint by %.4f meters", dist);
        _move_base_ac->sendGoal(goal);
        _move_base_ac->waitForResult();
        if (_move_base_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("RobotController::move_to_waypoint : successfully drove to waypoint");
        else
            ROS_WARN("RobotController::move_to_waypoint : failed to drive to waypoint");
        ros::Duration(0.1).sleep();


    }

    PointCloud<PointT> robot_pos;
    robot_pos.resize(1);
    robot_pos.points[0].x = 0.0;
    robot_pos.points[0].y = 0.0;
    robot_pos.points[0].z = 0.0;
    PointCloud<PointT> robot_map_pos;
    if (!frame_to_frame(_BASE, _MAP, robot_pos, robot_map_pos))
        ROS_ERROR("RobotController::move_to_waypoint : could not transform robot location to map frame");
    else
        ROS_INFO("RobotController::move_to_waypoint : final robot location is [%.2f, %.2f]", robot_map_pos.points[0].x, robot_map_pos.points[0].y);
    double dist = distance3D(x, y, 0, robot_map_pos.points[0].x, robot_map_pos.points[0].y, 0);
    ROS_WARN("RobotController::move_to_waypoint : final location error of %.2f meters", dist);

    return true;
}

bool RobotController::move_to_waypoint(const double &x, const double &y) const
{
    return move_to_waypoint(x, y);
}

bool RobotController::move_to_waypoint_slow(const double &x, const double &y)
{
//    // Publish the marker to rviz
//    visualization_msgs::Marker marker;
//    marker.header.frame_id = "map";
//    marker.header.stamp = ros::Time();
//    marker.ns = _MARKER_NS;
//    marker.id = _wpt_id;
//    marker.type = visualization_msgs::Marker::SPHERE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = x;
//    marker.pose.position.y = y;
//    marker.pose.position.z = 0.1;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = orientation;
//    marker.pose.orientation.w = 1.0;
//    marker.scale.x = 2;
//    marker.scale.y = 2.1;
//    marker.scale.z = 2.1;
//    marker.color.a = 1.0; // Don't forget to set the alpha!
//    marker.color.r = 0.0;
//    marker.color.g = 1.0;
//    marker.color.b = 0.0;
//    _marker_pub.publish(marker);


    // First do a normal move to waypoint
    if (!move_to_waypoint(x,y))
    {
        ROS_ERROR("RobotController::move_to_waypoint_slow : could not move to waypoint");
        return false;
    }

    // Print out the information
    ROS_INFO("RobotController::move_to_waypoint_slow : goal location was [%.2f, %.2f]", x, y);
    PointCloud<PointT> robot_pos;
    robot_pos.resize(1);
    robot_pos.points[0].x = 0.0;
    robot_pos.points[0].y = 0.0;
    robot_pos.points[0].z = 0.0;
    PointCloud<PointT> robot_map_pos;
    if (!frame_to_frame(_BASE, _MAP, robot_pos, robot_map_pos))
        ROS_ERROR("RobotController::move_to_waypoint_slow : could not transform robot location to map frame");
    else
        ROS_INFO("RobotController::move_to_waypoint_slow : robot location is [%.2f, %.2f]", robot_map_pos.points[0].x, robot_map_pos.points[0].y);
    // Print out the error
    double dist = distance3D(x, y, 0, robot_map_pos.points[0].x, robot_map_pos.points[0].y, 0);
    ROS_WARN("RobotController::move_to_waypoint_slow : location error of %.2f meters", dist);

    // If the error is more than
    if (dist > _MIN_DIST_ERROR)
    {
        int num_tries = 0;
        int max_num_tries = dist / _MAX_DRIVE_STEP + 2;

        while (true)
        {
            if (dist <= _MIN_DIST_ERROR)
                break;
            // If dist is greater than the maximum dist
            if (dist > _MAX_DRIVE_STEP)
                dist = _MAX_DRIVE_STEP;
            else
                break;

            // First rotate to the new waypoint
            if (!rotate(x, y))
            {
                ROS_ERROR("RobotController::move_to_waypoint_slow : could not transform location to base_link frame");
                return false;
            }

            // Second drive forward until the goal is reached
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = dist;
            goal.target_pose.pose.orientation.w = 1.0;
            ROS_INFO("RobotController::move_to_waypoint_slow: driving forward to waypoint by %.4f meters", dist);
            _move_base_ac->sendGoal(goal);
            _move_base_ac->waitForResult();
            if (_move_base_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("RobotController::move_to_waypoint_slow : successfully drove to waypoint");
            else
                ROS_WARN("RobotController::move_to_waypoint_slow : failed to drive to waypoint");
            ros::Duration(0.1).sleep();

            ++num_tries;
            if (num_tries >= max_num_tries)
                break;

            robot_pos.clear();
            robot_pos.resize(1);
            robot_pos.points[0].x = 0.0;
            robot_pos.points[0].y = 0.0;
            robot_pos.points[0].z = 0.0;
            robot_map_pos.clear();
            if (!frame_to_frame(_BASE, _MAP, robot_pos, robot_map_pos))
            {
                ROS_ERROR("RobotController::move_to_waypoint_slow : could not transform robot location to map frame");
                return false;
            }
            dist = distance3D(x, y, 0, robot_map_pos.points[0].x, robot_map_pos.points[0].y, 0);
        }

        robot_pos.clear();
        robot_pos.resize(1);
        robot_pos.points[0].x = 0.0;
        robot_pos.points[0].y = 0.0;
        robot_pos.points[0].z = 0.0;
        robot_map_pos.clear();
        if (!frame_to_frame(_BASE, _MAP, robot_pos, robot_map_pos))
            ROS_ERROR("RobotController::move_to_waypoint_slow : could not transform robot location to map frame");
        else
            ROS_INFO("RobotController::move_to_waypoint_slow : final robot location is [%.2f, %.2f]", robot_map_pos.points[0].x, robot_map_pos.points[0].y);
        dist = distance3D(x, y, 0, robot_map_pos.points[0].x, robot_map_pos.points[0].y, 0);
        ROS_WARN("RobotController::move_to_waypoint_slow : final location error of %.2f meters", dist);
    }

    return true;
}

bool RobotController::move_to_waypoint_slow(const double &x, const double &y) const
{
    return move_to_waypoint_slow(x, y);
}

bool RobotController::rotate(const double &x, const double &y)
{
    int max_iters = _SPLIT_ITERATIONS - 1;
    for (int i = 0; i <= max_iters; ++i)
    {
        PointCloud<PointT> map_pos;
        map_pos.resize(1);
        map_pos.points[0].x = x;
        map_pos.points[0].y = y;
        map_pos.points[0].z = 0.0;
        PointCloud<PointT> base_pos;
        if (!frame_to_frame(_MAP, _BASE, map_pos, base_pos))
        {
            ROS_ERROR("RobotController::rotate : could not transform location to base_link frame");
            return false;
        }
        double angle_rotation = atan2(base_pos.points[0].y, base_pos.points[0].x);
        if (i < max_iters)
            angle_rotation = angle_rotation / 2.0;

        // Rotate to the goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.orientation.z = angle_rotation;
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("RobotController::rotate: rotating by %.4f radians", angle_rotation);
        _move_base_ac->sendGoal(goal);
        _move_base_ac->waitForResult();
        if (_move_base_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("RobotController::rotate: successfully rotated");
        else
            ROS_WARN("RobotController::rotate : failed to rotate");

        ros::Duration(0.1).sleep();
    }

    return true;
}

bool RobotController::rotate(const double &x, const double &y) const
{
    return rotate(x, y);
}

bool RobotController::rotate_slow(const double &x, const double &y)
{
    // First do a normal rotation
    if (!rotate(x, y))
    {
        ROS_ERROR("RobotController::rotate_slow : could not transform location to base_link frame");
        return false;
    }

    // Now refine the rotation
    PointCloud<PointT> map_pos;
    map_pos.resize(1);
    map_pos.points[0].x = x;
    map_pos.points[0].y = y;
    map_pos.points[0].z = 0.0;
    PointCloud<PointT> base_pos;
    if (!frame_to_frame(_MAP, _BASE, map_pos, base_pos))
    {
        ROS_ERROR("RobotController::rotate_slow : could not transform location to base_link frame");
        return false;
    }
    double angle_rotation = atan2(base_pos.points[0].y, base_pos.points[0].x);

    int num_tries = 0;
    double max_num_tries = angle_rotation / _MAX_ANGLE_STEP + 2;
    while (true)
    {
        if (angle_rotation < _MIN_ANGLE_ERROR)
            break;
        if (angle_rotation > _MAX_ANGLE_STEP)
            angle_rotation = _MAX_ANGLE_STEP;
        else
            break;
        // Rotate to the goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.orientation.z = angle_rotation;
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("RobotController::rotate_slow: rotating by %.4f radians", angle_rotation);
        _move_base_ac->sendGoal(goal);
        _move_base_ac->waitForResult();
        if (_move_base_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("RobotController::rotate_slow: successfully rotated");
        else
            ROS_WARN("RobotController::rotate_slow : failed to rotate");

        ros::Duration(0.1).sleep();

        ++num_tries;
        if (num_tries >= max_num_tries)
            break;

        map_pos.clear();
        map_pos.resize(1);
        map_pos.points[0].x = x;
        map_pos.points[0].y = y;
        map_pos.points[0].z = 0.0;
        base_pos.clear();
        if (!frame_to_frame(_MAP, _BASE, map_pos, base_pos))
        {
            ROS_ERROR("RobotController::rotate_slow : could not transform location to base_link frame");
            return false;
        }
        angle_rotation = atan2(base_pos.points[0].y, base_pos.points[0].x);
    }

    if (angle_rotation > _MIN_ANGLE_ERROR)
    {
        // Rotate to the goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.orientation.z = angle_rotation;
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("RobotController::rotate: rotating by %.4f radians", angle_rotation);
        _move_base_ac->sendGoal(goal);
        _move_base_ac->waitForResult();
        if (_move_base_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("RobotController::rotate_slow: successfully rotated");
        else
            ROS_WARN("RobotController::rotate_slow : failed to rotate");

        ros::Duration(0.1).sleep();

        map_pos.clear();
        map_pos.resize(1);
        map_pos.points[0].x = x;
        map_pos.points[0].y = y;
        map_pos.points[0].z = 0.0;
        base_pos.clear();
        if (!frame_to_frame(_MAP, _BASE, map_pos, base_pos))
        {
            ROS_ERROR("RobotController::rotate_slow : could not transform location to base_link frame");
            return false;
        }
    }

    return true;
}

bool RobotController::rotate_slow(const double &x, const double &y) const
{
    return rotate_slow(x, y);
}

/* === ROBOT INFORMATION === */

bool RobotController::robot_position_in_map(Eigen::Vector4f &position)
{
    PointCloud<PointT> robot_pos;
    robot_pos.resize(1);
    robot_pos.points[0].x = 0.0;
    robot_pos.points[0].y = 0.0;
    robot_pos.points[0].z = 0.0;
    PointCloud<PointT> robot_map_pos;
    if (!frame_to_frame(_BASE, _MAP, robot_pos, robot_map_pos))
    {
        ROS_ERROR("RobotController::robot_position_in_map : could not transform robot location to map frame");
        return false;
    }
    else
    {
        ROS_INFO("RobotController::robot_position_in_map : robot location is [%.2f, %.2f]", robot_map_pos.points[0].x, robot_map_pos.points[0].y);
        position[0] = robot_map_pos.points[0].x;
        position[1] = robot_map_pos.points[0].y;
        position[2] = robot_map_pos.points[0].z;
        position[3] = 0.0;
        return true;
    }
}

bool RobotController::robot_position_in_map(Eigen::Vector4f &position) const
{
    return robot_position_in_map(position);
}

/* === VISUALIZATION === */

void RobotController::visualize(const PointCloud<PointT>::Ptr in_cloud)
{
    ROS_INFO("RobotController::visualize : starting");
    PointCloud<PointT>::Ptr robot_pos (new PointCloud<PointT>());
    robot_pos->resize(1);
    robot_pos->points[0].x = 0;
    robot_pos->points[0].y = 0;
    robot_pos->points[0].z = 0;
    visualization::PCLVisualizer viewer("Point cloud from robot");
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // Define R,G,B colors for the point cloud
    visualization::PointCloudColorHandlerCustom<PointT> cloud_handler (in_cloud, 255, 255, 255);  // White
    visualization::PointCloudColorHandlerCustom<PointT> robot_handler (robot_pos, 20, 20, 230);  // Blue
    // Add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (in_cloud, cloud_handler, "cloud");
    viewer.addPointCloud (robot_pos, robot_handler, "robot");
    // Set the point cloud properties
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "robot");

    // Wait for the viewer to close before exiting
    while (!viewer.wasStopped ())
        viewer.spinOnce ();

    ROS_INFO("RobotController::visualize : finished");
}

void RobotController::visualize(const pcl::PointCloud<PointT> &in_cloud)
{
    PointCloud<PointT>::Ptr ptr_cloud (new PointCloud<PointT>(in_cloud));
    visualize(ptr_cloud);
}

void RobotController::visualize(const sensor_msgs::PointCloud2Ptr in_cloud)
{
    visualize (*in_cloud);
}

void RobotController::visualize(const sensor_msgs::PointCloud2 &in_cloud)
{
    // Convert to a pcl::PointCloud
    PointCloud<PointT>::Ptr pcl_in_cloud (new PointCloud<PointT>());
    fromROSMsg (in_cloud, *pcl_in_cloud);
    // Visualize
    visualize (pcl_in_cloud);
}

/* === GETTERS === */

ros::NodeHandle* RobotController::get_ros_node_handle()
{
    return _n;
}

ros::NodeHandle* RobotController::get_ros_node_handle() const
{
    return _n;
}

octomap::OcTree* RobotController::get_octree()
{
    return _octree;
}

octomap::OcTree* RobotController::get_octree() const
{
    return _octree;
}

bool RobotController::transforms_available()
{
    return _all_tfs;
}

bool RobotController::transforms_available() const
{
    return _all_tfs;
}

/* === PRIVATE FUNCTIONS === */

void RobotController::listen_all_transforms()
{
    // Kinect to baselink
    _all_tfs = false;
    if (!_received_pc_to_bl_tf)
    {
        if (listen_transform(_KINECT, _BASE))
            _received_pc_to_bl_tf = true;
        else
            _all_tfs = false;
    }
    // Baselink to map
    if (!_received_bl_to_mp_tf)
    {
        if (listen_transform(_BASE, _MAP))
            _received_bl_to_mp_tf = true;
        else
            _all_tfs = false;
    }
    // Kinect to map
    if (!_received_pc_to_mp_tf)
    {
        if (listen_transform(_KINECT, _MAP))
            _received_pc_to_mp_tf = true;
        else
            _all_tfs = false;
    }
    if (_received_pc_to_bl_tf && _received_bl_to_mp_tf && _received_pc_to_mp_tf)
        _all_tfs = true;
}

bool RobotController::listen_transform(const string &frame1, const string &frame2)
{
    tf::TransformListener listener (ros::Duration(1.0));
    tf::StampedTransform tf;
    ros::Time now = ros::Time(0);
    ros::Duration(1.0).sleep();
    try
    {
        ROS_INFO("RobotController::listen_transform : waiting for TF %s to %s", frame1.c_str(), frame2.c_str());
        listener.lookupTransform("/map", "/base_link", now, tf);
        //listener.waitForTransform(frame1, frame2, now, ros::Duration(0.2));
        return true;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("RobotController::listen_transform : could not hear TF %s to %s", frame1.c_str(), frame2.c_str());
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}
