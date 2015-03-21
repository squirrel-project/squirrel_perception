/**
 * lump_tracker.hpp
 *
 * This simply tracks a lump of stuff in front of the robot.
 * It segments the nearest lump from the floor and returs its centroid.
 *
 * @date March 2015
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 */

#ifndef SQUIRREL_TRACKING_HPP
#define SQUIRREL_TRACKING_HPP

#include <pcl/common/common.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <squirrel_object_perception_msgs/StartObjectTracking.h>
#include <squirrel_object_perception_msgs/StopObjectTracking.h>

class SquirrelTrackingNode
{
private:
  typedef pcl::PointXYZ PointT;
  static const double MAX_OBJECT_DIST = 0.8;

  ros::NodeHandle *n_;
  ros::ServiceServer startTrackingService_;
  ros::ServiceServer stopTrackingService_;
  ros::Subscriber pointcloudSubscriber;
  bool startedTracking;
  tf::TransformBroadcaster tfBroadcast;
  pcl::PointCloud<PointT>::Ptr cloud_;
  tf::TransformListener tf_listener;

  geometry_msgs::PoseStamped kinect2base_link(double x, double y, double z);
  void receivePointcloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool startTracking(squirrel_object_perception_msgs::StartObjectTracking::Request &req, squirrel_object_perception_msgs::StartObjectTracking::Response &response);
  bool stopTracking(squirrel_object_perception_msgs::StopObjectTracking::Request &req, squirrel_object_perception_msgs::StopObjectTracking::Response &response);

public:
  SquirrelTrackingNode();
  ~SquirrelTrackingNode();

  void initialize(int argc, char ** argv);
};

#endif

