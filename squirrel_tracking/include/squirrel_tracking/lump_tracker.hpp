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
#include <squirrel_object_perception_msgs/StartLumpTracking.h>
#include <squirrel_object_perception_msgs/StopLumpTracking.h>

class LumpTracker
{
private:
  typedef pcl::PointXYZ PointT;
  static const double MAX_OBJECT_DIST = 0.8;
  static const double MAX_OBJECT_HEIGHT = 0.25;

  ros::NodeHandle *n_;
  ros::ServiceServer startTrackingService_;
  ros::ServiceServer stopTrackingService_;
  ros::Subscriber pointcloudSubscriber;
  bool startedTracking;
  tf::TransformBroadcaster tfBroadcast;
  pcl::PointCloud<PointT>::Ptr cloud_;
  tf::TransformListener tf_listener;
  std::string cameraName;
  std::string trackedObjectId;

  geometry_msgs::PoseStamped kinect2base_link(double x, double y, double z);
  geometry_msgs::PoseStamped base_link2kinect(double x, double y, double z);
  geometry_msgs::PoseStamped transform(double x, double y, double z, const std::string &from, const std::string &to);
  void tranformCluster2base_link(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster);
  bool isValidCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster);

  void receivePointcloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool startTracking(squirrel_object_perception_msgs::StartLumpTracking::Request &req, squirrel_object_perception_msgs::StartLumpTracking::Response &response);
  bool stopTracking(squirrel_object_perception_msgs::StopLumpTracking::Request &req, squirrel_object_perception_msgs::StopLumpTracking::Response &response);

public:
  LumpTracker();
  ~LumpTracker();

  void initialize(int argc, char ** argv);
};

#endif

