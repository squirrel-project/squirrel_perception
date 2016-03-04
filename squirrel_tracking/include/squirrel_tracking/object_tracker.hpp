/**
 * squirrel_tracking.hpp
 *
 * This wraps the mono image object tracker from the v4r/KeypointSlam package by Hannes Prankl.
 *
 * @date March 2015
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 */

#ifndef SQUIRREL_OBJECT_TRACKER_HPP
#define SQUIRREL_OBJECT_TRACKER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <v4r/tracking/ObjectTrackerMono.h>
#include <squirrel_object_perception_msgs/StartInstanceTracking.h>
#include <squirrel_object_perception_msgs/StopInstanceTracking.h>

class SquirrelTrackingNode
{
private:
  ros::NodeHandle *n_;
  v4r::ObjectTrackerMono::Ptr tracker;
  ros::ServiceServer startTrackingService_;
  ros::ServiceServer stopTrackingService_;
  ros::Subscriber imageSubscriber;
  ros::Subscriber caminfoSubscriber;
  bool haveCameraInfo;
  bool startedTracking;
  std::string modelPath;
  tf::TransformBroadcaster tfBroadcast;
  std::string trackedObjectId;
  std::string trackedObjectClass;
  cv::Mat intrinsic;
  cv::Mat dist;
  std::string cameraName;

  void receiveCameraInfo(const sensor_msgs::CameraInfo::ConstPtr &msg);
  void receiveImage(const sensor_msgs::Image::ConstPtr &msg);
  bool startTracking(squirrel_object_perception_msgs::StartInstanceTracking::Request &req, squirrel_object_perception_msgs::StartInstanceTracking::Response &response);
  bool stopTracking(squirrel_object_perception_msgs::StopInstanceTracking::Request &req, squirrel_object_perception_msgs::StopInstanceTracking::Response &response);

public:
  SquirrelTrackingNode();
  ~SquirrelTrackingNode();

  void initialize(int argc, char ** argv);
};

#endif
