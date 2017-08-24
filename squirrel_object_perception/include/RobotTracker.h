/**
 * The robot tracker tracks the robots position until the service gets stopped
 * @author Edith Langer
 * @date 2017-08
 */


#ifndef SQUIRREL_ROBOT_TRACKER_HPP
#define SQUIRREL_ROBOT_TRACKER_HPP

#include <ros/ros.h>
#include <squirrel_object_perception_msgs/StartRobotTracking.h>
#include <squirrel_object_perception_msgs/StopRobotTracking.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

class RobotTracker
{
private:
  ros::NodeHandle *n_;
  ros::ServiceServer startTrackingService_;
  ros::ServiceServer stopTrackingService_;
  ros::Subscriber pose_sub;
  tf::TransformListener tf_listener;

  ros::Time last_pub_;
  ros::Duration period_;

  bool startedTracking;
  std::vector<geometry_msgs::Point> robotPositions;

  void robotTrackingCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);
  bool startTracking(squirrel_object_perception_msgs::StartRobotTracking::Request &req, squirrel_object_perception_msgs::StartRobotTracking::Response &response);
  bool stopTracking(squirrel_object_perception_msgs::StopRobotTracking::Request &req, squirrel_object_perception_msgs::StopRobotTracking::Response &response);

public:
  RobotTracker();
  ~RobotTracker();

  void initialize(int argc, char ** argv);
};

#endif

