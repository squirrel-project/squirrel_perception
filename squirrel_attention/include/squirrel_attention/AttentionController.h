/**
 * AttentionController.h
 *
 * Gets to look to from various attention channels and decides how to move the
 * camera (and robot if necessary).
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef ATTENTION_CONTROLLER_H
#define ATTENTION_CONTROLLER_H

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <robotino_msgs/LookAtImagePosition.h>
#include <robotino_msgs/LookAtPosition.h>
#include <robotino_msgs/LookAtPanTilt.h>
#include <robotino_msgs/FixatePosition.h>
#include <robotino_msgs/ClearFixation.h>

class AttentionController
{
public:
  AttentionController();
  virtual ~AttentionController();
  void run();

private:
  ros::NodeHandle nh_;
  ros::Publisher panPub_, tiltPub_;
  ros::ServiceServer lookImageSrv_, lookSrv_, fixateSrv_, clearSrv_, lookPanTiltSrv_;
  ros::Subscriber panStateSub_, tiltStateSub_;
  tf::TransformListener listener_;
  boost::mutex jointMutex_;
  float pan_, tilt_;

  bool lookAtImagePosition(robotino_msgs::LookAtImagePosition::Request &req,
                           robotino_msgs::LookAtImagePosition::Response &res);
  bool lookAtPosition(robotino_msgs::LookAtPosition::Request &req,
                      robotino_msgs::LookAtPosition::Response &res);
  bool lookAtPanTilt(robotino_msgs::LookAtPanTilt::Request &req,
                      robotino_msgs::LookAtPanTilt::Response &res);
  bool fixatePosition(robotino_msgs::FixatePosition::Request &req,
                      robotino_msgs::FixatePosition::Response &res);
  bool clearFixation(robotino_msgs::ClearFixation::Request &req,
                      robotino_msgs::ClearFixation::Response &res);
  void panStateCallback(const dynamixel_msgs::JointState::ConstPtr& panStateMsg);
  void tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr& tiltStateMsg);
};

#endif
