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

#include <squirrel_object_perception_msgs/LookAtImagePosition.h>
#include <squirrel_object_perception_msgs/LookAtPosition.h>
#include <squirrel_object_perception_msgs/FixatePosition.h>
#include <squirrel_object_perception_msgs/ClearFixation.h>

class AttentionController
{
public:
  AttentionController();
  virtual ~AttentionController();
  void run();

private:
  ros::NodeHandle nh_;
  ros::Publisher panPub_, tiltPub_;
  ros::ServiceServer lookImageSrv_, lookSrv_, fixateSrv_, clearSrv_;
  ros::Subscriber panStateSub_, tiltStateSub_;
  boost::mutex jointMutex_;
  float pan_, tilt_;

  bool lookAtImagePosition(squirrel_object_perception_msgs::LookAtImagePosition::Request &req,
                           squirrel_object_perception_msgs::LookAtImagePosition::Response &res);
  bool lookAtPosition(squirrel_object_perception_msgs::LookAtPosition::Request &req,
                      squirrel_object_perception_msgs::LookAtPosition::Response &res);
  bool fixatePosition(squirrel_object_perception_msgs::FixatePosition::Request &req,
                      squirrel_object_perception_msgs::FixatePosition::Response &res);
  bool clearFixation(squirrel_object_perception_msgs::ClearFixation::Request &req,
                      squirrel_object_perception_msgs::ClearFixation::Response &res);
  void panStateCallback(const dynamixel_msgs::JointState::ConstPtr& panStateMsg);
  void tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr& tiltStateMsg);
};

#endif
