/**
 * AttentionControllerNode.h
 *
 * Gets to look to from various attention channels and decides how to move the
 * camera (and robot if necessary).
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef ATTENTION_CONTROLLER_H
#define ATTENTION_CONTROLLER_H

#include <ros/ros.h>

#include <squirrel_object_perception_msgs/LookAtImagePosition.h>
#include <squirrel_object_perception_msgs/LookAtPosition.h>
#include <squirrel_object_perception_msgs/FixatePosition.h>
#include <squirrel_object_perception_msgs/ClearFixation.h>

class AttentionController
{
public:
  AttentionController();
  virtual ~AttentionController();

private:
  ros::NodeHandle nh_;
  ros::Publisher panPub_, tiltPub_;
  ros::ServiceServer lookImageSrv_, lookSrv_, fixateSrv_, clearSrv_;

  bool lookAtImagePosition(squirrel_object_perception_msgs::LookAtImagePosition::Request &req,
                           squirrel_object_perception_msgs::LookAtImagePosition::Response &res);
  bool lookAtPosition(squirrel_object_perception_msgs::LookAtPosition::Request &req,
                      squirrel_object_perception_msgs::LookAtPosition::Response &res);
  bool fixatePosition(squirrel_object_perception_msgs::FixatePosition::Request &req,
                      squirrel_object_perception_msgs::FixatePosition::Response &res);
  bool clearFixation(squirrel_object_perception_msgs::ClearFixation::Request &req,
                      squirrel_object_perception_msgs::ClearFixation::Response &res);
};

#endif
