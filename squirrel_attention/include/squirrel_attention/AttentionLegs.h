/**
 * AttentionLegs.h
 *
 * Attends to the legs in the laser data.
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef ATTENTION_LEGS_H
#define ATTENTION_LEGS_H

#include <ros/ros.h>
#include <std_msgs/String.h>

class AttentionLegs
{
public:
  AttentionLegs();
  virtual ~AttentionLegs();
  void run();

private:
  ros::NodeHandle nh_;
  ros::Subscriber legsSub_;
  ros::ServiceClient controllerSrv_;

  void legsCallback(const std_msgs::String& msg);
};

#endif
