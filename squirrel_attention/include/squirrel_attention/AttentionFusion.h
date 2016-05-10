/**
 * AttentionFusion.h
 *
 * Attends to the legs in the laser data and faces in the camera data.
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef ATTENTION_FUSION_H
#define ATTENTION_FUSION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
#include <people_msgs/People.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

class AttentionFusion
{
public:
  AttentionFusion();
  virtual ~AttentionFusion();
  void run();

private:
  ros::NodeHandle nh_;
  ros::Subscriber legsSub_;
  ros::Subscriber attentionSub;
  ros::Publisher fakespeechPub;
  ros::ServiceClient controllerSrv_;
  ros::Time last_observation_;
  boost::mutex observeMutex_;
  ros::Timer timer;
  ros::Timer facetimer;
  tf::TransformListener listener_;
  geometry_msgs::PointStamped next_;
  std::string reason_;

  void legsCallback(const people_msgs::People& msg);
  void robotInFovCallback(const std_msgs::String& msg);
  void observeTimerCallback(const ros::TimerEvent&);
  void faceTimerCallback(const ros::TimerEvent&);
};

#endif
