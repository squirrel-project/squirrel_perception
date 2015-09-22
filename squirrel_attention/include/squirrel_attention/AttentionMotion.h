/**
 * AttentionMotion.h
 *
 * Attends to the "most" moving part in the image. What is considered "most"
 * depends on the optical flow method used. It might mean largest or fastest.
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef ATTENTION_MOTION_H
#define ATTENTION_MOTION_H

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamixel_msgs/JointState.h>

class AttentionMotion
{
public:
  AttentionMotion();
  virtual ~AttentionMotion();
  void run();

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::ServiceClient controllerSrv_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher imagePub_;
  ros::Subscriber panStateSub_, tiltStateSub_;
  boost::mutex movingMutex_;
  bool panning_, tilting_, moving_, cameraSteady_, timerStarted_;
  ros::Timer steadyTimer_;
  cv::Mat prevgray, gray, flow, cflow, frame;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void panStateCallback(const dynamixel_msgs::JointState::ConstPtr& panStateMsg);
  void tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr& tiltStateMsg);
  void cameraSteadyCallback(const ros::TimerEvent& event);
  void checkMovement();
};

#endif
