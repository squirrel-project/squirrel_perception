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

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class AttentionMotion
{
public:
  AttentionMotion();
  virtual ~AttentionMotion();
  void run();

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::ServiceClient controllerClient_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher imagePub_;
  cv::Mat prevgray, gray, flow, cflow, frame;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
