/*
 * squirrel_attention_location.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>
#include "squirrel_object_perception_msgs/get_saliency_location.h"
#include "v4r/AttentionModule/AttentionModule.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#ifndef SQUIRREL_ATTENTION_LOCATION_HPP_
#define SQUIRREL_ATTENTION_LOCATION_HPP_

class AttentionLocationService
{
private:
  typedef pcl::PointXYZRGB PointT;
  int location_;
  cv::Point center_point_;
  ros::ServiceServer attention_;
  ros::NodeHandle *n_;
  boost::shared_ptr<AttentionModule::LocationSaliencyMap> saliencyMap_;
  
  bool
  calculate (squirrel_object_perception_msgs::get_saliency_location::Request & req, squirrel_object_perception_msgs::get_saliency_location::Response & response);
  
public:
  AttentionLocationService ();
  ~AttentionLocationService ();

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_ATTENTION_LOCATION_HPP_