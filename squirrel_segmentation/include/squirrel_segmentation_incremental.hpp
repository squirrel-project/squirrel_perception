/*
 * squirrel_segmentation_incremental.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>

#include "squirrel_object_perception_msgs/segment_init.h"
#include "squirrel_object_perception_msgs/segment_once.h"

#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/SurfaceSegmenter/segmentation.hpp"
#include "v4r/EPUtils/EPUtils.hpp"
#include "v4r/AttentionModule/AttentionModule.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#ifndef SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_
#define SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_

class SegmenterIncremental
{
private:
  typedef pcl::PointXYZRGB PointT;
  ros::ServiceServer segment_init_;
  ros::ServiceServer segment_once_;
  ros::NodeHandle *n_;
  boost::shared_ptr<segmentation::Segmenter> segmenter_;
  std::string model_filename_, scaling_filename_;

  bool
  segmentInit (squirrel_object_perception_msgs::segment_init::Request & req, squirrel_object_perception_msgs::segment_init::Response & response);
  
  bool
  segmentOnce (squirrel_object_perception_msgs::segment_once::Request & req, squirrel_object_perception_msgs::segment_once::Response & response);
public:
  SegmenterIncremental ();
  ~SegmenterIncremental ();

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_