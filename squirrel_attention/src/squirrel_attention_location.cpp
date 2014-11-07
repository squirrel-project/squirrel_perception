/*
 * squirrel_attention_location.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>
#include "squirrel_object_perception_msgs/get_saliency.h"
#include "v4r/AttentionModule/AttentionModule.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class AttentionLocationService
{
private:
  typedef pcl::PointXYZRGB PointT;
  int location_;
  cv::Point center_point_;
  //double chop_at_z_;
  //int v1_,v2_, v3_;
  ros::ServiceServer attention_;
  ros::NodeHandle *n_;
  boost::shared_ptr<AttentionModule::LocationSaliencyMap> saliencyMap_;
  //std::string model_filename_, scaling_filename_;

  bool
  calculate (squirrel_object_perception_msgs::get_saliency::Request & req, squirrel_object_perception_msgs::get_saliency::Response & response)
  {
    std::cerr << "in calculate " << std::endl;
    
    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (req.cloud, *scene);

    saliencyMap_->setWidth(scene->width);
    saliencyMap_->setHeight(scene->height);
    saliencyMap_->calculate();
    
    cv::Mat map;
//     if(saliencyMap_->getMap(map))
//     {
//       cv::imshow("map",map);
//       cv::waitKey(-1);
//     }
    
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    // convert OpenCV image to ROS message
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "image";
    cv_ptr->encoding = "bgr8";
    cv_ptr->image = map;
    
    //sensor_msgs::Image im;
    cv_ptr->toImageMsg(response.saliency_map);
    
    std::cerr << "out calculate " << std::endl;
    return(true);
  }
public:
  AttentionLocationService ()
  {
    //default values
    location_ = AttentionModule::AM_CENTER;
    center_point_ = cv::Point(0,0);
  }

  void
  initialize (int argc, char ** argv)
  {
    ros::init (argc, argv, "squirrel_attention_location_server");
    n_ = new ros::NodeHandle (  );//"~"
    n_->getParam ( "location", location_ );
    int x_, y_;
    n_->getParam ( "x", x_ );
    n_->getParam ( "y", y_ );
    center_point_ = cv::Point(x_,y_);
    
    saliencyMap_.reset(new AttentionModule::LocationSaliencyMap);
    saliencyMap_->setLocation(location_);
    if(AttentionModule::AM_LOCATION_CUSTOM == location_)
      saliencyMap_->setCenter(center_point_);
    
    attention_ = n_->advertiseService ("/squirrel_attention_location", &AttentionLocationService::calculate, this);
    ROS_INFO("Ready to get service calls...");
    ros::spin ();
  }
};

int
main (int argc, char ** argv)
{
  AttentionLocationService m;
  m.initialize (argc, argv);

  return 0;
}