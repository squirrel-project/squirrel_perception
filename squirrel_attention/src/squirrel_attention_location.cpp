/*
 * squirrel_attention_location.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#include <squirrel_attention_location.hpp>

bool 
AttentionLocationService::calculate (squirrel_object_perception_msgs::get_saliency::Request & req, squirrel_object_perception_msgs::get_saliency::Response & response)
{
  //std::cerr << "in calculate " << std::endl;
  
  pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (req.cloud, *scene);

  saliencyMap_->setWidth(scene->width);
  saliencyMap_->setHeight(scene->height);
  saliencyMap_->calculate();
    
  cv::Mat map;
  saliencyMap_->getMap(map);
  map.convertTo(map,CV_8U,255);
//     {
//       cv::imshow("map",map);
//       cv::waitKey(-1);
//     }
    
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  ros::Time time = ros::Time::now();
  // convert OpenCV image to ROS message
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "image";
  cv_ptr->encoding = "mono8";
  cv_ptr->image = map;
    
  //sensor_msgs::Image im;
  cv_ptr->toImageMsg(response.saliency_map);
    
  //std::cerr << "out calculate " << std::endl;
  return(true);
}

AttentionLocationService::AttentionLocationService ()
{
  //default values
  location_ = AttentionModule::AM_CENTER;
  center_point_ = cv::Point(0,0);
}

AttentionLocationService::~AttentionLocationService ()
{
  if(n_)
    delete n_;
}

void
AttentionLocationService::initialize (int argc, char ** argv)
{
  ros::init (argc, argv, "squirrel_attention_location_server");
  n_ = new ros::NodeHandle ();
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

int
main (int argc, char ** argv)
{
  AttentionLocationService m;
  m.initialize (argc, argv);

  return 0;
}