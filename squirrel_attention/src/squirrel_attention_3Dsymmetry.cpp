/*
 * squirrel_attention_3Dsymmetry.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#include <squirrel_attention/squirrel_attention_3Dsymmetry.hpp>

bool 
Attention3DSymmetryService::calculate (squirrel_object_perception_msgs::GetSaliency3DSymmetry::Request & req, squirrel_object_perception_msgs::GetSaliency3DSymmetry::Response & response)
{
  pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (req.cloud, *scene);
  printf("Attention3DSymmetryService::calculate: point cloud is organised? %s\n", (scene->isOrganized() ? "yes" : "no")); // HACK
  // HACK: The gezebo somilated kinect seems to output a non-orgnized point cloud. Just fix that here.
  if(scene->height == 1)
  {
    if(scene->points.size() == 640*480)
    {
      scene->height = 480;
      scene->width = 640;
    }
  }
  printf("Attention3DSymmetryService::calculate: point cloud is organised? %s\n", (scene->isOrganized() ? "yes" : "no")); // HACK

  //prepare point cloud
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
  // HACK MZ
  scene_indices->indices.resize(scene->points.size());
  for(size_t i = 0; i < scene->points.size(); i++)
    scene_indices->indices[i] = i;
  // HACK END
  printf("[DEBUG]: Attention3DSymmetryService::calculate: 1 indices size %d normals %d\n", (int)scene_indices->indices.size(), (int)normals->points.size());
  int ret = preparePointCloud(scene,coefficients,normals,scene_indices);
  printf("[DEBUG]: Attention3DSymmetryService::calculate: 2 preparePointCloud returned: %d\n", ret);

  // start creating parameters
  saliencyMap_.reset(new AttentionModule::Symmetry3DMap);
  saliencyMap_->setWidth(scene->width);
  saliencyMap_->setHeight(scene->height);
  saliencyMap_->setCloud(scene);
  saliencyMap_->setIndices(scene_indices);
  saliencyMap_->setNormals(normals);
  saliencyMap_->setCombinationType(AttentionModule::AM_COMB_SUM);
  saliencyMap_->setNormalizationType(EPUtils::NT_NONE);
  
  saliencyMap_->calculatePyramid(AttentionModule::SIMPLE_PYRAMID);
  
  cv::Mat map;
  if(!(saliencyMap_->getMap(map)))
  {
    return(false);
  }
//   cv::imshow("map",map);
//   cv::waitKey(-1);
  
  resize(map,map,cv::Size(scene->width,scene->height));
  map.convertTo(map,CV_8U,255);

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  ros::Time time = ros::Time::now();
  // convert OpenCV image to ROS message
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "image";
  cv_ptr->encoding = "mono8";
  cv_ptr->image = map;
    
  //sensor_msgs::Image im;
  cv_ptr->toImageMsg(response.saliency_map);
  return(true);
}

void
Attention3DSymmetryService::initialize (int argc, char ** argv)
{
  ros::init (argc, argv, "squirrel_attention_3Dsymmetry_server");
  n_ = new ros::NodeHandle ();
    
  attention_ = n_->advertiseService ("/squirrel_attention_3Dsymmetry", &Attention3DSymmetryService::calculate, this);
  ROS_INFO("Ready to get service calls...");
  ros::spin ();
}

int
main (int argc, char ** argv)
{
  Attention3DSymmetryService m;
  m.initialize (argc, argv);

  return 0;
}