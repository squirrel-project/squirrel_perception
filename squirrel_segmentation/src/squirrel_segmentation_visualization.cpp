/*
 * squirrel_segmentation_visualization.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include <squirrel_segmentation/squirrel_segmentation_visualization.hpp>

bool
SegmenterVisualization::segmentVisualizationInit (squirrel_object_perception_msgs::SegmentVisualizationInit::Request & req, squirrel_object_perception_msgs::SegmentVisualizationInit::Response & response)
{
  //get point cloud
  pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (req.cloud, *scene);
  ROS_INFO ("Number of points in the scene: %ld", scene->points.size());

  // create image ot publish
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  EPUtils::pointCloudXYZRGB_2_cloudXYZimageRGB(scene,scene_xyz,RGB,scene->width,scene->height);

  //publish saliency map
  SaliencyPub_.publish(req.saliency_map);

//   //get saliency map
//   cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req.saliency_map, sensor_msgs::image_encodings::MONO8);
//   cv::Mat salMap = cv_ptr->image;
//   salMap.convertTo(salMap,CV_32F,1.0/255);

//   cv::imshow("salMap",salMap);
//   cv::waitKey(-1);

  //publish color image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  ros::Time time = ros::Time::now();
  // convert OpenCV image to ROS message
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "image";
  cv_ptr->encoding = "bgr8";
  cv_ptr->image = RGB;

  sensor_msgs::Image im;
  cv_ptr->toImageMsg(im);
  SegmentationPub_.publish(im);

  ROS_INFO("Finished initialization.");
  return true;
}

bool
SegmenterVisualization::segmentVisualizationOnce (squirrel_object_perception_msgs::SegmentVisualizationOnce::Request & req, squirrel_object_perception_msgs::SegmentVisualizationOnce::Response & response)
{
  ROS_INFO ("Going to show an object.");

  srand (time(NULL));

  uchar r = std::rand()%255;
  uchar g = std::rand()%255;
  uchar b = std::rand()%255;

  assert(req.clusters_indices.size() == 1);
  for(size_t k=0; k < req.clusters_indices[0].data.size(); k++)
  {
    int idx = req.clusters_indices[0].data[k];

    cv::Vec3b &cvp = RGB.at<cv::Vec3b> (idx/RGB.cols,idx%RGB.cols);
    cvp[0] = r;
    cvp[1] = g;
    cvp[2] = b;
  }

  //publish color image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  ros::Time time = ros::Time::now();
  // convert OpenCV image to ROS message
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "image";
  cv_ptr->encoding = "bgr8";
  cv_ptr->image = RGB;

  sensor_msgs::Image im;
  cv_ptr->toImageMsg(im);
  SegmentationPub_.publish(im);

  return true;
}

SegmenterVisualization::SegmenterVisualization ()
{
  //default values
}

SegmenterVisualization::~SegmenterVisualization ()
{
  if(n_)
    delete n_;
}

void
SegmenterVisualization::initialize (int argc, char ** argv)
{
  ros::init (argc, argv, "squirrel_segmentation_visualization_server");
  n_ = new ros::NodeHandle ("~");

  SegmentVisualizationInit_ = n_->advertiseService ("/squirrel_segmentation_visualization_init", &SegmenterVisualization::segmentVisualizationInit, this);
  SegmentVisualizationOnce_ = n_->advertiseService ("/squirrel_segmentation_visualization_once", &SegmenterVisualization::segmentVisualizationOnce, this);
  SaliencyPub_ = n_->advertise<sensor_msgs::Image>("/squirrel_segmentation_visualization_saliencymap", 1000, true);
  SegmentationPub_ = n_->advertise<sensor_msgs::Image>("/squirrel_segmentation_visualization_segmentation", 1000, true);
  ROS_INFO ("Ready to get service calls...");
  ros::spin ();
}


int
main (int argc, char ** argv)
{
  SegmenterVisualization m;
  m.initialize (argc, argv);

  return 0;
}
