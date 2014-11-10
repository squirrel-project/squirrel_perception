/*
 * squirrel_segmentation_incremental.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>

#include "squirrel_object_perception_msgs/segment_start.h"
#include "squirrel_object_perception_msgs/segment_once.h"

#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/SurfaceSegmenter/segmentation.hpp"
#include "v4r/EPUtils/EPUtils.hpp"
#include "v4r/AttentionModule/AttentionModule.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class SegmenterIncremental
{
private:
  typedef pcl::PointXYZRGB PointT;
  ros::ServiceServer segment_;
  ros::NodeHandle *n_;
  boost::shared_ptr<segmentation::Segmenter> segmenter_;
  std::string model_filename_, scaling_filename_;

  bool
  segmentStart (squirrel_object_perception_msgs::segment_start::Request & req, squirrel_object_perception_msgs::segment_start::Response & response)
  {
    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (req.cloud, *scene);
    std::cout << scene->points.size() << std::endl;
    
    
    //get saliency map
    std::vector<cv::Mat> saliencyMaps;
    
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req.saliency_map, sensor_msgs::image_encodings::BGR8);
    cv::Mat salMap = cv_ptr->image;
    
    saliencyMaps.resize(1);
    salMap.copyTo(saliencyMaps.at(0));
    
    segmenter_->setPointCloud(scene);
    segmenter_->setSaliencyMaps(saliencyMaps);
  
    segmenter_->attentionSegmentInit();
/*    segmenter_->attentionSegmentNext();
    
    //segmenter_->attentionSegment(1);
  
    //std::vector<cv::Mat> masks = segmenter.getMasks();
    std::vector<std::vector<int> > clusters = segmenter_->getSegmentedObjectsIndices();
    
    std::cout << scene->points.size() << " " << clusters.size() << std::endl;
    for(size_t i=0; i < clusters.size(); i++)
    {
        std_msgs::Int32MultiArray indx;
    	for(size_t k=0; k < clusters[i].size(); k++)
    	{
            indx.data.push_back(clusters[i][k]);
        }
    	//response.clusters_indices.push_back(indx);
    }*/
    
    return true;
  }
  
  bool
  segmentNext (squirrel_object_perception_msgs::segment_once::Request & req, squirrel_object_perception_msgs::segment_once::Response & response)
  {
    //pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    //pcl::fromROSMsg (req.cloud, *scene);
    //std::cout << scene->points.size() << std::endl;
    
    
    //get saliency map
    //std::vector<cv::Mat> saliencyMaps;
    
    //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req.saliency_map, sensor_msgs::image_encodings::BGR8);
    //cv::Mat salMap = cv_ptr->image;
    
    //saliencyMaps.resize(1);
    //salMap.copyTo(saliencyMaps.at(0));
    
    //segmenter_->setPointCloud(scene);
    //segmenter_->setSaliencyMaps(saliencyMaps);
  
    //segmenter_->attentionSegmentInit();
    segmenter_->attentionSegmentNext();
    
    //segmenter_->attentionSegment(1);
  
    //std::vector<cv::Mat> masks = segmenter.getMasks();
    std::vector<std::vector<int> > clusters = segmenter_->getSegmentedObjectsIndices();
    
    std::cout << clusters.size() << std::endl;
    for(size_t i=0; i < clusters.size(); i++)
    {
        std_msgs::Int32MultiArray indx;
    	for(size_t k=0; k < clusters[i].size(); k++)
    	{
            indx.data.push_back(clusters[i][k]);
        }
    	response.cluster_indices = indx;
    }
    
    return true;
  }
public:
  SegmenterIncremental ()
  {
    //default values
  }

  void
  initialize (int argc, char ** argv)
  {
    ros::init (argc, argv, "squirrel_segmentation_incremental_server");
    n_ = new ros::NodeHandle ();
    n_->getParam ( "model_filename", model_filename_ );
    n_->getParam ( "scaling_filename", scaling_filename_ );
    
    if (model_filename_.compare ("") == 0)
    {
      PCL_ERROR ("Set -model_filename option in the command line, ABORTING");
      return;
    }
    
    if (scaling_filename_.compare ("") == 0)
    {
      PCL_ERROR ("Set -scaling_filename option in the command line, ABORTING");
      return;
    }
    
    segmenter_.reset(new segmentation::Segmenter);
    segmenter_->setModelFilename(model_filename_);
    segmenter_->setScaling(scaling_filename_);
    
    segment_ = n_->advertiseService ("/squirrel_segmentation_incremental_start", &SegmenterIncremental::segmentStart, this);
    segment_ = n_->advertiseService ("/squirrel_segmentation_incremental_once", &SegmenterIncremental::segmentNext, this);
    std::cout << "Ready to get service calls..." << std::endl;
    ros::spin ();
  }
};

int
main (int argc, char ** argv)
{
  SegmenterIncremental m;
  m.initialize (argc, argv);

  return 0;
}
