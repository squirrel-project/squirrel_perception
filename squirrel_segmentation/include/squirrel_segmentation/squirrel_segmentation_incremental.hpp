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
#include <squirrel_segmentation/pcl_conversions.h>

#include "squirrel_object_perception_msgs/SegmentInit.h"
#include "squirrel_object_perception_msgs/SegmentOnce.h"

#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/SurfaceSegmenter/segmentation.hpp"
#include "v4r/EPUtils/EPUtils.hpp"
#include "v4r/AttentionModule/AttentionModule.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#ifndef SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_
#define SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_

class SegmenterIncremental
{
private:
  static const double MAX_OBJECT_DIST = 1.5;
  static const double MAX_OBJECT_HEIGHT = 0.25;
  static const double MIN_OBJECT_HEIGHT = 0.03;

  class PersistentObject
  {
  public:
    static int cnt;
    geometry_msgs::Point pos;  // position, in base_link
    double size;  // diameter of bounding sphere
    std::string name;

    PersistentObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cluster, double x, double y, double z)
    {
      std::stringstream ss;
      ss << "cluster" << cnt++;
      name = ss.str();
      pos.x = x;
      pos.y = y;
      pos.z = z;
      size = 0.;
      for(size_t i = 0; i < cloud_cluster->points.size(); i++)
      {
        geometry_msgs::Point dp;
        dp.x = cloud_cluster->points[i].x - pos.x;
        dp.y = cloud_cluster->points[i].y - pos.y;
        dp.z = cloud_cluster->points[i].z - pos.z;
        double d = sqrt(dp.x*dp.x + dp.y*dp.y + dp.z*dp.z);
        size = std::max(size, 2*d);
      }
    }
    bool isSame(const PersistentObject &other)
    {
      geometry_msgs::Point d;
      d.x = other.pos.x - pos.x;
      d.y = other.pos.y - pos.y;
      d.z = other.pos.z - pos.z;
      if(sqrt(d.x*d.x + d.y*d.y + d.z*d.z) < std::min(size/2., other.size/2.))
        return true;
      else
        return false;
    }
  };

  typedef pcl::PointXYZRGB PointT;
  ros::ServiceServer SegmentInit_;
  ros::ServiceServer SegmentOnce_;
  ros::NodeHandle *n_;
  boost::shared_ptr<segmentation::Segmenter> segmenter_;
  std::string model_filename_, scaling_filename_;
  pcl::PointCloud<PointT>::Ptr scene;
  tf::TransformListener tf_listener;
  // list of all known objects ever encountered
  std::list<PersistentObject> knownObjects;

  bool
  segmentInit (squirrel_object_perception_msgs::SegmentInit::Request & req, squirrel_object_perception_msgs::SegmentInit::Response & response);
  
  bool
  segmentOnce (squirrel_object_perception_msgs::SegmentOnce::Request & req, squirrel_object_perception_msgs::SegmentOnce::Response & response);

  geometry_msgs::PoseStamped kinect2base_link(double x, double y, double z);
  geometry_msgs::PoseStamped base_link2kinect(double x, double y, double z);
  geometry_msgs::PoseStamped base_link2map(double x, double y, double z);
  geometry_msgs::PoseStamped transform(double x, double y, double z, const std::string &from, const std::string &to);
  void transformCluster2base_link(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cluster);
  bool checkCluster(std::vector<int> &cluster_indices);
  bool isValidCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cluster, Eigen::Vector4f &centroid);

public:
  SegmenterIncremental ();
  ~SegmenterIncremental ();

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_
