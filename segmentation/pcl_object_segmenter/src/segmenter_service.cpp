/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include "perception_srv_definitions/segment.h"
#include "std_msgs/Int32MultiArray.h"

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/features/normal_3d_omp.h>

//#define DEBUG_WITH_VIS 1
#ifdef DEBUG_WITH_VIS
    #include<pcl/visualization/pcl_visualizer.h>
#endif

#include<pcl/common/transforms.h>


class PCLSegmenterService
{
private:
  typedef pcl::PointXYZRGB PointT;
  double chop_at_z_;
  int v1_,v2_, v3_;
  ros::ServiceServer segment_;
  ros::NodeHandle *n_;
  int seg_type_;
  std::string camera_frame_;
  std::string base_frame_;
  float max_angle_plane_to_ground_;
  double sensor_noise_max_;
  double table_range_min_, table_range_max_;
  int min_cluster_size_;
  int MAX_VERTICAL_PLANE_SIZE_;

  void
  getBaseCameraTransform (Eigen::Matrix<float, 4, 4, Eigen::RowMajor> & trans)
  {
    tf::StampedTransform tf_transform;
    tf::TransformListener tf_listener;

    std::cout << "Getting transform from:" << camera_frame_ << " to " << base_frame_ << std::endl;

    try
    {
      tf_listener.waitForTransform (base_frame_, camera_frame_, ros::Time (0), ros::Duration (5.0));
      tf_listener.lookupTransform (base_frame_, camera_frame_, ros::Time (0), tf_transform);
    }
    catch (tf::TransformException ex)
    {
      std::cout << ex.what () << std::endl;
    }

    //cast tf_transform to eigen::Matrix4f
    tf::Vector3 p = tf_transform.getOrigin ();
    tf::Quaternion q = tf_transform.getRotation ();

    Eigen::Vector3f translation = Eigen::Vector3f (p.getX (), p.getY (), p.getZ ());

    Eigen::Quaternion<float> rot (q.getW (), q.getX (), q.getY (), q.getZ ());
    rot.normalize();
    Eigen::Matrix3f rotation = rot.toRotationMatrix();

    trans.setIdentity();
    trans.block<3,1>(0,3) = translation;
    trans.block<3,3>(0,0) = rotation;
  }

  template<typename PointT>
    void
    doSegmentation (typename pcl::PointCloud<PointT>::Ptr & xyz_points, pcl::PointCloud<pcl::Normal>::Ptr & normal_cloud,
                    std::vector<pcl::PointIndices> & indices, Eigen::Vector4f & table_plane, int seg, std::vector<float> &supplied_transform)
    {


      if (seg == 0)
      {
        int num_plane_inliers = 1000;

        pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
        mps.setMinInliers (num_plane_inliers);
        mps.setAngularThreshold (0.017453 * 10.f); // 2 degrees
        mps.setDistanceThreshold (sensor_noise_max_); // 1cm
        mps.setInputNormals (normal_cloud);
        mps.setInputCloud (xyz_points);

        std::vector < pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
        std::vector < pcl::ModelCoefficients > model_coefficients;
        std::vector < pcl::PointIndices > inlier_indices;
        pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
        std::vector < pcl::PointIndices > label_indices;
        std::vector < pcl::PointIndices > boundary_indices;

        typename pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr ref_comp (
                                                                                                new pcl::PlaneRefinementComparator<PointT, pcl::Normal,
                                                                                                    pcl::Label> ());
        ref_comp->setDistanceThreshold (sensor_noise_max_, false);
        ref_comp->setAngularThreshold (0.017453 * 2);
        mps.setRefinementComparator (ref_comp);
        mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

        std::cout << "Number of planes found:" << model_coefficients.size () << std::endl;
        if (model_coefficients.size () == 0)
          return;

        int table_plane_selected = 0;
        int max_inliers_found = -1;
        std::vector<int> plane_inliers_counts;
        plane_inliers_counts.resize (model_coefficients.size ());

        for (size_t i = 0; i < model_coefficients.size (); i++)
        {
          Eigen::Vector4f table_plane = Eigen::Vector4f (model_coefficients[i].values[0], model_coefficients[i].values[1],
                                                         model_coefficients[i].values[2], model_coefficients[i].values[3]);

          std::cout << "Number of inliers for this plane:" << inlier_indices[i].indices.size () << std::endl;
          int remaining_points = 0;
          typename pcl::PointCloud<PointT>::Ptr plane_points (new pcl::PointCloud<PointT> (*xyz_points));
          for (int j = 0; j < plane_points->points.size (); j++)
          {
            Eigen::Vector3f xyz_p = plane_points->points[j].getVector3fMap ();

            if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
              continue;

            float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

            if (std::abs (val) > sensor_noise_max_)
            {
              plane_points->points[j].x = std::numeric_limits<float>::quiet_NaN ();
              plane_points->points[j].y = std::numeric_limits<float>::quiet_NaN ();
              plane_points->points[j].z = std::numeric_limits<float>::quiet_NaN ();
            }
            else
              remaining_points++;
          }

          plane_inliers_counts[i] = remaining_points;

          if (remaining_points > max_inliers_found)
          {
            table_plane_selected = i;
            max_inliers_found = remaining_points;
          }
        }

        size_t itt = static_cast<size_t> (table_plane_selected);
        table_plane = Eigen::Vector4f (model_coefficients[itt].values[0], model_coefficients[itt].values[1], model_coefficients[itt].values[2],
                                       model_coefficients[itt].values[3]);

        Eigen::Vector3f normal_table = Eigen::Vector3f (model_coefficients[itt].values[0], model_coefficients[itt].values[1],
                                                        model_coefficients[itt].values[2]);

        int inliers_count_best = plane_inliers_counts[itt];

        //check that the other planes with similar normal are not higher than the table_plane_selected
        for (size_t i = 0; i < model_coefficients.size (); i++)
        {
          Eigen::Vector4f model = Eigen::Vector4f (model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2],
                                                   model_coefficients[i].values[3]);

          Eigen::Vector3f normal = Eigen::Vector3f (model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);

          int inliers_count = plane_inliers_counts[i];

          std::cout << "Dot product is:" << normal.dot (normal_table) << std::endl;
          if ((normal.dot (normal_table) > 0.95) && (inliers_count_best * 0.5 <= inliers_count))
          {
            //check if this plane is higher, projecting a point on the normal direction
            std::cout << "Check if plane is higher, then change table plane" << std::endl;
            std::cout << model[3] << " " << table_plane[3] << std::endl;
            if (model[3] < table_plane[3])
            {
              PCL_WARN ("Changing table plane...");
              table_plane_selected = i;
              table_plane = model;
              normal_table = normal;
              inliers_count_best = inliers_count;
            }
          }
        }

        table_plane = Eigen::Vector4f (model_coefficients[table_plane_selected].values[0], model_coefficients[table_plane_selected].values[1],
                                       model_coefficients[table_plane_selected].values[2], model_coefficients[table_plane_selected].values[3]);

        //cluster..
        typename pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr
                                                                                       euclidean_cluster_comparator_ (
                                                                                                                      new pcl::EuclideanClusterComparator<
                                                                                                                          PointT, pcl::Normal,
                                                                                                                          pcl::Label> ());

        //create two labels, 1 one for points belonging to or under the plane, 1 for points above the plane
        label_indices.resize (2);

        for (int j = 0; j < xyz_points->points.size (); j++)
        {
          Eigen::Vector3f xyz_p = xyz_points->points[j].getVector3fMap ();

          if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
            continue;

          float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

          if (val >= sensor_noise_max_)
          {
            /*plane_points->points[j].x = std::numeric_limits<float>::quiet_NaN ();
             plane_points->points[j].y = std::numeric_limits<float>::quiet_NaN ();
             plane_points->points[j].z = std::numeric_limits<float>::quiet_NaN ();*/
            labels->points[j].label = 1;
            label_indices[0].indices.push_back (j);
          }
          else
          {
            labels->points[j].label = 0;
            label_indices[1].indices.push_back (j);
          }
        }

        std::vector<bool> plane_labels;
        plane_labels.resize (label_indices.size (), false);
        plane_labels[0] = true;

        euclidean_cluster_comparator_->setInputCloud (xyz_points);
        euclidean_cluster_comparator_->setLabels (labels);
        euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
        euclidean_cluster_comparator_->setDistanceThreshold (0.035f, true);

        pcl::PointCloud < pcl::Label > euclidean_labels;
        std::vector < pcl::PointIndices > euclidean_label_indices;
        pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
        euclidean_segmentation.setInputCloud (xyz_points);
        euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

        for (size_t i = 0; i < euclidean_label_indices.size (); i++)
        {
          if (euclidean_label_indices[i].indices.size () >= min_cluster_size_)
          {
            indices.push_back (euclidean_label_indices[i]);
          }
        }
      }
      else if(seg == 1)
      {
        pcl::apps::DominantPlaneSegmentation<PointT> dps;
        dps.setInputCloud (xyz_points);
        dps.setMaxZBounds (3.F);
        dps.setObjectMinHeight (sensor_noise_max_);
        dps.setMinClusterSize (min_cluster_size_);
        dps.setWSize (9);
        dps.setDistanceBetweenClusters (0.03f);
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
        dps.setDownsamplingSize (0.01f);
        dps.compute_fast (clusters);
        dps.getIndicesClusters (indices);
        dps.getTableCoefficients (table_plane);
      }
      else if(seg == 4)
      {
         // HACK: Michael Zillich, 2014-07: removed that option as it depends on v4r library
      }
      else
      {
        pcl::apps::DominantPlaneSegmentation<PointT> dps;
        dps.setInputCloud (xyz_points);
        dps.setMaxZBounds (3.F);
        dps.setObjectMinHeight (sensor_noise_max_);
        dps.setMinClusterSize (min_cluster_size_);
        dps.setWSize (9);
        dps.setDistanceBetweenClusters (0.03f);
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
        dps.setDownsamplingSize (0.01f);
        dps.compute_full (clusters);
        dps.getIndicesClusters (indices);
        dps.getTableCoefficients (table_plane);
      }
    }

  bool
  segment (perception_srv_definitions::segment::Request & req, perception_srv_definitions::segment::Response & response)
  {
    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (req.cloud, *scene);
    
    if(chop_at_z_ > 0)
    {
        pcl::PassThrough<PointT> pass_;
        pass_.setFilterLimits (0.f, chop_at_z_);
        pass_.setFilterFieldName ("z");
        pass_.setInputCloud (scene);
        pass_.setKeepOrganized (true);
        pass_.filter (*scene);
    }
    
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ROS_INFO("skipping computation of normals: NOTE only works if seg_type != 0!\n");
    /*pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ROS_INFO("before compute normals() set radius\n");
    ne.setRadiusSearch(0.02f);
    ROS_INFO("before compute normals() set input cloud\n");
    ne.setInputCloud (scene);
    ROS_INFO("before compute normals() compute\n");
    ne.compute (*normal_cloud);*/

    std::vector<pcl::PointIndices> indices;
    Eigen::Vector4f table_plane;
    doSegmentation<PointT>(scene, normal_cloud, indices, table_plane, seg_type_, req.transform);

    for(size_t i=0; i < indices.size(); i++)
    {
        std_msgs::Int32MultiArray indx;
        for(size_t k=0; k < indices[i].indices.size(); k++)
    	{
            indx.data.push_back(indices[i].indices[k]);
        }
    	response.clusters_indices.push_back(indx);
    }
    return true;
  }
public:
  PCLSegmenterService ()
  {
    //default values
    chop_at_z_ = 2.f;
    seg_type_ = 0;
    max_angle_plane_to_ground_ = 15.f;
    min_cluster_size_ = 500;
    sensor_noise_max_ = 0.01f;
    MAX_VERTICAL_PLANE_SIZE_ = 5000;
  }

  void
  initialize (int argc, char ** argv)
  {

    ros::init (argc, argv, "object_segmenter_service");
    n_ = new ros::NodeHandle ( "~" );
    n_->getParam ( "chop_z", chop_at_z_ );
    n_->getParam ( "seg_type", seg_type_ );
    n_->getParam ( "camera_frame", camera_frame_ );
    n_->getParam ( "base_frame", base_frame_ );
    n_->getParam ( "min_cluster_size", min_cluster_size_ );

    n_->getParam ( "sensor_noise_max", sensor_noise_max_ );

    if(!n_->getParam ( "table_range_min", table_range_min_ ))
        table_range_min_ = 0.6f;

    if(!n_->getParam ( "table_range_max", table_range_max_ ))
            table_range_max_ = 1.2f;

    n_->getParam ( "max_vertical_plane_size", MAX_VERTICAL_PLANE_SIZE_ );

    std::cout << "Started segmentation service with arguments : " << std::endl
             << "chop_z: " << chop_at_z_ << std::endl
             << "seg_type: " << seg_type_ << std::endl
             << "sensor_noise_max_: " << sensor_noise_max_ << std::endl
             << "table_range_min_: " << table_range_min_ << std::endl
             << "table_range_min_: " << table_range_min_ << std::endl
             << "table_range_max_: " << table_range_max_ << std::endl
             << "min_cluster_size: " << min_cluster_size_ << std::endl;

    segment_ = n_->advertiseService ("object_segmenter", &PCLSegmenterService::segment, this);
    std::cout << "Ready to get service calls..." << chop_at_z_ << " " << seg_type_ << std::endl;
    ros::spin ();
  }
};

int
main (int argc, char ** argv)
{
  PCLSegmenterService m;
  m.initialize (argc, argv);

  return 0;
}
