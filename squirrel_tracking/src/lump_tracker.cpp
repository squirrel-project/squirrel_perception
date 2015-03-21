/**
 * This simply tracks a lump of stuff in front of the robot.
 * It segments the nearest lump from the floor and returs its centroid.
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <squirrel_tracking/lump_tracker.hpp>

using namespace std;

SquirrelTrackingNode::SquirrelTrackingNode()
{
  n_ = 0;
  startedTracking = false;
  cloud_.reset(new pcl::PointCloud<PointT>());
}

SquirrelTrackingNode::~SquirrelTrackingNode()
{
  delete n_;
}

void SquirrelTrackingNode::initialize(int argc, char ** argv)
{
  ROS_INFO("initialize");
  ROS_INFO("ros::init called");
  n_ = new ros::NodeHandle("~");
  ROS_INFO("node handle created");
  startTrackingService_ = n_->advertiseService("/squirrel_start_object_tracking", &SquirrelTrackingNode::startTracking, this);
  stopTrackingService_ = n_->advertiseService("/squirrel_stop_object_tracking", &SquirrelTrackingNode::stopTracking, this);
  ROS_INFO("Ready to get service calls...");

  ros::spin();
}

bool SquirrelTrackingNode::startTracking(squirrel_object_perception_msgs::StartObjectTracking::Request &req, squirrel_object_perception_msgs::StartObjectTracking::Response &response)
{
  bool ret = false;
  if(!startedTracking)
  {
    pointcloudSubscriber = n_->subscribe("/kinect/depth_registered/points", 1, &SquirrelTrackingNode::receivePointcloud, this);
    startedTracking = true;
    ret = true;
    ROS_INFO("SquirrelTrackingNode::startTracking: started");
  }
  else
  {
    ROS_ERROR("SquirrelTrackingNode::startTracking: I am already tracking an object, can only do one at a time.");
  }
  return ret;
}

bool SquirrelTrackingNode::stopTracking(squirrel_object_perception_msgs::StopObjectTracking::Request &req, squirrel_object_perception_msgs::StopObjectTracking::Response &response)
{
  if(startedTracking)
  {
    startedTracking = false;
    pointcloudSubscriber.shutdown();
    ROS_INFO("SquirrelTrackingNode::stopTracking: stopped");
  }
  else
  {
    ROS_ERROR("SquirrelTrackingNode::stopTracking: currently not tracking an object");
  }
  return true;
}

void SquirrelTrackingNode::receivePointcloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  //ROS_INFO("%s: new point cloud", ros::this_node::getName().c_str());

  pcl::fromROSMsg(*msg, *cloud_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud_);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);

  //ROS_INFO("%s: cloud filtered", ros::this_node::getName().c_str());

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    ROS_ERROR("%s: Failed to estimate the ground plane.", ros::this_node::getName().c_str());
    return;
  }

  //ROS_INFO("%s: cloud plane segmented", ros::this_node::getName().c_str());

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  std::cout << ros::this_node::getName() << ": PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_f);
  *cloud_filtered = *cloud_f;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  //ROS_INFO("%s: kd-tree created", ros::this_node::getName().c_str());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);

    std::cout << ros::this_node::getName() << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    j++;
  }

  //ROS_INFO("%s: found all clusters", ros::this_node::getName().c_str());

  // now select the cluster that is nearest (in base_link, i.e. min x)
  j = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr selected;
  double x_min = 1000.;
  for(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = clusters.begin(); it != clusters.end(); it++)
  {
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;
    pcl::computeMeanAndCovarianceMatrix(**it, covariance_matrix, centroid);
    geometry_msgs::PoseStamped pose_base = kinect2base_link(centroid[0], centroid[1], centroid[2]);
    // sanity check: only push things that are near enough
    if(pose_base.pose.position.x < MAX_OBJECT_DIST)
    {
      if(pose_base.pose.position.x < x_min)
      {
        x_min = centroid[2];
        selected = *it;
      }
    }
  }
  if(selected)
  {
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;
    pcl::computeMeanAndCovarianceMatrix(*selected, covariance_matrix, centroid);
    tf::Transform transform;
    tf::Vector3 p(centroid[0], centroid[1], centroid[2]);
    tf::Quaternion q(0., 0., 0., 1.);
    transform.setOrigin(p);
    transform.setRotation(q);
    tfBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect_depth_optical_frame", "lump"));

    ROS_INFO("%s: cluster %s: (%.3f %.3f %.3f)", ros::this_node::getName().c_str(), "lump",
      centroid[0], centroid[1], centroid[2]);
    /*geometry_msgs::PoseStamped pose_base = kinect2base_link(centroid[0], centroid[1], centroid[2]);
    ROS_INFO("%s: cluster in base_link %s: (%.3f %.3f %.3f)", ros::this_node::getName().c_str(), "lump",
      pose_base.pose.position.x, 
      pose_base.pose.position.y, 
      pose_base.pose.position.z);*/
  }

  //ROS_INFO("%s: done", ros::this_node::getName().c_str());
}

geometry_msgs::PoseStamped SquirrelTrackingNode::kinect2base_link(double x, double y, double z)
{
  geometry_msgs::PoseStamped Ekin, Eloc;

  Ekin.pose.position.x=x;
  Ekin.pose.position.y=y;
  Ekin.pose.position.z=z;
  Ekin.pose.orientation.x=0;
  Ekin.pose.orientation.y=0;
  Ekin.pose.orientation.z=0;
  Ekin.pose.orientation.w=1;
  Ekin.header.frame_id="/kinect_depth_optical_frame";
  try {
      tf_listener.waitForTransform("/kinect_depth_optical_frame","/base_link", ros::Time::now(), ros::Duration(0.2));
      tf_listener.transformPose("/base_link",Ekin,Eloc);
  } catch (tf::TransformException& ex) {
      std::string ns = ros::this_node::getNamespace();
      std::string node_name = ros::this_node::getName();
      ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
  }
  return Eloc;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lump_tracker");
  SquirrelTrackingNode tracker;
  tracker.initialize(argc, argv);

  return 0;
}
