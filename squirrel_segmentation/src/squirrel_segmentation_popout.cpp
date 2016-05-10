/**
 * This performs basic plane poput segmentation
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
#include <visualization_msgs/Marker.h>
#include <squirrel_segmentation/squirrel_segmentation_popout.hpp>

using namespace std;

int SegmentationPopoutNode::PersistentObject::cnt = 0;

SegmentationPopoutNode::SegmentationPopoutNode()
{
  n_ = 0;
  cloud_.reset(new pcl::PointCloud<PointT>());
}

SegmentationPopoutNode::~SegmentationPopoutNode()
{
  delete n_;
}

void SegmentationPopoutNode::initialize(int argc, char ** argv)
{
  ROS_INFO("initialize");
  ROS_INFO("ros::init called");
  n_ = new ros::NodeHandle("~");
  ROS_INFO("node handle created");
  markerPublisher = n_->advertise<visualization_msgs::Marker>("visualization_marker", 0);
  //segmentService_ = n_->advertiseService("/squirrel_segmentation", &SegmentationPopoutNode::segment, this);
  SegmentInit_ = n_->advertiseService ("/squirrel_segmentation_incremental_init", &SegmentationPopoutNode::segment, this);
  SegmentOnce_ = n_->advertiseService ("/squirrel_segmentation_incremental_once", &SegmentationPopoutNode::returnNextResult, this);
  ROS_INFO("Ready to get service calls...");

  ros::spin();
}

bool SegmentationPopoutNode::segment(squirrel_object_perception_msgs::SegmentInit::Request & req, squirrel_object_perception_msgs::SegmentInit::Response & response)
{
  bool ret = false;

  ROS_INFO("%s: new point cloud", ros::this_node::getName().c_str());

  // clear results for a new segmentation run
  results.clear();

  pcl::PointCloud<PointT>::Ptr inCloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg (req.cloud, *inCloud);
  cloud_ = inCloud->makeShared();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

//  // Create the filtering object: downsample the dataset using a leaf size of 1cm
//  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//  vg.setInputCloud (cloud_);
//  vg.setLeafSize (0.01f, 0.01f, 0.01f);
//  vg.filter (*cloud_filtered);
  cloud_filtered->resize(cloud_->size());
  for (size_t i = 0; i < cloud_->size(); ++i)
  {
     cloud_filtered->points[i].x = cloud_->points[i].x;
     cloud_filtered->points[i].y = cloud_->points[i].y;
     cloud_filtered->points[i].z = cloud_->points[i].z;
  }

  ROS_INFO("%s: cloud filtered", ros::this_node::getName().c_str());

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
  {
    ROS_ERROR("%s: Failed to estimate the ground plane.", ros::this_node::getName().c_str());
    return ret;
  }

  ROS_INFO("%s: cloud plane segmented", ros::this_node::getName().c_str());

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

  ROS_INFO("%s: kd-tree created", ros::this_node::getName().c_str());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
    std::cout << ros::this_node::getName() << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
  }

  ROS_INFO("%s: found all clusters", ros::this_node::getName().c_str());

  // transform to base, as this is more convenient to conduct sanity checks
  for(size_t i = 0; i < clusters.size(); i++)
    tranformCluster2base_link(clusters[i]);

  for(size_t i = 0; i < clusters.size(); i++)
  {
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance_matrix;
    pcl::computeMeanAndCovarianceMatrix(*clusters[i], covariance_matrix, centroid);
    if(isValidCluster(clusters[i], centroid))   
    {
      geometry_msgs::PoseStamped inMap = base_link2map(centroid[0], centroid[1], centroid[2]);
      PersistentObject newObject(clusters[i], inMap.pose.position.x, inMap.pose.position.y, inMap.pose.position.z);
      std::list<PersistentObject>::iterator knownObject = knownObjects.end();
      for(std::list<PersistentObject>::iterator ot = knownObjects.begin(); ot != knownObjects.end(); ot++)
        if(ot->isSame(newObject))
          knownObject = ot;
      if(knownObject == knownObjects.end())
      {
        knownObjects.push_back(newObject);
        visualizePersistentObject(newObject);
        ROS_INFO("%s: found new object of size %.3f with %d points at (in map): (%.3f %.3f %.3f)",
          ros::this_node::getName().c_str(),
          newObject.size, (int)clusters[i]->points.size(), newObject.pos.x, newObject.pos.y, newObject.pos.z);   
        
        // save the valid results of the current segmentation, to be returned incrementally lalter
        results.push_back(SegmentationResult());
        for(size_t k = 0; k < cluster_indices[i].indices.size(); k++)
          results.back().indices.data.push_back(cluster_indices[i].indices[k]);
        results.back().distanceFromRobot = centroid[0];
	results.back().pose = inMap;
	pcl::toROSMsg(*clusters[i], results.back().points);
      }
      else
      {
        ROS_INFO("%s: found object with of size %.3f with %d points at (in base_link): (%.3f %.3f %.3f), which is the same as known object of size %.3f at (%.3f %.3f %.3f)",
          ros::this_node::getName().c_str(),
          newObject.size, (int)clusters[i]->points.size(), newObject.pos.x, newObject.pos.y, newObject.pos.z,
          knownObject->size, knownObject->pos.x, knownObject->pos.y, knownObject->pos.z);
      }
    }
  }

  ROS_INFO("%s: done", ros::this_node::getName().c_str());

  ret = true;

  return ret;
}

bool SegmentationPopoutNode::returnNextResult(squirrel_object_perception_msgs::SegmentOnce::Request & req, squirrel_object_perception_msgs::SegmentOnce::Response & response)
{
  double x_min = 1000.;
  size_t selected = results.size();
  // return the nearest object not returned yet
  for(size_t i = 0; i < results.size(); i++)
  {
    if(!results[i].alreadyReturned)
    {
      if(results[i].distanceFromRobot < x_min)
      {
        x_min = results[i].distanceFromRobot;
        selected = i;
      }
    }
  }
  if(selected < results.size())
  {
    ROS_INFO("%s: returning cluster with %d points", ros::this_node::getName().c_str(), (int)results[selected].indices.data.size());
    results[selected].alreadyReturned = true;
    response.clusters_indices.push_back(results[selected].indices);
    response.poses.push_back(results[selected].pose);
    response.points.push_back(results[selected].points);
    return true;
  }
  return false;
}

void SegmentationPopoutNode::visualizePersistentObject(PersistentObject &obj)
{
  static int cnt = 0;

  // draw the bounding sphere
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = obj.name;
  marker.id = cnt;
  marker.lifetime = ros::Duration();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = obj.pos.x;
  marker.pose.position.y = obj.pos.y;
  marker.pose.position.z = obj.pos.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = obj.size;
  marker.scale.y = obj.size;
  marker.scale.z = obj.size;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5; // Don't forget to set the alpha!
  markerPublisher.publish(marker);


  visualization_msgs::Marker label;
  label.header.frame_id = "map";
  label.header.stamp = ros::Time();
  std::stringstream ss;
  ss << obj.name << "_label";
  label.ns = ss.str();
  label.id = cnt;
  label.lifetime = ros::Duration();
  label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::Marker::ADD;
  label.pose.position.x = obj.pos.x;
  label.pose.position.y = obj.pos.y;
  label.pose.position.z = obj.pos.z + 0.5;
  label.pose.orientation.x = 0.0;
  label.pose.orientation.y = 0.0;
  label.pose.orientation.z = 0.0;
  label.pose.orientation.w = 1.0;
  label.scale.x = 0;
  label.scale.y = 0;
  label.scale.z = 0.2;
  label.color.r = 0.0;
  label.color.g = 1.0;
  label.color.b = 0.0;
  label.color.a = 1; // Don't forget to set the alpha!
  label.text = obj.name;
  markerPublisher.publish(label);

  // and a transform
  tf::Transform transform;
  tf::Vector3 p(obj.pos.x, obj.pos.y, obj.pos.z);
  tf::Quaternion q(0., 0., 0., 1.);
  transform.setOrigin(p);
  transform.setRotation(q);
  tfBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", obj.name));

  cnt++;
}

geometry_msgs::PoseStamped SegmentationPopoutNode::kinect2base_link(double x, double y, double z)
{
  return transform(x, y, z, "/kinect_depth_optical_frame", "/base_link");
}

geometry_msgs::PoseStamped SegmentationPopoutNode::base_link2kinect(double x, double y, double z)
{
  return transform(x, y, z, "/base_link", "/kinect_depth_optical_frame");
}

geometry_msgs::PoseStamped SegmentationPopoutNode::base_link2map(double x, double y, double z)
{
  return transform(x, y, z, "/base_link", "/map");
}

geometry_msgs::PoseStamped SegmentationPopoutNode::transform(double x, double y, double z, const std::string &from, const std::string &to)
{
  geometry_msgs::PoseStamped before, after;

  before.pose.position.x = x;
  before.pose.position.y = y;
  before.pose.position.z = z;
  before.pose.orientation.x = 0;
  before.pose.orientation.y = 0;
  before.pose.orientation.z = 0;
  before.pose.orientation.w = 1;
  before.header.frame_id = from;
  try
  {
    tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(0.2));
    tf_listener.transformPose(to, before, after);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
  }
  return after;
}

/**
 * Transform a cluster from kinect to base coordinates.
 * NOTE: I am not sure if this is really efficient.
 */
void SegmentationPopoutNode::tranformCluster2base_link(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster)
{
  try
  {
    tf_listener.waitForTransform("/kinect_depth_optical_frame","/base_link", ros::Time::now(), ros::Duration(0.2));
    for(size_t i = 0; i < cloud_cluster->points.size(); i++)
    {
      geometry_msgs::PointStamped p, pb;
      p.point.x = cloud_cluster->points[i].x;
      p.point.y = cloud_cluster->points[i].y;
      p.point.z = cloud_cluster->points[i].z;
      p.header.frame_id = "/kinect_depth_optical_frame";
      tf_listener.transformPoint("/base_link", p, pb);
      cloud_cluster->points[i].x = pb.point.x;
      cloud_cluster->points[i].y = pb.point.y;
      cloud_cluster->points[i].z = pb.point.z;
    }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
  }
}

/**
 * Perform sanity checks to rule out stupid clusters like walls, people ..
 */
bool SegmentationPopoutNode::isValidCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster, Eigen::Vector4f &centroid)
{
  // reject objects too far away (e.g. the wall when looking straight)
  // NOTE: cluster is in base_link frame, with x pointing forward, z up
  if(centroid[0] > MAX_OBJECT_DIST)
    return false;

  // reject objects thare are too tall, e.g. people, walls
  double z_max = 0.;
  for(size_t i = 0; i < cloud_cluster->points.size(); i++)
    if(cloud_cluster->points[i].z > z_max)
      z_max = cloud_cluster->points[i].z;
  if(z_max > MAX_OBJECT_HEIGHT)
    return false;

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_popout");
  SegmentationPopoutNode seg;
  seg.initialize(argc, argv);

  return 0;
}
