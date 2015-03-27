/*
 * squirrel_segmentation_incremental.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include <squirrel_segmentation/squirrel_segmentation_incremental.hpp>

int SegmenterIncremental::PersistentObject::cnt = 0;

bool
SegmenterIncremental::segmentInit (squirrel_object_perception_msgs::SegmentInit::Request & req, squirrel_object_perception_msgs::SegmentInit::Response & response)
{
  //get point cloud
  scene.reset(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (req.cloud, *scene);
  //printf("Attention3DSymmetryService::calculate: point cloud is organised? %s\n", (scene->isOrganized() ? "yes" : "no")); // HACK
  // HACK: The gezebo somilated kinect seems to output a non-orgnized point cloud. Just fix that here.
  if(scene->height == 1)
  {
    if(scene->points.size() == 640*480)
    {
      scene->height = 480;
      scene->width = 640;
    }
  }
  //printf("Attention3DSymmetryService::calculate: point cloud is organised? %s\n", (scene->isOrganized() ? "yes" : "no")); // HACK
  //get saliency map
  std::vector<cv::Mat> saliencyMaps;
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req.saliency_map, sensor_msgs::image_encodings::MONO8);
  cv::Mat salMap = cv_ptr->image;
  salMap.convertTo(salMap,CV_32F,1.0/255);

//   cv::imshow("salMap",salMap);
//   cv::waitKey(-1);

  saliencyMaps.resize(1);
  salMap.copyTo(saliencyMaps.at(0));

  segmenter_->setPointCloud(scene);
  segmenter_->setSaliencyMaps(saliencyMaps);

  segmenter_->attentionSegmentInit();

  ROS_INFO("Finished initialization.");
  return true;
}

bool
SegmenterIncremental::segmentOnce (squirrel_object_perception_msgs::SegmentOnce::Request & req, squirrel_object_perception_msgs::SegmentOnce::Response & response)
{
  ROS_INFO ("Going to segment an object.");

  if(segmenter_->attentionSegmentNext())
  {
    std::vector<std::vector<int> > clusters = segmenter_->getSegmentedObjectsIndices();
    if(clusters.size() == 1)
    {
      if(checkCluster(clusters[0]))
      {
        std_msgs::Int32MultiArray indx;
        for(size_t k=0; k < clusters[0].size(); k++)
        {
          indx.data.push_back(clusters[0][k]);
        }
        response.clusters_indices.push_back(indx);
      }
    }
    else
    {
      ROS_INFO("Segmented more than one object at once, which should not happen.");
    }
  }
  else
  {
    ROS_INFO("The object that is going to be returned is emty. There can be two reasons for that: 1) the whole scene was segmented; 2) the object you are trying segment was already segmented from this view.");
  }

  return true;
}

bool SegmenterIncremental::checkCluster(std::vector<int> &cluster_indices)
{
  tf::TransformBroadcaster tfBroadcast;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (size_t i = 0; i < cluster_indices.size(); i++)
    cluster->points.push_back(scene->points[cluster_indices[i]]);

  // transform to base, as this is more convenient to conduct sanity checks
  transformCluster2base_link(cluster);

  Eigen::Vector4f centroid;
  Eigen::Matrix3f covariance_matrix;
  pcl::computeMeanAndCovarianceMatrix(*cluster, covariance_matrix, centroid);
  if(isValidCluster(cluster, centroid))   
  {
    geometry_msgs::PoseStamped inMap = base_link2map(centroid[0], centroid[1], centroid[2]);
    PersistentObject newObject(cluster, inMap.pose.position.x, inMap.pose.position.y, inMap.pose.position.z);
    std::list<PersistentObject>::iterator knownObject = knownObjects.end();
    for(std::list<PersistentObject>::iterator ot = knownObjects.begin(); ot != knownObjects.end(); ot++)
      if(ot->isSame(newObject))
        knownObject = ot;
    if(knownObject == knownObjects.end())
    {
      knownObjects.push_back(newObject);

      // visualise in rviz
      tf::Transform transform;
      tf::Vector3 p(centroid[0], centroid[1], centroid[2]);
      tf::Quaternion q(0., 0., 0., 1.);
      transform.setOrigin(p);
      transform.setRotation(q);
      tfBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", newObject.name));

      ROS_INFO("%s: found new object of size %.3f with %d points at (in base_link): (%.3f %.3f %.3f)",
        ros::this_node::getName().c_str(),
        newObject.size, (int)cluster->points.size(), newObject.pos.x, newObject.pos.y, newObject.pos.z);   
      return true;
    }
    else
    {
      ROS_INFO("%s: found object with of size %.3f with %d points at (in base_link): (%.3f %.3f %.3f), which is the same as known object of size %.3f at (%.3f %.3f %.3f)",
        ros::this_node::getName().c_str(),
        newObject.size, (int)cluster->points.size(), newObject.pos.x, newObject.pos.y, newObject.pos.z,
        knownObject->size, knownObject->pos.x, knownObject->pos.y, knownObject->pos.z);
    }
  }
  return false;
}

/**
 * Perform sanity checks to rule out stupid clusters like walls, people ..
 */
bool SegmenterIncremental::isValidCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cluster, Eigen::Vector4f &centroid)
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

  // reject objects thare are too flat (e.g. the floor)
  double z_min = 1000.;
  for(size_t i = 0; i < cloud_cluster->points.size(); i++)
    if(cloud_cluster->points[i].z < z_min)
      z_min = cloud_cluster->points[i].z;
  if(z_min < MIN_OBJECT_HEIGHT)
    return false;

  return true;
}

/**
 * Transform a cluster from kinect to base coordinates.
 * NOTE: I am not sure if this is really efficient.
 */
void SegmenterIncremental::transformCluster2base_link(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cluster)
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

geometry_msgs::PoseStamped SegmenterIncremental::kinect2base_link(double x, double y, double z)
{
  return transform(x, y, z, "/kinect_depth_optical_frame", "/base_link");
}

geometry_msgs::PoseStamped SegmenterIncremental::base_link2kinect(double x, double y, double z)
{
  return transform(x, y, z, "/base_link", "/kinect_depth_optical_frame");
}

geometry_msgs::PoseStamped SegmenterIncremental::base_link2map(double x, double y, double z)
{
  return transform(x, y, z, "/base_link", "/map");
}

geometry_msgs::PoseStamped SegmenterIncremental::transform(double x, double y, double z, const std::string &from, const std::string &to)
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

SegmenterIncremental::SegmenterIncremental ()
{
  //default values
}

SegmenterIncremental::~SegmenterIncremental ()
{
  if(n_)
    delete n_;
}

void
SegmenterIncremental::initialize (int argc, char ** argv)
{
  n_ = new ros::NodeHandle ("~");
  n_->getParam ( "model_filename", model_filename_);
  n_->getParam ( "scaling_filename", scaling_filename_);

  //model_filename_ = "/home/ekaterina/work/catkin_ws/squirrel_ros/object_perception/squirrel_segmentation/data/ST-TrainAll.txt.model";
  //scaling_filename_ = "/home/ekaterina/work/catkin_ws/squirrel_ros/object_perception/squirrel_segmentation/data/ST-TrainAll.txt.scalingparams";

  if (model_filename_.compare ("") == 0)
  {
    ROS_ERROR ("Set -model_filename option in the command line, ABORTING");
    return;
  }

  if (scaling_filename_.compare ("") == 0)
  {
    ROS_ERROR ("Set -scaling_filename option in the command line, ABORTING");
    return;
  }

  segmenter_.reset(new segmentation::Segmenter);
  segmenter_->setModelFilename(model_filename_);
  segmenter_->setScaling(scaling_filename_);

  SegmentInit_ = n_->advertiseService ("/squirrel_segmentation_incremental_init", &SegmenterIncremental::segmentInit, this);
  SegmentOnce_ = n_->advertiseService ("/squirrel_segmentation_incremental_once", &SegmenterIncremental::segmentOnce, this);
  ROS_INFO ("Ready to get service calls...");
  ros::spin ();
}


int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "squirrel_segmentation_incremental_server");
  SegmenterIncremental m;
  m.initialize (argc, argv);

  return 0;
}
