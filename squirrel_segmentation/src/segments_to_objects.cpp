/**
 * segments_to_objects.cpp
 * 
 * Takes a list of segmented clusters and returns "proto" objects: 3D poses and 3D points.
 *
 * @author: Michael Zillich
 * @date 2015-03-04
 */

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <squirrel_object_perception_msgs/SegmentsToObjects.h>

class SegmentsToObjectsImpl
{
private:
  typedef pcl::PointXYZRGB PointT;

  ros::ServiceServer service_;
  ros::NodeHandle *nh_;
  tf::TransformListener tf_listener;

  bool segmentsToObjects(squirrel_object_perception_msgs::SegmentsToObjects::Request & req, squirrel_object_perception_msgs::SegmentsToObjects::Response & response);
  geometry_msgs::PoseStamped transform(double x, double y, double z, const std::string &from, const std::string &to);

public:
  SegmentsToObjectsImpl();
  ~SegmentsToObjectsImpl();
  void init(int argc, char ** argv);
};

SegmentsToObjectsImpl::SegmentsToObjectsImpl()
{
  nh_ = 0;
}

SegmentsToObjectsImpl::~SegmentsToObjectsImpl()
{
  delete nh_;
}

geometry_msgs::PoseStamped SegmentsToObjectsImpl::transform(double x, double y, double z, const std::string &from, const std::string &to)
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
    tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
    tf_listener.transformPose(to, before, after);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
  }
  return after;
}

bool SegmentsToObjectsImpl::segmentsToObjects(squirrel_object_perception_msgs::SegmentsToObjects::Request &req,
                                              squirrel_object_perception_msgs::SegmentsToObjects::Response &response)
{
  printf ("SegmentsToObjectsImpl::segmentsToObjects called\n");
  pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(req.cloud, *scene);
  response.points.resize(req.clusters_indices.size());
  response.poses.resize(req.clusters_indices.size());
  for(size_t i = 0; i < req.clusters_indices.size(); i++)
  {
    pcl::PointCloud<PointT>::Ptr object(new pcl::PointCloud<PointT>);
    for(size_t j = 0; j < req.clusters_indices[i].data.size(); j++)
    {
      object->points.push_back(scene->points[req.clusters_indices[i].data[j]]);
    }
    pcl::toROSMsg(*object, response.points[i]);

    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*scene, req.clusters_indices[i].data, centroid);
    response.poses[i].header.stamp = req.cloud.header.stamp;
    response.poses[i].header.frame_id = "/map";
    // note: the orientation is quite arbitrary
    response.poses[i] = transform(centroid[0], centroid[1], centroid[2], "/kinect_depth_optical_frame", "/map");
    printf("%s: new object for the scene database at position (/map): %.3f %.3f %.3f\n", ros::this_node::getName().c_str(),
      centroid[0], centroid[1], centroid[2]);
      //response.poses[i].pose.position.x,
      //response.poses[i].pose.position.y,
      //response.poses[i].pose.position.z);
  }
  
  return true;
}

void SegmentsToObjectsImpl::init(int argc, char ** argv)
{
  nh_ =  new ros::NodeHandle ("~");
  service_ = nh_->advertiseService ("/squirrel_segments_to_objects", &SegmentsToObjectsImpl::segmentsToObjects, this);
  ROS_INFO ("Ready to receive service calls...");
}

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "segments_to_objects");
  SegmentsToObjectsImpl s;
  s.init(argc, argv);
  ros::spin();
  return 0;
}
