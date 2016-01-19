#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <squirrel_object_perception_msgs/Classification.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/GetSaliency3DSymmetry.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationInit.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationOnce.h>
#include <squirrel_object_perception_msgs/Recognize.h>
#include <squirrel_planning_knowledge_msgs/UpdateObjectService.h>
#include <squirrel_planning_knowledge_msgs/AddObjectService.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <algorithm>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

class Object
{

public:
    std::string id;
    std::string category;
    geometry_msgs::PoseStamped pose;
    sensor_msgs::PointCloud2 points;
    std_msgs::Int32MultiArray point_indices;
};

class LookForObjectsAction
{
protected:

  typedef pcl::PointXYZRGB PointT;

  ros::NodeHandle nh_;
  tf::TransformListener tf_listener;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<squirrel_object_perception_msgs::LookForObjectsAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  squirrel_object_perception_msgs::LookForObjectsFeedback feedback_;
  squirrel_object_perception_msgs::LookForObjectsResult result_;
  // create needed variables
  sensor_msgs::PointCloud2ConstPtr scene;
  bool success;
  sensor_msgs::Image saliency_map;
  std::vector<Object> objects;
  std::vector<Object>::iterator objectIterator;
  // just for now
  std::vector<squirrel_object_perception_msgs::RecognizeResponse> recognized_object;
  std::vector<std_msgs::Int32MultiArray> cluster_indices;
  int id_cnt_;

  void set_publish_feedback(std::string phase, std::string status, int percent)
  {
      this->feedback_.current_phase = phase;
      this->feedback_.current_status = status;
      this->feedback_.percent_completed = percent;
      this->as_.publishFeedback(this->feedback_);
      return;
  }

  std::string get_unique_object_id()
  {
      std::stringstream ss;
      ss << this->id_cnt_;
      std::string str = ss.str();
      this->id_cnt_++;
      return (std::string("object") + str);
  }

  bool do_recognition()
  {
      if (!ros::service::waitForService("/mp_recognition", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Recognize>("/mp_recognition");
      squirrel_object_perception_msgs::Recognize srv;
      srv.request.cloud = *(this->scene);
      if (client.call(srv))
      {
          ROS_INFO("Called service %s: ", "/mp_recognition");
          this->recognized_object.push_back(srv.response);
          return true;
      }
      else
      {
          return false;
      }
  }

  bool setup_visualization()
  {
      if (!ros::service::waitForService("/squirrel_segmentation_visualization_init", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentVisualizationInit>("/squirrel_segmentation_visualization_init");
      squirrel_object_perception_msgs::SegmentVisualizationInit srv;
      srv.request.cloud = *(this->scene);
      srv.request.saliency_map = this->saliency_map;
      if (client.call(srv))
      {
          ROS_INFO("Called service %s: ", "/squirrel_segmentation_visualization_init");
          return true;
      }
      else
      {
          return false;
      }
  }

  bool run_visualization_once()
  {
      if (!ros::service::waitForService("/squirrel_segmentation_visualization_once", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentVisualizationOnce>("/squirrel_segmentation_visualization_once");
      squirrel_object_perception_msgs::SegmentVisualizationOnce srv;
      srv.request.clusters_indices = this->cluster_indices;
      if (client.call(srv))
      {
          ROS_INFO("Called service %s: ", "/squirrel_segmentation_visualization_once");
          return true;
      }
      else
      {
          return false;
      }
  }

  bool get_saliency_map()
  {
      if (!ros::service::waitForService("/squirrel_attention_itti", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::GetSaliency3DSymmetry>("/squirrel_attention_itti");
      squirrel_object_perception_msgs::GetSaliency3DSymmetry srv;
      srv.request.cloud = *(this->scene);
      if (client.call(srv))
      {
          ROS_INFO("Called service %s: ", "/squirrel_attention_itti");
          this->saliency_map = srv.response.saliency_map;
          return true;
      }
      else
      {
          ROS_ERROR("Failed to call service %s", "/squirrel_attention_itti");
          return false;
      }
  }

  geometry_msgs::PoseStamped transform(double x, double y, double z, const std::string &from, const std::string &to)
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

  bool segments_to_objects(sensor_msgs::PointCloud2 cloud, std::vector<std_msgs::Int32MultiArray> clusters_indices)
  {
    std::vector<geometry_msgs::PoseStamped> poses;
    std::vector<sensor_msgs::PointCloud2> points;
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(cloud, *scene);
    points.resize(clusters_indices.size());
    poses.resize(clusters_indices.size());
    for(size_t i = 0; i < clusters_indices.size(); i++)
    {
      pcl::PointCloud<PointT>::Ptr object(new pcl::PointCloud<PointT>);
      for(size_t j = 0; j < clusters_indices[i].data.size(); j++)
      {
        object->points.push_back(scene->points[clusters_indices[i].data[j]]);
      }
      pcl::toROSMsg(*object, points[i]);

      Eigen::Vector4d centroid;
      pcl::compute3DCentroid(*scene, clusters_indices[i].data, centroid);
      poses[i].header.stamp = cloud.header.stamp;
      poses[i].header.frame_id = "/map";
      // note: the orientation is quite arbitrary
      poses[i] = transform(centroid[0], centroid[1], centroid[2], "/kinect_depth_optical_frame", "/map");
      printf("%s: new object for the scene database at position (/map): %.3f %.3f %.3f\n", ros::this_node::getName().c_str(),
        centroid[0], centroid[1], centroid[2]);
    }

    return true;
  }

  bool add_object_to_db(Object object)
  {
      if (!ros::service::waitForService("/kcl_rosplan/add_object", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_planning_knowledge_msgs::AddObjectService>("/kcl_rosplan/add_object");
      squirrel_planning_knowledge_msgs::AddObjectService srv;
      srv.request.id = object.id;
      srv.request.category = object.category;
      srv.request.pose = object.pose;
      srv.request.cloud = object.points;
      if (client.call(srv))
      {
          ROS_INFO("Called service %s: ", "/kcl_rosplan/add_object");
          return true;
      }
      else
      {
          ROS_ERROR("Failed to call service %s", "/kcl_rosplan/add_object");
          return false;
      }
  }

  bool setup_segmentation()
  {
      if (!ros::service::waitForService("/squirrel_segmentation_incremental_init", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentInit>("/squirrel_segmentation_incremental_init");
      squirrel_object_perception_msgs::SegmentInit srv;
      srv.request.cloud = *(this->scene);
      srv.request.saliency_map = this->saliency_map;
      if (client.call(srv))
      {
          ROS_INFO("Called service %s: ", "/squirrel_segmentation_incremental_init");
          return true;
      }
      else
      {
          ROS_ERROR("Failed to call service %s", "/squirrel_segmentation_incremental_init");
          return false;
      }
  }

  bool run_segmentation_once()
  {
    if (!ros::service::waitForService("/squirrel_segmentation_incremental_once", ros::Duration(5.0)))
        return false;
    ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");
    
    squirrel_object_perception_msgs::SegmentOnce srv;
    
    if (client.call(srv))
    {
        ROS_INFO("Called service %s: ", "/squirrel_segmentation_incremental_once");
    }
    else
    {
        ROS_ERROR("Failed to call service %s", "/squirrel_segmentation_incremental_once");
        return false;
    }

    this->cluster_indices = srv.response.clusters_indices;

    for(int i=0; srv.response.poses.size(); i++) {
        ROS_INFO("appending object");
        Object obj;
        obj.category = "thing";
        obj.id = get_unique_object_id();
        obj.point_indices = srv.response.clusters_indices[i];
        obj.points = srv.response.points[i];
        obj.pose = srv.response.poses[i];
        this->objects.push_back(obj);
        return true;
    }
  }

  bool update_object_in_db(Object object)
  {
      if (!ros::service::waitForService("/kcl_rosplan/update_object", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_planning_knowledge_msgs::UpdateObjectService>("/kcl_rosplan/update_object");
      squirrel_planning_knowledge_msgs::UpdateObjectService srv;
      srv.request.id = object.id;
      srv.request.category = object.category;
      srv.request.pose = object.pose;
      srv.request.cloud = object.points;
      if (client.call(srv))
      {
          return true;
      }
      else
      {
          return false;
      }
  }

  std::string most_confident_class(squirrel_object_perception_msgs::Classification classification)
  {
     std::vector<float>::iterator max;
     max = std::max_element(classification.confidence.begin(), classification.confidence.end());
     int max_index = std::distance(classification.confidence.begin(), max);
     return classification.class_type[max_index].data;
  }

  bool run_classifier()
  {
      if (!ros::service::waitForService("/squirrel_classify", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Classify>("/squirrel_classify");
      squirrel_object_perception_msgs::Classify srv;
      // TODO: check data
      std::vector<std_msgs::Int32MultiArray> point_indices;
      point_indices.push_back(this->objects.back().point_indices);
      srv.request.cloud = *(this->scene);
      srv.request.clusters_indices = point_indices;
      if (client.call(srv))
      {
          if (srv.response.class_results.size() == 1)
          {
              this->objects.back().category = this->most_confident_class(srv.response.class_results[0]);
              return true;
          }
          else
          {
            return false;
          }
      }
      else
      {
          return false;
      }
  }

public:

  LookForObjectsAction(std::string name) :
    as_(nh_, name, boost::bind(&LookForObjectsAction::executeCB, this, _1), false),
    action_name_(name)
  {
    id_cnt_ = 1;
    as_.start();
    success = false;
  }

  ~LookForObjectsAction(void)
  {
  }

  void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
  {

    ROS_INFO("%s: executeCB started", action_name_.c_str());

    sleep(2); // HACK: Michael Zillich

    if (as_.isPreemptRequested())
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted(result_);
    }

    // get data from depth camera
    scene = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_, ros::Duration(5));
    if (scene)
    {
        ROS_DEBUG("%s: Received data", action_name_.c_str());
    }
    else
    {
        ROS_INFO("squirrel_object_perception: Did not receive any data from the camera");
        result_.result_status = "Unable to get data from the camera.";
        as_.setAborted(result_);
        success = false;
        return;
    }

    //TODO this is neede later, but not for popout_segmentation
    /*if (!get_saliency_map())
    {
        result_.result_status = "unable to get saliency map";
        as_.setAborted(result_);
        return;
    }*/

    if (!setup_segmentation())
    {
        result_.result_status = "unable to initialze segmentation";
        as_.setAborted(result_);
        return;
    }

    //TODO needed later, not supported by popout_segmentation
    /*if (!setup_visualization())
    {
        result_.result_status = "unable to initialze visualization";
        as_.setAborted(result_);
        return;
    }*/


    // TODO: find a reasonable number of times to run here
    for(int i=0; i<1; i++)
    {
        run_segmentation_once();
        //run_visualization_once();
    }
    if (objects.size() < 1)
    {
        result_.result_status = "No objects classified";
        as_.setAborted(result_);
        return;
    }
    for(objectIterator = objects.begin(); objectIterator != objects.end(); objectIterator++)
    {
        success = add_object_to_db(*objectIterator);
        if (!success)
            break;
    }

    if(success)
    {
      //result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else
    {
        result_.result_status = "Some error has occured.";
        as_.setAborted(result_);
    }

  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "look_for_objects");
  ROS_INFO("%s: started node", ros::this_node::getName().c_str());

  LookForObjectsAction lookforobjects(ros::this_node::getName());
  ros::spin();

  return 0;
}
