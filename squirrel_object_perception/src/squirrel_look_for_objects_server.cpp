#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <squirrel_object_perception_msgs/GetSaliency3DSymmetry.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>
#include <squirrel_object_perception_msgs/SegmentsToObjects.h>
#include <squirrel_planning_knowledge_msgs/AddObjectService.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

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

  ros::NodeHandle nh_;
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
  int id_cnt_;

  std::string get_unique_id()
  {
      std::stringstream ss;
      ss << this->id_cnt_;
      std::string str = ss.str();
      this->id_cnt_++;
      return (std::string("object") + str);
  }

public:

  LookForObjectsAction(std::string name) :
    as_(nh_, name, boost::bind(&LookForObjectsAction::executeCB, this, _1), false),
    action_name_(name)
  {
    id_cnt_ = 1;
    as_.start();
  }

  ~LookForObjectsAction(void)
  {
  }

  bool get_saliency_map()
  {
      if (!ros::service::waitForService("/squirrel_attention_3Dsymmetry", ros::Duration(5.0)))
          return false;
      ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::GetSaliency3DSymmetry>("/squirrel_attention_3Dsymmetry");
      squirrel_object_perception_msgs::GetSaliency3DSymmetry srv;
      srv.request.cloud = *(this->scene);
      if (client.call(srv))
      {
          ROS_INFO("Called service %s: ", "/squirrel_attention_3Dsymmetry");
          this->saliency_map = srv.response.saliency_map;
          return true;
      }
      else
      {
          ROS_ERROR("Failed to call service %s", "/squirrel_attention_3Dsymmetry");
          return false;
      }
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
      if (!ros::service::waitForService("/squirrel_attention_3Dsymmetry", ros::Duration(5.0)))
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
    if (!ros::service::waitForService("/squirrel_segmentation_incremental_once", ros::Duration(5.0))||
        !ros::service::waitForService("/squirrel_segments_to_objects", ros::Duration(5.0)))
        return false;
    ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");
    ros::ServiceClient client1 = nh_.serviceClient<squirrel_object_perception_msgs::SegmentsToObjects>("/squirrel_segments_to_objects");
    squirrel_object_perception_msgs::SegmentOnce srv;
    squirrel_object_perception_msgs::SegmentsToObjects srv1;
    if (client.call(srv))
    {
        ROS_INFO("Called service %s: ", "/squirrel_attention_3Dsymmetry");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service %s", "/squirrel_attention_3Dsymmetry");
        return false;
    }
    srv1.request.cloud = *(this->scene);
    srv1.request.clusters_indices = srv.response.clusters_indices;
    if (client1.call(srv1))
    {
        ROS_INFO("Called service %s: ", "/squirrel_attention_3Dsymmetry");
        ROS_INFO("Found %ld objects", srv1.response.points.size());
        if (srv1.response.points.size() > 0)
        {
            ROS_INFO("appending object");
            Object obj;
            obj.category = "thing";
            obj.id = "0";
            obj.point_indices = srv.response.clusters_indices[0];
            obj.points = srv1.response.points[0];
            obj.pose = srv1.response.poses[0];
            this->objects.push_back(obj);
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service %s", "/squirrel_attention_3Dsymmetry");
        return false;
    }
  }

  void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
  {

    success = false;
    ROS_INFO("%s: executeCB started", action_name_.c_str());

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
    if (!get_saliency_map())
    {
        result_.result_status = "unable to get saliency map";
        as_.setAborted(result_);
        return;
    }

    if (!setup_segmentation())
    {
        result_.result_status = "unable to initialze segmentation";
        as_.setAborted(result_);
        return;
    }

    // TODO: find a reasonable number of times to run here
    for(int i=0; i<1; i++)
    {
        run_segmentation_once();
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
