#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>

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
  sensor_msgs::PointCloud2ConstPtr scene;
  bool success;

public:

  LookForObjectsAction(std::string name) :
    as_(nh_, name, boost::bind(&LookForObjectsAction::executeCB, this, _1), false),
    action_name_(name)
  {
    ROS_INFO("%s: in constructed", ros::this_node::getName().c_str());
    as_.start();
  }

  ~LookForObjectsAction(void)
  {
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

    if(success)
    {
      //result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else
    {
        result_.result_status = "Empty";
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
