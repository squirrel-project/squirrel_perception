#ifndef FINDLUMPSVIAWAYPOINTS
#define FINDLUMPSVIAWAYPOINTS

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <vector>

#include <squirrel_object_perception_msgs/FindLumpsAction.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <squirrel_view_controller_msgs/LookAtPanTilt.h>
#include <squirrel_object_perception_msgs/FindDynamicObjects.h>


class FindLumpsViaWaypointsAction
{
protected:

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<squirrel_object_perception_msgs::FindLumpsAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    squirrel_object_perception_msgs::FindLumpsFeedback feedback_;
    squirrel_object_perception_msgs::FindLumpsResult result_;
    bool success;
    std::vector<squirrel_object_perception_msgs::SceneObject> lumps;
    double tilt_angle;
public:
    FindLumpsViaWaypointsAction(ros::NodeHandle &nh, std::string name);
    ~FindLumpsViaWaypointsAction();

    void executeCB(const squirrel_object_perception_msgs::FindLumpsGoalConstPtr &goal);
};

#endif
