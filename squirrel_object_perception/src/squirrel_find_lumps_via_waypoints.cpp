#include "squirrel_find_lumps_via_waypoints.h"


FindLumpsViaWaypointsAction::FindLumpsViaWaypointsAction(ros::NodeHandle &nh, std::string name) :
    as_(nh_, name, boost::bind(&FindLumpsViaWaypointsAction::executeCB, this, _1), false),
    action_name_(name)
{
    nh_ = nh;
    as_.start();
    success = false;


    if(nh_.getParam ( "tilt_angle", tilt_angle)) {
    } else {
        tilt_angle = 0.7; //default value if param is not set
    }
    ROS_INFO("Set camera tilt angle to %f", tilt_angle);
}

FindLumpsViaWaypointsAction::~FindLumpsViaWaypointsAction()
{
}

void FindLumpsViaWaypointsAction::executeCB(const squirrel_object_perception_msgs::FindLumpsGoalConstPtr &goal)
{
    success = false;
    feedback_.percent_completed = 0;

    //Tilt camera
    ros::ServiceClient tilt_client = nh_.serviceClient<squirrel_view_controller_msgs::LookAtPanTilt>("");
    squirrel_view_controller_msgs::LookAtPanTilt tilt_srv;
    tilt_srv.request.pan = 0.0;
    tilt_srv.request.tilt = tilt_angle;
    tilt_srv.request.reason = "look for objects";

    //Define MoveBaseClient
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_client("move_base", true);
    move_base_msgs::MoveBaseGoal mb_goal;
    while(!mb_client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Define octomap comparison service
    ros::ServiceClient find_lumps_client = nh_.serviceClient<squirrel_object_perception_msgs::FindDynamicObjects>("/squirrel_find_dynamic_objects");
    squirrel_object_perception_msgs::FindDynamicObjects find_lumps_srv;

    if (goal->return_after_first_lump) {
        if (find_lumps_client.call(find_lumps_srv)) {
            if (find_lumps_srv.response.dynamic_objects_added.size() > 0) {
                result_.lumps_found = find_lumps_srv.response.dynamic_objects_added;
                success = true;
            } else {
                success = false;
            }
        } else {
            ROS_ERROR("Failed to call service squirrel_find_dynamic_objects");
            result_.result_status = "Failed to call service squirrel_find_dynamic_objects";
            as_.setAborted(result_);
            return;
        }
    }

    //Iterate over waypoints
    if (!success) {
        for (size_t i = 0; i < goal->waypointPoses.poses.size(); i++) {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }

            //move to waypoint
            mb_goal.target_pose.header.frame_id = goal->waypointPoses.header.frame_id;
            mb_goal.target_pose.header.stamp = ros::Time::now();
            mb_goal.target_pose.pose = goal->waypointPoses.poses.at(i);
            ROS_INFO("Move to waypoint");
            mb_client.sendGoal(mb_goal);
            mb_client.waitForResult(ros::Duration(60.0));
            if (mb_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Failed to move to waypoint. Try next waypoint.");
                continue;
            }

            //call find dynamic object service
            if (goal->return_after_first_lump) {
                if (find_lumps_client.call(find_lumps_srv)) {
                    if (find_lumps_srv.response.dynamic_objects_added.size() > 0) {
                        result_.lumps_found = find_lumps_srv.response.dynamic_objects_added;
                        success = true;
                        break;
                    }
                } else {
                    ROS_ERROR("Failed to call service squirrel_find_dynamic_objects");
                    result_.result_status = "Failed to call service squirrel_find_dynamic_objects";
                    as_.setAborted(result_);
                    success = false;
                    break;
                }
            }
            feedback_.percent_completed = i/goal->waypointPoses.poses.size();
        }

        //if all lumps should be found, do octomap differencing only once in the end
        if(!success && !goal->return_after_first_lump) {
            if (find_lumps_client.call(find_lumps_srv)) {
                if (find_lumps_srv.response.dynamic_objects_added.size() > 0) {
                    result_.lumps_found = find_lumps_srv.response.dynamic_objects_added;
                }
                success = true; //action also succeedes if no lumps were found
            } else {
                ROS_ERROR("Failed to call service squirrel_find_dynamic_objects");
                result_.result_status = "Failed to call service squirrel_find_dynamic_objects";
                as_.setAborted(result_);
                success = false;
            }
        }
    }
    if(success) {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        result_.result_status = "Succeeded! Found " + boost::lexical_cast<std::string>(result_.lumps_found.size()) + " lumps";
        as_.setSucceeded(result_);
    } else {
        if(result_.lumps_found.size() == 0) {
            ROS_INFO("Did not find any lumps!");
            result_.result_status = "Did not find any lumps.";
        } else {
            result_.result_status = "Something went wrong!?";
        }
        as_.setAborted(result_);
    }
}

int main (int argc, char ** argv)
{
    ros::init (argc, argv, "squirrel_find_lumps_via_waypoints");

    ros::NodeHandle n ("~");
    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    FindLumpsViaWaypointsAction findLumps(n, ros::this_node::getName());
    ROS_INFO("%s: ready...", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
