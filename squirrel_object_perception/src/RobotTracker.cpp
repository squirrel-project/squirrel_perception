#include "RobotTracker.h"

RobotTracker::RobotTracker()
{
    n_ = 0;
    startedTracking = false;
}

RobotTracker::~RobotTracker()
{
    delete n_;
}

void RobotTracker::initialize(int argc, char ** argv)
{
    n_ = new ros::NodeHandle("~");
    startTrackingService_ = n_->advertiseService("/squirrel_start_robot_tracking", &RobotTracker::startTracking, this);
    stopTrackingService_ = n_->advertiseService("/squirrel_stop_robot_tracking", &RobotTracker::stopTracking, this);

    ROS_INFO("RobotTracker: Ready to receive service calls.");

    period_ = ros::Duration(1.0); //listen to the topic every second
    last_pub_ = ros::Time::now();

    ros::spin();
}

bool RobotTracker::startTracking(squirrel_object_perception_msgs::StartRobotTracking::Request &req, squirrel_object_perception_msgs::StartRobotTracking::Response &response)
{
    if (startedTracking) {
        ROS_ERROR("I'm already tracking the robot");
        return false;
    } else {
        ROS_INFO("Started tracking robot");
        startedTracking = true;
        robotPositions.clear();
        pose_sub = n_->subscribe("/squirrel_2d_localizer/pose", 1, &RobotTracker::robotTrackingCb, this);
        return true;
    }
}

bool RobotTracker::stopTracking(squirrel_object_perception_msgs::StopRobotTracking::Request &req, squirrel_object_perception_msgs::StopRobotTracking::Response &response)
{
    if (startedTracking) {
        startedTracking = false;
        pose_sub.shutdown();
        response.positions = robotPositions;
        std::cout <<  "Number of tracked positions: " << robotPositions.size() << std::endl;
        ROS_INFO("Stopped tracking robot!");
        return true;
    } else {
        ROS_ERROR("No robot tracking instance to stop");
        return false;
    }
}

void RobotTracker::robotTrackingCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
    ros::Time cur = ros::Time::now();
    ros::Duration elapsed = cur - last_pub_;
    ros::Duration remaining = period_ - elapsed;
    if (remaining.toSec() > 0)
    {
        remaining.sleep();
        return;
    }

    geometry_msgs::Point p;
    p.x = pose->pose.pose.position.x;
    p.y = pose->pose.pose.position.y;
    p.z = pose->pose.pose.position.z;

    robotPositions.push_back(p);
    std::cout << "x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
    last_pub_ = cur;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tracker");
    RobotTracker tracker;
    tracker.initialize(argc, argv);

    return 0;
}
