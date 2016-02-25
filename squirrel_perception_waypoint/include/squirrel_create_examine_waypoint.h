#ifndef CREATEEXAMINEWAYPOINT_H
#define CREATEEXAMINEWAYPOINT_H

#include "ros/ros.h"
#include <sstream>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>

#include "std_srvs/Empty.h"
#include "squirrel_waypoint_msgs/ExamineWaypoint.h"
#include "squirrel_waypoint_msgs/ExamineWaypointRequest.h"
#include "squirrel_waypoint_msgs/ExamineWaypointResponse.h"
#include "squirrel_object_perception_msgs/BCylinder.h"

class CreateExamineWaypoint {

    static const double distance_to_lump = 0.30; //in meters
    static const int nr_of_waypoints = 5;

private:
    ros::ServiceServer examineWaypointsServer;
    ros::NodeHandle *n_;

    ros::Publisher markerPublisher;
    ros::Publisher posePublisher;

public:
    CreateExamineWaypoint ();
    ~CreateExamineWaypoint ();

    void initialize (int argc, char ** argv);
    bool createExamineWaypoints(squirrel_waypoint_msgs::ExamineWaypointRequest &req, squirrel_waypoint_msgs::ExamineWaypointResponse &resp);

    void visualizeWayPoints(squirrel_waypoint_msgs::ExamineWaypointRequest &req, squirrel_waypoint_msgs::ExamineWaypointResponse &resp);
};

#endif
