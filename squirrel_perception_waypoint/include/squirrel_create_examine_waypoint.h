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
#include "squirrel_tests/ExamineWaypoint.h"
#include "squirrel_tests/ExamineWaypointRequest.h"
#include "squirrel_tests/ExamineWaypointResponse.h"

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
    bool createExamineWaypoints(squirrel_tests::ExamineWaypointRequest &req, squirrel_tests::ExamineWaypointResponse &resp);

    void visualizeWayPoints(geometry_msgs::PoseStamped lump_pose, squirrel_tests::ExamineWaypointResponse &resp);
};

#endif
