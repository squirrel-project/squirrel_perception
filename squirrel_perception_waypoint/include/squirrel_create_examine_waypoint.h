#ifndef CREATEEXAMINEWAYPOINT_H
#define CREATEEXAMINEWAYPOINT_H

#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "std_srvs/Empty.h"
#include "squirrel_waypoint_msgs/ExamineWaypoint.h"
#include "squirrel_waypoint_msgs/ExamineWaypointRequest.h"
#include "squirrel_waypoint_msgs/ExamineWaypointResponse.h"
#include "squirrel_object_perception_msgs/BCylinder.h"

class CreateExamineWaypoint {

    //static const double distance_to_lump = 0.40; // min. distance in meters
    static const double robot_base_dia = 0.45;   // diameter of robot's base
    static const double robot_height = 0.8;      // robot's full height
    static const double camera_height = 0.68;    // camera mounting height
    //static const int nr_of_waypoints = 5;        /* aimed at number of waypoints around object */
    static const double hApAngle = 57*M_PI/180;  // horiz. cam aperture angle
    static const double vApAngle = 43*M_PI/180;  // vert. cam aperture angle
    static const double maxPanTilt = M_PI/(2*M_PI_2); // angle at pan/tilt = 1

    struct wpData {          // waypoint structure for output (internal only)
        octomap::point3d pos;  // position of found waypoint
        float pan;             // max camer pan
        float tiltLB;          // camera tilt (lower bound)
        float tiltUB;          //     -"-     (upper bound)
    };

private:
    ros::ServiceServer examineWaypointsServer;
    ros::NodeHandle *n_;

    ros::Publisher markerPublisher;
    ros::Publisher posePublisher;

    double distance_to_lump;
    int nr_of_waypoints;

    octomap::OcTree *oct = NULL;

public:
    CreateExamineWaypoint ();
    ~CreateExamineWaypoint ();

    void initialize (int argc, char ** argv);
    bool createExamineWaypoints(squirrel_waypoint_msgs::ExamineWaypointRequest &req, squirrel_waypoint_msgs::ExamineWaypointResponse &resp);

    void visualizeWayPoints(squirrel_waypoint_msgs::ExamineWaypointRequest &req, squirrel_waypoint_msgs::ExamineWaypointResponse &resp);

private:
    bool isBBXOccupied(octomap::OcTree *oct, octomap::point3d min, octomap::point3d max);
    bool readOctomapFromFile(std::string fileOctomap);
    int computeWaypoints(octomap::OcTree *oct, geometry_msgs::PoseStamped lump, float radius, float height, std::vector<wpData> &wp);
};

#endif
