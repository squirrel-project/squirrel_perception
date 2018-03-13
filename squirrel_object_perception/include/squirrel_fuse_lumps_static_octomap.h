#ifndef FUSELUMPSOCTOMAP_H
#define FUSELUMPSOCTOMAP_H

#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include "OctomapLib.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <tf/transform_listener.h>

#include <squirrel_object_perception_msgs/CreateOctomapWithLumps.h>
#include <squirrel_object_perception_msgs/ReceiveOctomapWithLumps.h>


class FuseLumpsIntoOctomap {
private:
    ros::NodeHandle n_;
    tf::TransformListener tf_listener;

    OctomapLib octomap_lib;
    octomap::OcTree *fused_map;

    bool octomap_is_binary;

    ros::ServiceServer create_octomap_with_lumps_srv;
    ros::ServiceServer receive_octomap_with_lumps_srv;

    ros::Publisher octomap_pub;

    inline bool ends_with(std::string const & value, std::string const & ending)
    {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    };

public:
    FuseLumpsIntoOctomap (ros::NodeHandle *nodehandle);
    ~FuseLumpsIntoOctomap ();

    void initialize (int argc, char ** argv);
    bool createOctomapWithLumpsCB (squirrel_object_perception_msgs::CreateOctomapWithLumpsRequest &request, squirrel_object_perception_msgs::CreateOctomapWithLumpsResponse &response);
    bool receiveOctomapWithLumpsCB (squirrel_object_perception_msgs::ReceiveOctomapWithLumpsRequest &request, squirrel_object_perception_msgs::ReceiveOctomapWithLumpsResponse &response);
};


#endif
