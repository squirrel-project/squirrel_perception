#ifndef POSTPROCESSOCTOMAP_H
#define POSTPROCESSOCTOMAP_H

#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include "OctomapLib.h"
#include <iostream>
#include <fstream>


class PostProcessOctomap {
private:
    ros::NodeHandle n_;

    octomap::OcTree *staticMap;

    OctomapLib octomap_lib;

    inline bool ends_with(std::string const & value, std::string const & ending)
    {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    };

public:
    PostProcessOctomap (ros::NodeHandle *nodehandle);
    ~PostProcessOctomap ();

    void initialize (int argc, char ** argv);
};


#endif
