#ifndef OCTOMAPLIB_H
#define OCTOMAPLIB_H

#include <string>
#include <octomap/OcTree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <cv.h>
#include <cstdlib>
#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

class OctomapLib {
public:
    typedef pcl::PointXYZRGB PointT;
    static const int tree_depth = 16;

    bool readOctoMapFromFile(std::string, octomap::OcTree *&ocTree, bool binary);
    void tranformCloud2Map(pcl::PointCloud<PointT>::Ptr &cloud);
    void checkCloudAgainstOctomap(const pcl::PointCloud<PointT>::Ptr &cloud, octomap::OcTree *ocTree);
    octomap::OcTree subtractOctomap(const octomap::OcTree *minuendMap, octomap::OcTree subtrahendMap);
    void writeOctomap(octomap::OcTree *ocTree, std::string path, bool isBinary);
    void octomapToPointcloud(octomap::OcTree *octomap, pcl::PointCloud<PointT>::Ptr &cloud);
    void getOctomapDimension(octomap::OcTree *octomap, unsigned int &width, unsigned int &height, unsigned int &depth);
    void octomapToMat(octomap::OcTree *octomap, cv::Mat &mat);
    void octomapExpandOccupiedNodes(octomap::OcTree *octomap);

    void addNodes(octomap::OcTree *ocTree);

    void printMapInfo(octomap::OcTree *ocTree);


    ~OctomapLib();
};

#endif // OCTOMAPLIB_H
