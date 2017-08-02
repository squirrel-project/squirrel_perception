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
#include <fstream>
#include <boost/unordered_set.hpp>

class OctomapLib {
public:
    typedef pcl::PointXYZRGB PointT;
    static const int tree_depth = 16;
    double leaf_size;
    octomap::KeySet static_keys;
    tf::TransformListener tf_listener;

    bool readOctoMapFromFile(std::string, octomap::OcTree *&ocTree, bool binary);
    void tranformCloud2Map(pcl::PointCloud<PointT>::Ptr &cloud);
    void checkCloudAgainstOctomap(const pcl::PointCloud<PointT>::Ptr &cloud, octomap::OcTree *ocTree);
    octomap::OcTree subtractOctomap(const octomap::OcTree *minuendMap, octomap::OcTree subtrahendMap);
    void writeOctomap(octomap::OcTree ocTree, std::string path, bool isBinary);
    void octomapToPointcloud(octomap::OcTree *octomap, pcl::PointCloud<PointT>::Ptr &cloud);
    void getOctomapDimension(octomap::OcTree *octomap, unsigned int &width, unsigned int &height, unsigned int &depth);
    void octomapToMat(octomap::OcTree *octomap, cv::Mat &mat);
    void expandOccupiedNodes(octomap::OcTree *octomap);
    int getNumberOccupiedLeafNodes(const octomap::OcTree *octomap);
    octomap::OcTree dilateOctomap(octomap::OcTree *octomap);
    void fillFloor(octomap::OcTree *octomap, octomap::point3d min, octomap::point3d max);
    void initStaticKeys(octomap::OcTree *staticMap);
    void addNodes(octomap::OcTree *ocTree);
    void printMapInfo(octomap::OcTree *ocTree);
    octomap::OcTree compareOctomapToStatic(const octomap::OcTree *staticOctomap, octomap::OcTree currentOctomap);

    ~OctomapLib();
private:
    void expandNodeRecurs(octomap::OcTreeNode* node, unsigned int depth, unsigned int max_depth);

};

#endif // OCTOMAPLIB_H
