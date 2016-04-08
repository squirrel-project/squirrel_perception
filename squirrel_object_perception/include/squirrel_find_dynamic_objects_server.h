#ifndef SQUIRRELREMOVEBACKGROUND_H
#define SQUIRRELREMOVEBACKGROUND_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"

#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/time.h>
#include <octomap/OcTree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <nav_msgs/GetMap.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include "OctomapLib.h"
#include "squirrel_object_perception_msgs/SceneObject.h"
#include "squirrel_object_perception_msgs/FindDynamicObjects.h"
#include <boost/foreach.hpp>
#include "mongodb_store/message_store.h"

typedef pcl::PointXYZRGB PointT;
const double POSE_THRESH = 0.10; //5 cm

class RemoveBackground {
private:

    ros::ServiceServer Remover_;
    ros::NodeHandle *n_;
    ros::Publisher markerPublisher;

    octomap::OcTree *staticMap;
    octomap::OcTree *currentMap;

    OctomapLib octomap_lib;

    std::string staticOctomapPath_;

    bool removeBackground (squirrel_object_perception_msgs::FindDynamicObjects::Request &req, squirrel_object_perception_msgs::FindDynamicObjects::Response & response);

    mongodb_store::MessageStoreProxy message_store;

    visualization_msgs::Marker zyl_marker;

    int id_cnt_;

    std::vector<int32_t> vis_marker_ids;

    inline bool ends_with(std::string const & value, std::string const & ending)
    {
        if (ending.size() > value.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    };

public:
    RemoveBackground ();
    ~RemoveBackground ();

    void initialize (int argc, char ** argv);
    std::string get_unique_object_id();
    void setStaticOctomap(std::string staticPath);
    void setCurrentOctomap(octomap::OcTree *currentMap);
    void init();
    void filterCloudNoise(pcl::PointCloud<PointT>::Ptr &cloud);
    void mapToMat(const nav_msgs::OccupancyGridConstPtr& grid_map, cv::Mat &mat);
    void compareCloudToMap(pcl::PointCloud<PointT>::Ptr &cloud, const nav_msgs::OccupancyGridConstPtr& grid_map);
    std::vector<pcl::PointCloud<PointT>::Ptr> removeClusters(pcl::PointCloud<PointT>::Ptr &cloud);
    octomap::OcTree subtractOctomaps();
    pcl::PointCloud<PointT>::Ptr compareOctomapToGrid(octomap::OcTree *octomap, const nav_msgs::OccupancyGridConstPtr& grid_map);
};


#endif
