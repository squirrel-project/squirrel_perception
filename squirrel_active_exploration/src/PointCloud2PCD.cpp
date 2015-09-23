#include <cstdlib>
#include <iostream>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "squirrel_active_exploration/pcl_conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <pcl/common/pca.h> // this brings in transformPointCloud, but should look for the more specific file

#define _CLOUD_PREFIX "cloud_"
#define _TRANSFORMATION_PREFIX "transformation_"
#define _VERIFICATION_PREFIX "verification_cloud_"

using namespace std;
using namespace pcl;

class PointCloud2PCD {


protected:

    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle* nh;

    static const bool savePose = true;

public:
    void initialize(int argc, char ** argv) {
        ros::init(argc, argv, "cloud2pcd");
        nh = new ros::NodeHandle ( "~" );
        ROS_INFO("Initialized");
    }

    void pointcloud2pcd() {

        string dir = "";
        int id = 100;
        nh->getParam("directory", dir);
        nh->getParam("id", id);

        // Add a backslash
        if (dir.size() == 0)
        {
            dir = "/";
        }
        else
        {
            // If the last character is already a back slash then ignore
            char last_char = dir[dir.length()-1];
            if (last_char != '/')
                dir = dir + "/";
        }

        // Point cloud
        sensor_msgs::PointCloud2ConstPtr scene = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points",
                                                                                                      *nh, ros::Duration(50));
        PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*scene,pcl_pc2);
        PointCloud<PointT> cloud;
        fromPCLPointCloud2(pcl_pc2, cloud);
        ROS_INFO("pointcloud2pcd : read %lu points", cloud.size());
        string f = dir + _CLOUD_PREFIX + boost::lexical_cast<string>(id) + ".pcd";
        io::savePCDFileBinary (f, cloud);
        ROS_INFO("pointcloud2pcd : point cloud saved");
        /*
        // Transform
        tf::StampedTransform transform;
        tf::TransformListener tf_listener;
        tf_listener.waitForTransform("/kinect_depth_optical_frame", "/map", ros::Time(0), ros::Duration(5.0));
        tf_listener.lookupTransform ("/kinect_depth_optical_frame", "/map", ros::Time(0), transform);
        tf::Vector3 t = transform.getOrigin();
        tf::Matrix3x3 r = transform.getBasis();
        Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
        tf(0,0) = r[0][0];
        tf(0,1) = r[0][1];
        tf(0,2) = r[0][2];
        tf(0,3) = t[0];
        tf(1,0) = r[1][0];
        tf(1,1) = r[1][1];
        tf(1,2) = r[1][2];
        tf(1,3) = t[1];
        tf(2,0) = r[2][0];
        tf(2,1) = r[2][1];
        tf(2,2) = r[2][2];
        tf(2,3) = t[2];
        tf = tf.inverse();  // inverse the transform
        ofstream out;
        f = dir + _TRANSFORMATION_PREFIX + boost::lexical_cast<string>(id) + ".txt";
        out.open(f.c_str());
        // Write results
        if (out.is_open())
        {
            out << tf(0,0) << " " << tf(0,1) << " " << tf(0,2) << " " << tf(0,3) << " "
                << tf(1,0) << " " << tf(1,1) << " " << tf(1,2) << " " << tf(1,3) << " "
                << tf(2,0) << " " << tf(2,1) << " " << tf(2,2) << " " << tf(2,3) << " "
                << tf(3,0) << " " << tf(3,1) << " " << tf(3,2) << " " << tf(3,3) << endl;
            out.close();
            ROS_INFO("pointcloud2pcd : transformation saved");
            cout << tf << endl;
        }
        else
        {
            ROS_ERROR("pointcloud2pcd : could not open file to save transform %s", f.c_str());
        }
        // Save a transformed point cloud for verification
        PointCloud<PointT> transformed_cloud;
        transformPointCloud (cloud, transformed_cloud, tf);
        f = dir + _VERIFICATION_PREFIX + boost::lexical_cast<string>(id) + ".pcd";
        io::savePCDFileBinary (f, transformed_cloud);
        */
    }

    ~PointCloud2PCD() {
        if (nh) {
            delete(nh);
        }
    }

};

int main(int argc, char **argv)
{
    PointCloud2PCD pc2pcd;
    pc2pcd.initialize(argc, argv);
    pc2pcd.pointcloud2pcd();
}
