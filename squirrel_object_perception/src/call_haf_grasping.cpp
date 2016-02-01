#include <cstdlib>
#include <iostream>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <ros/ros.h>


class PointCloud2topic {


protected:

    ros::NodeHandle* nh;
    ros::Publisher pub; 

public:
    void initialize(int argc, char ** argv) {
        ros::init(argc, argv, "cloud2pcd");
        nh = new ros::NodeHandle ( "~" );
        ROS_INFO("Initialized");
    }

    void pointcloud2topic() {
        ROS_INFO("Got input. Send point cloud to haf_grasping action client/server");
	pub = nh->advertise<sensor_msgs::PointCloud2>("/haf_grasping/depth_registered/single_cloud/points_in_lcs", 100);
        sensor_msgs::PointCloud2ConstPtr scene;
        scene = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", *nh, ros::Duration(50));

        pub.publish(scene);
        ROS_INFO("PointCloud published");
        // wait 5 seconds to make sure the data is actually published	
        ros::Duration(5).sleep();
    }

    ~PointCloud2topic() {
        if (nh) {
            delete(nh);
        }
    }

};

int main(int argc, char **argv)
{
    PointCloud2topic pc2topic;
    pc2topic.initialize(argc, argv);
    pc2topic.pointcloud2topic();







}

