/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Faeulhammer
 */

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>

#include <squirrel_object_perception_msgs/Recognize.h>

class SingleViewRecognizerDemo
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    ros::ServiceClient sv_rec_client_;
    ros::Subscriber sub_pc_;
    std::string directory_;
    std::string topic_;
    bool called_service_;

public:
    SingleViewRecognizerDemo()
    {
        called_service_ = false;
    }

    void callSvRecognizerUsingCam(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::cout << "Received point cloud.\n" << std::endl;
        squirrel_object_perception_msgs::Recognize srv;
        srv.request.cloud = *msg;

        if (sv_rec_client_.call(srv))
            std::cout << "Call done..." << std::endl;
        else
            ROS_ERROR("Failed to call service");

        called_service_ = true;
    }

    bool callSvRecognizerUsingFiles()
    {
        std::vector<std::string> test_cloud = v4r::io::getFilesInDirectory(directory_, ".*.pcd", false);
        for(size_t i=0; i < test_cloud.size(); i++)
        {
            pcl::PointCloud<PointT> cloud;
            pcl::io::loadPCDFile(directory_ + "/" + test_cloud[i], cloud);
            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(cloud, cloud_ros);
            squirrel_object_perception_msgs::Recognize srv_rec;
            srv_rec.request.cloud = cloud_ros;

            if (sv_rec_client_.call(srv_rec))
            {
                std::cout << "found " << srv_rec.response.ids.size() << " objects:\n";
                for(size_t i = 0; i < srv_rec.response.ids.size(); i++)
                {
                  std::cout << " ID: " << srv_rec.response.ids[i] << 
                    " position: " << srv_rec.response.transforms[i].translation << std::endl;
                }
            }
            else
            {
                ROS_ERROR("Error calling recognition service. ");
                return false;
            }
        }
        return true;
    }
  
    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "test_recognizer");
        n_ = new ros::NodeHandle ( "~" );

        std::cout <<  "You can either select a topic param 'topic' or "
          " test pcd files from a directory by specifying param 'directory'." << std::endl;

        std::string service_name_sv_rec = "/squirrel_recognize_objects";
        sv_rec_client_ = n_->serviceClient<squirrel_object_perception_msgs::Recognize>(service_name_sv_rec);

        if(n_->getParam ( "directory", directory_ ) && !directory_.empty())
        {
            std::cout << "Opening directory " << directory_ << std::endl;

            callSvRecognizerUsingFiles();
        }
        else
        {
            if(!n_->getParam ( "topic", topic_ ))
            {
                topic_ = "/kinect/depth_registered/points";
            }

            std::cout << "Connecting to camera on topic " << topic_ << std::endl;

            ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &SingleViewRecognizerDemo::callSvRecognizerUsingCam, this);
            ros::Rate loop_rate (1);
            // poll until we did receive a point cloud
            while(!called_service_)
            {
                ros::spinOnce ();
                loop_rate.sleep ();
            }
        }

        return true;
    }
};

int
main (int argc, char ** argv)
{
    SingleViewRecognizerDemo m;
    m.initialize(argc, argv);
    return 0;
}
