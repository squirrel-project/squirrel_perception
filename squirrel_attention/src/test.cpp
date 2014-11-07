/*
 * squirrel_attention_location.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions.h>
#include "squirrel_object_perception_msgs/get_saliency.h"
#include "v4r/AttentionModule/AttentionModule.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class SOCDemo
{
  private:
    typedef pcl::PointXYZRGB PointT;
    pcl::PointCloud<PointT>::Ptr cloud_;
    int service_calls_;
    ros::NodeHandle n_;
    std::string cloud_name_;
    
    void
    readCloud ()
    {
      cloud_.reset(new pcl::PointCloud<PointT>);

      if (pcl::io::loadPCDFile<PointT> (cloud_name_.c_str(), *cloud_) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file \n");
        return;
      }
      std::cout << "Cloud is up and running" << std::endl;
    }

    void
    callService (const sensor_msgs::PointCloud2& msg)
    {
      //if( (service_calls_ % (30 * 5)) == 0)
      {
        std::cout << "going to call service..." << std::endl;
        ros::ServiceClient client = n_.serviceClient<squirrel_object_perception_msgs::get_saliency>("/squirrel_attention_location");
        squirrel_object_perception_msgs::get_saliency srv;
        srv.request.cloud = msg;
        if (client.call(srv))
        {
          std::cout << "Saliency calculated!" << std::endl;
        }
        else
        {
          std::cout << "Call did not succeed" << std::endl;
        }
      }
      service_calls_++;
    }

  public:
    SOCDemo()
    {
      cloud_name_ = "/home/ekaterina/work/catkin_ws/test45.pcd";
    }

    bool initialize(int argc, char ** argv)
    {
      readCloud();
      return true;
    }

    void run()
    {
      while(true)
      {
        sensor_msgs::PointCloud2 msg;
        //toROSMsg (const pcl::PointCloud< PointT > &cloud, sensor_msgs::PointCloud2 &msg)
        pcl::toROSMsg (*cloud_,msg);
        callService (msg);
        ros::spinOnce();
      }
    }
};

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "attention_demo");

  SOCDemo m;
  m.initialize (argc, argv);
  m.run();
  return 0;
}
