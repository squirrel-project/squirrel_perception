/*
 * main.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Thomas Faeulhammer
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>

#include <squirrel_object_perception_msgs/Recognize2d.h>

class Recognizer2dDemo
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
    Recognizer2dDemo()
    {
        called_service_ = false;
    }

    void callSvRecognizerUsingCam(const sensor_msgs::Image::ConstPtr& msg)
    {
        std::cout << "Received camera image.\n" << std::endl;
        squirrel_object_perception_msgs::Recognize2d srv;
        srv.request.image = *msg;

        if (sv_rec_client_.call(srv))
            std::cout << "Call done..." << std::endl;
        else
            ROS_ERROR("Failed to call service");

        called_service_ = true;
    }

    bool callSvRecognizerUsingFiles()
    {
        std::vector<std::string> test_image = v4r::io::getFilesInDirectory(directory_, ".*.png", false);
        for(size_t i=0; i < test_image.size(); i++)
        {
            cv::Mat image = cv::imread(directory_ + "/" + test_image[i], 1);

            cv_bridge::CvImage out_msg;
            out_msg.header.frame_id = "/kinect_rgb_optical_frame";
            out_msg.header.stamp = ros::Time::now();
            out_msg.encoding = sensor_msgs::image_encodings::RGB8;
            out_msg.image = image;

            squirrel_object_perception_msgs::Recognize2d srv_rec;
            srv_rec.request.image = *out_msg.toImageMsg();

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
        ros::init (argc, argv, "test_recognizer2d");
        n_ = new ros::NodeHandle ( "~" );

        std::cout <<  "You can either select a topic param 'topic' or "
          " test pcd files from a directory by specifying param 'directory'." << std::endl;

        std::string service_name_sv_rec = "/squirrel_recognizer2d/squirrel_recognize_objects_2d";
        sv_rec_client_ = n_->serviceClient<squirrel_object_perception_msgs::Recognize2d>(service_name_sv_rec);

        if(n_->getParam ( "directory", directory_ ) && !directory_.empty())
        {
            std::cout << "Opening directory " << directory_ << std::endl;

            callSvRecognizerUsingFiles();
        }
        else
        {
            if(!n_->getParam ( "topic", topic_ ))
            {
                topic_ = "/kinect/rgb/image_rect_color";
            }

            std::cout << "Connecting to camera on topic " << topic_ << std::endl;

            ros::Subscriber sub_pc = n_->subscribe (topic_, 1, &Recognizer2dDemo::callSvRecognizerUsingCam, this);
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
    Recognizer2dDemo m;
    m.initialize(argc, argv);
    return 0;
}
