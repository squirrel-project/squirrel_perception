#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <v4r/io/filesystem.h>
#include <tf/transform_listener.h>

#include <squirrel_object_perception_msgs/Recognize.h>
#include <squirrel_object_perception_msgs/Recognize2d.h>
#include <squirrel_object_perception_msgs/BBox.h>

class RecognizerWizard
{
private:
    typedef pcl::PointXYZRGB PointT;
    ros::NodeHandle *n_;
    ros::ServiceServer recognize_;
    ros::ServiceClient sv_rec_client_;
    float confidence_threshold_;
    bool called_service_;
    std::string models_dir_;
    std::string mode_;
    std::vector<std::string> models_;

public:
    RecognizerWizard()
    {
    }

    bool checkConfidenceThreshold(std::vector<float> & confidence)
    {
        if (confidence.size() == 0)
            return false;

        float max_confidence = 0.;
        for (int i=0; i < confidence.size(); i++)
        {
            if (confidence[i] > max_confidence)
                max_confidence = confidence[i];
        }

        std::cout << "max confidence: " << max_confidence << std::endl;

        return max_confidence < confidence_threshold_;
    }

    bool callSvRecognizer(squirrel_object_perception_msgs::Recognize::Request &req,
                          squirrel_object_perception_msgs::Recognize::Response &res)
    {
        std::cout << "Received point cloud." << std::endl;
        squirrel_object_perception_msgs::Recognize srv;
        srv.request = req;
        bool vision_success = false;

        if (sv_rec_client_.call(srv) && srv.response.ids.size() > 0)
        {
            std::cout << "Call done..." << std::endl;
            vision_success = true;
            res = srv.response;
        }
        else
            ROS_ERROR("Failed to call service");

        if (!vision_success || checkConfidenceThreshold(res.confidence))
        {
            while (true)
            {
                int class_number = 0;
                displayClass();
                std::cout << ">> ";
                std::cin >> class_number;

                if (class_number >= 0 && class_number < models_.size())
                {
                    std_msgs::String input_id;
                    input_id.data = models_[class_number];
                    res.ids.push_back(input_id);
                    geometry_msgs::Transform empty_transform;
                    res.transforms.push_back(empty_transform);
                    res.confidence.push_back(1.0);
                    geometry_msgs::Point32 empty_centroid;
                    res.centroid.push_back(empty_centroid);
                    squirrel_object_perception_msgs::BBox empty_bbox;
                    res.bbox.push_back(empty_bbox);
                    res.models_cloud.push_back(req.cloud);

                    break;
                }
            }
        }
        if (res.ids.size() == 0)
            return false;

        return true;
    }

    bool callSv2dRecognizer(squirrel_object_perception_msgs::Recognize2d::Request &req,
                          squirrel_object_perception_msgs::Recognize2d::Response &res)
    {
        std::cout << "Received point cloud." << std::endl;
        squirrel_object_perception_msgs::Recognize2d srv;
        srv.request = req;
        bool vision_success = false;

        if (sv_rec_client_.call(srv) && srv.response.ids.size() > 0)
        {
            std::cout << "Call done..." << std::endl;
            vision_success = true;
            res = srv.response;
        }
        else
            ROS_ERROR("Failed to call service");

        if (!vision_success || checkConfidenceThreshold(res.confidences))
        {
            while (true)
            {
                //Handle failure case:
                int class_number = 0;
                displayClass();
                std::cout << ">> ";
                std::cin >> class_number;

                if (class_number >= 0 && class_number < models_.size())
                {
                    std_msgs::String input_id;
                    input_id.data = models_[class_number];
                    res.ids.push_back(input_id);

                    tf::TransformListener tf_listener;
                    geometry_msgs::PoseStamped before, after;
                    before.header.frame_id = "/hand_palm_link";
                    before.pose.position.x = 0;
                    before.pose.position.y = 0;
                    before.pose.position.z = 0;
                    before.pose.orientation.x = 0;
                    before.pose.orientation.y = 0;
                    before.pose.orientation.z = 0;
                    before.pose.orientation.w = 1;
                    try
                    {
                        tf_listener.waitForTransform("/hand_palm_link", "/kinect_rgb_optical_frame", ros::Time::now(), ros::Duration(1.0));
                        tf_listener.transformPose("/kinect_rgb_optical_frame", before, after);
                       }
                    catch (tf::TransformException& ex)
                    {
                        ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
                    }

                    geometry_msgs::Transform hand;
                    hand.translation.x = after.pose.position.x;
                    hand.translation.y = after.pose.position.y;
                    hand.translation.z = after.pose.position.z;
                    hand.rotation.x = 0;
                    hand.rotation.y = 0;
                    hand.rotation.z = 0;
                    hand.rotation.w = 1;
                    res.transforms.push_back(hand);
                    res.confidences.push_back(1.0);

                    std::cout << hand.translation << std::endl;

                    break;
                }
            }
        }
        if (res.ids.size() == 0)
            return false;

        return true;
    }

    void displayClass()
    {
        std::cout << "Please select one option: (Total number: " << models_.size() << ")" << std::endl;
        for (int i=0; i < models_.size(); i++)
        {
            std::cout << "[ " << i << " ] " << models_[i] << std::endl;
        }
    }

    bool loadModels()
    {
        models_.clear();
        models_ = v4r::io::getFoldersInDirectory (models_dir_);
        std::cout << "There are " << models_.size() << " models." << std::endl;

        if (models_.size() == 0)
            return false;

        return true;
    }

    bool initialize(int argc, char ** argv)
    {
        ros::init (argc, argv, "recognizer_wizard");
        n_ = new ros::NodeHandle ( "~" );

        std::string service_name_sv_rec;


        if (!n_->getParam ( "mode", mode_ ))
        {
            ROS_ERROR("unknown mode. Valid options are 2d or 3d. Defaulting to 3d!");
            mode_ = "3d";
        }
        std::cout << "Wizard is running in mode " << mode_ << std::endl;

        if (!n_->getParam ( "service_recognition", service_name_sv_rec ))
        {
            if (mode_ == "2d")
            {
                service_name_sv_rec = "/squirrel_recognize_objects_2d";
            }
            else
            {
                service_name_sv_rec = "/squirrel_recognize_objects";
            }
        }

        if(!n_->getParam ( "confidence_threshold", confidence_threshold_ ))
        {
            confidence_threshold_ = 0.2;
        }
        if(!n_->getParam ( "models_dir", models_dir_ ))
        {
            ROS_ERROR("You must set the models_dir parameter to a model directory!");
            return false;
        }
        if (!loadModels())
        {
            ROS_ERROR("Failed to load model names in directory %s", models_dir_.c_str());
            return false;
        }

        if (mode_ == "2d")
        {
            sv_rec_client_ = n_->serviceClient<squirrel_object_perception_msgs::Recognize2d>(service_name_sv_rec);
            recognize_  = n_->advertiseService ("/squirrel_wizard_recognize2d", &RecognizerWizard::callSv2dRecognizer, this);
        }
        else if (mode_ == "3d")
        {
            sv_rec_client_ = n_->serviceClient<squirrel_object_perception_msgs::Recognize>(service_name_sv_rec);
            recognize_  = n_->advertiseService ("/squirrel_wizard_recognize", &RecognizerWizard::callSvRecognizer, this);
        }
        else
        {
            ROS_ERROR("unknown mode. Valid options are 2d or 3d");
            return false;
        }

        ros::spin();

        return true;
    }
};

int
main (int argc, char ** argv)
{
    RecognizerWizard m;
    m.initialize(argc, argv);
    return 0;
}
