#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Transform.h>

#include <v4r/io/filesystem.h>

#include <squirrel_object_perception_msgs/Recognize.h>
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
    std::vector<std::string> models_;

public:
    RecognizerWizard()
    {
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
            return true;
        }
        else
            ROS_ERROR("Failed to call service");

        float max_confidence = 0.;
        for (int i=0; i < res.confidence.size(); i++)
        {
            if (res.confidence[i] > max_confidence)
                max_confidence = res.confidence[i];
        }

        if (!vision_success || max_confidence < confidence_threshold_)
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

        std::string service_name_sv_rec = "/squirrel_recognizer/squirrel_recognize_objects";

        if(!n_->getParam ( "confidence_threshold", confidence_threshold_ ))
        {
            confidence_threshold_ = 0.2;
        }
        if(!n_->getParam ( "models_dir", models_dir_ ))
        {
            ROS_ERROR("You must input a model directory!");
            return false;
        }
        if (!loadModels())
        {
            ROS_ERROR("Failed to load model names in directory %s", models_dir_.c_str());
            return false;
        }

        sv_rec_client_ = n_->serviceClient<squirrel_object_perception_msgs::Recognize>(service_name_sv_rec);
        recognize_  = n_->advertiseService ("squirrel_wizard_recognize", &RecognizerWizard::callSvRecognizer, this);

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
