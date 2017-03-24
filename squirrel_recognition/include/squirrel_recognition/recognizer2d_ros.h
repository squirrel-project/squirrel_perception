#include <squirrel_object_perception_msgs/Recognize2d.h>
#include <image_transport/image_transport.h>
#include <v4r/recognition/IMKRecognizer.h>

namespace v4r
{

class Recognizer2dROS
{
private:
   
    boost::shared_ptr<IMKRecognizer> imkRecognizer_;
    
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::ServiceServer recognize2d_;
    image_transport::Publisher image_pub_;
    ros::Subscriber sub_ci;
    std::vector<v4r::triple<std::string, double, Eigen::Matrix4f> > objects;
    cv::Mat_<double> dist_coeffs;// = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat_<double> intrinsic = cv::Mat_<double>::eye(3,3);
    boost::shared_ptr<image_transport::ImageTransport> it_;

    double thr_conf = 0.4;
    std::string base_dir, codebook_filename;
    std::vector<std::string> object_names;

    void setup(int argc, char **argv);
    void callSvCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
    cv::Point2f drawCoordinateSystem(cv::Mat &im, const Eigen::Matrix4f &pose, const cv::Mat_<double> &intrinsic, const cv::Mat_<double> &dist_coeffs, double size, int thickness);

public:
    Recognizer2dROS() {
        codebook_filename="";
    }

    bool recognize2dROS (squirrel_object_perception_msgs::Recognize2d::Request & req,
                    squirrel_object_perception_msgs::Recognize2d::Response & response);

    bool initialize (int argc, char ** argv);
};

}
