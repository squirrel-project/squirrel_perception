#include <squirrel_recognition/recognizer2d_ros.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl/common/time.h>

#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>
#include <v4r/reconstruction/impl/projectPointToImage.hpp>
#include <v4r_config.h>
#include <v4r/keypoints/impl/toString.hpp>
#include <boost/program_options.hpp>

#ifdef HAVE_SIFTGPU
#define USE_SIFT_GPU
#include <v4r/features/FeatureDetector_KD_SIFTGPU.h>
#else
#include <v4r/features/FeatureDetector_KD_CVSIFT.h>
#endif

namespace po = boost::program_options;
namespace v4r
{

cv::Point2f Recognizer2dROS::drawCoordinateSystem(cv::Mat &im, const Eigen::Matrix4f &_pose, const cv::Mat_<double> &_intrinsic, const cv::Mat_<double> &_dist_coeffs, double size, int thickness)
{
    Eigen::Matrix3f R = _pose.topLeftCorner<3,3>();
    Eigen::Vector3f t = _pose.block<3, 1>(0,3);

    Eigen::Vector3f pt0 = R * Eigen::Vector3f(0,0,0) + t;
    Eigen::Vector3f pt_x = R * Eigen::Vector3f(size,0,0) + t;
    Eigen::Vector3f pt_y = R * Eigen::Vector3f(0,size,0) + t;
    Eigen::Vector3f pt_z = R * Eigen::Vector3f(0,0,size) +t ;

    cv::Point2f im_pt0, im_pt_x, im_pt_y, im_pt_z;

    if (!_dist_coeffs.empty())
    {
        v4r::projectPointToImage(&pt0[0], &_intrinsic(0), &_dist_coeffs(0), &im_pt0.x);
        v4r::projectPointToImage(&pt_x[0], &_intrinsic(0), &_dist_coeffs(0), &im_pt_x.x);
        v4r::projectPointToImage(&pt_y[0], &_intrinsic(0), &_dist_coeffs(0), &im_pt_y.x);
        v4r::projectPointToImage(&pt_z[0], &_intrinsic(0), &_dist_coeffs(0), &im_pt_z.x);
    }
    else
    {
        v4r::projectPointToImage(&pt0[0], &_intrinsic(0), &im_pt0.x);
        v4r::projectPointToImage(&pt_x[0], &_intrinsic(0), &im_pt_x.x);
        v4r::projectPointToImage(&pt_y[0], &_intrinsic(0), &im_pt_y.x);
        v4r::projectPointToImage(&pt_z[0], &_intrinsic(0), &im_pt_z.x);
    }

    cv::line(im, im_pt0, im_pt_x, CV_RGB(255,0,0), thickness);
    cv::line(im, im_pt0, im_pt_y, CV_RGB(0,255,0), thickness);
    cv::line(im, im_pt0, im_pt_z, CV_RGB(0,0,255), thickness);

    return im_pt0;
}


bool
Recognizer2dROS::recognize2dROS(squirrel_object_perception_msgs::Recognize2d::Request &req,
                                squirrel_object_perception_msgs::Recognize2d::Response &response)
{
    cv::Mat_<cv::Vec3b> image;
    cv::Mat_<cv::Vec3b> im_draw;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr->image;

    image.copyTo(im_draw);
    imkRecognizer_->dbg = im_draw;

    // track
    { pcl::ScopeTime t("overall time");

        imkRecognizer_->recognize(image, objects);

    } //-- overall time --

    std::cout<<"Confidence value threshold for visualization: "<<thr_conf<<std::endl;
    std::cout<<"Found objects:"<<std::endl;
    for (unsigned j=0; j<objects.size(); j++)
    {
        std::cout<<j<<": name= "<<objects[j].first<<", conf= "<<objects[j].second<<std::endl;
        if (objects[j].second>=thr_conf)
        {
            cv::Point2f origin = drawCoordinateSystem(im_draw, objects[j].third, intrinsic, dist_coeffs, 0.1, 4);
            std::string txt = v4r::toString(j)+std::string(": ")+objects[j].first+std::string(" (")+v4r::toString(objects[j].second)+std::string(")");
            cv::putText(im_draw, txt, origin+cv::Point2f(0,10), cv::FONT_HERSHEY_PLAIN, 1.5,  CV_RGB(255,255,255), 2, CV_AA);
        }

        std_msgs::String ss_tmp;
        ss_tmp.data = objects[j].first;
        response.ids.push_back(ss_tmp);
        response.confidences.push_back(objects[j].second);
        Eigen::Matrix4f trans = objects[j].third;
        geometry_msgs::Transform tt;
        tt.translation.x = trans(0,3);
        tt.translation.y = trans(1,3);
        tt.translation.z = trans(2,3);

        Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
        Eigen::Quaternionf q(rotation);
        tt.rotation.x = q.x();
        tt.rotation.y = q.y();
        tt.rotation.z = q.z();
        tt.rotation.w = q.w();
        response.transforms.push_back(tt);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im_draw).toImageMsg();
    image_pub_.publish(msg);

    return true;
}


bool
Recognizer2dROS::initialize (int argc, char ** argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );

    // init recognizer
#ifdef USE_SIFT_GPU
    v4r::IMKRecognizer::Parameter param;
    /*param.cb_param.nnr = 0.92;
    param.cb_param.thr_desc_rnn = 0.3;
    param.cb_param.max_dist = 0.4;*/
    param.cb_param.nnr = 1.000001;
    param.cb_param.thr_desc_rnn = 0.25;
    param.cb_param.max_dist = FLT_MAX;
    param.pnp_param.eta_ransac = 0.01;
    param.pnp_param.max_rand_trials = 10000;
    param.pnp_param.inl_dist = 4;
    param.vc_param.cluster_dist = 40;
    v4r::FeatureDetector::Ptr detector(new v4r::FeatureDetector_KD_SIFTGPU());
    std::cout << "GPU SIFT" << std::endl;
#else
    v4r::KeypointObjectRecognizer::Parameter param;
    param.cb_param.nnr = .92;
    param.cb_param.thr_desc_rnn = 250.;
    param.cb_param.max_dist = 500;
    v4r::FeatureDetector::Ptr detector(new v4r::FeatureDetector_KD_CVSIFT());
    std::cout << "CV SIFT" << std::endl;
#endif

    boost::shared_ptr<sensor_msgs::CameraInfo const> camera_info;
    camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/kinect/rgb/camera_info", *n_, ros::Duration(10));
    if (camera_info == NULL) {
        ROS_INFO("Using fixed standard camera paramters");
        intrinsic(0,0)=intrinsic(1,1)=525;
        intrinsic(0,2)=320, intrinsic(1,2)=240;
    } else {
        intrinsic(0,0)=camera_info->K[0]; //fx
        intrinsic(1,1)=camera_info->K[4]; //fy
        intrinsic(0,2)=camera_info->K[2]; //cx
        intrinsic(1,2)=camera_info->K[5]; //cy
    }

    setup(argc, argv);

    imkRecognizer_.reset(new v4r::IMKRecognizer(param, detector, detector));
    imkRecognizer_->setCameraParameter(intrinsic, dist_coeffs); //get those parameters from the camera topic
    imkRecognizer_->setDataDirectory(base_dir); //that is the directory with all the models
    if (!codebook_filename.empty())
        imkRecognizer_->setCodebookFilename(codebook_filename);

    if (object_names.size() == 0) { //take all direcotry names from the base_dir
        object_names = v4r::io::getFoldersInDirectory(base_dir);
    }

    for (unsigned i=0; i<object_names.size(); i++) {
        std::cout << object_names[i] << std::endl;
        imkRecognizer_->addObject(object_names[i]); //the names of the models that should be recognized (in our case all directory names in base_dir
    }
    imkRecognizer_->initModels();

    recognize2d_  = n_->advertiseService ("/squirrel_recognize_objects_2d", &Recognizer2dROS::recognize2dROS, this);
    it_.reset(new image_transport::ImageTransport(*n_));
    image_pub_ = it_->advertise("/recognized_2d_object_instances_img", 1, true);
    return true;
}

/**
 * setup
 */
void Recognizer2dROS::setup(int argc, char **argv)
{
    po::options_description general("General options");
    general.add_options()
            ("help,h", "show help message")
            ("models_dir,d", po::value<std::string>(&base_dir)->required(), "Object model directory")
            ("codebook_filename,c", po::value<std::string>(&codebook_filename), "Optional filename for codebook")
            ("object_names,n", po::value< std::vector<std::string> >(&object_names)->multitoken(), "Object names (if empty all directories from base_dir are used")
            ("thr_conf,t", po::value<double>(&thr_conf)->default_value(thr_conf), "Confidence value threshold (visualization)")
            ;

    po::options_description all("");
    all.add(general);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(all).run(), vm);
    po::notify(vm);

}
}

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "squirrel_recognizer2d");
    v4r::Recognizer2dROS m;
    m.initialize (argc, argv);
    ros::spin ();
    return 0;
}
