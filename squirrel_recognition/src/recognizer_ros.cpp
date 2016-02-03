#include <squirrel_recognition/recognizer_ros.h>
#include <squirrel_object_perception_msgs/BBox.h>

#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <v4r/common/miscellaneous.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/common/visibility_reasoning.h>
#include <v4r/io/filesystem.h>

#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <sstream>

#include <boost/program_options.hpp>
#include <glog/logging.h>

namespace po = boost::program_options;

namespace v4r
{
template<typename PointT>
bool
RecognizerROS<PointT>::respondSrvCall(squirrel_object_perception_msgs::Recognize::Request &req,
                            squirrel_object_perception_msgs::Recognize::Response &response) const
{
    typename pcl::PointCloud<PointT>::Ptr pRecognizedModels (new pcl::PointCloud<PointT>);
    cv::Mat annotated_img = ConvertPCLCloud2Image(*scene_);

    for (size_t j = 0; j < models_verified_.size(); j++)
    {
      std_msgs::String ss_tmp;
      ss_tmp.data = models_verified_[j]->id_;
      response.ids.push_back(ss_tmp);

      Eigen::Matrix4f trans = transforms_verified_[j];
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

      typename pcl::PointCloud<PointT>::ConstPtr model_cloud = models_verified_[j]->getAssembled ( resolution_ );
      typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_verified_[j]);
      *pRecognizedModels += *model_aligned;
      sensor_msgs::PointCloud2 rec_model;
      pcl::toROSMsg(*model_aligned, rec_model);
      response.model_clouds.push_back(rec_model);

      pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = models_verified_[j]->getNormalsAssembled ( resolution_ );

      pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
      transformNormals(*normal_cloud, *normal_aligned, transforms_verified_[j]);

      //ratio of inlier points
      float confidence = 0;
      const float focal_length = 525.f;
      const int img_width = 640;
      const int img_height = 480;

      VisibilityReasoning<pcl::PointXYZRGB> vr (focal_length, img_width, img_height);
      vr.setThresholdTSS (0.01f);
      /*float fsv_ratio =*/ vr.computeFSVWithNormals (scene_, model_aligned, normal_aligned);
      confidence = 1.f - vr.getFSVUsedPoints() / static_cast<float>(model_aligned->points.size());
      response.confidences.push_back(confidence);

      //centroid and BBox
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*model_aligned, centroid);
      geometry_msgs::Point32 centroid_msg;
      centroid_msg.x = centroid[0];
      centroid_msg.y = centroid[1];
      centroid_msg.z = centroid[2];
      response.centroids.push_back(centroid_msg);

      Eigen::Vector4f min;
      Eigen::Vector4f max;
      pcl::getMinMax3D (*model_aligned, min, max);

      squirrel_object_perception_msgs::BBox bbox;
      geometry_msgs::Point32 pt;
      pt.x = min[0]; pt.y = min[1]; pt.z = min[2]; bbox.points.push_back(pt);
      pt.x = min[0]; pt.y = min[1]; pt.z = max[2]; bbox.points.push_back(pt);
      pt.x = min[0]; pt.y = max[1]; pt.z = min[2]; bbox.points.push_back(pt);
      pt.x = min[0]; pt.y = max[1]; pt.z = max[2]; bbox.points.push_back(pt);
      pt.x = max[0]; pt.y = min[1]; pt.z = min[2]; bbox.points.push_back(pt);
      pt.x = max[0]; pt.y = min[1]; pt.z = max[2]; bbox.points.push_back(pt);
      pt.x = max[0]; pt.y = max[1]; pt.z = min[2]; bbox.points.push_back(pt);
      pt.x = max[0]; pt.y = max[1]; pt.z = max[2]; bbox.points.push_back(pt);
      response.bboxes.push_back(bbox);

      const float cx = static_cast<float> (img_width) / 2.f; //- 0.5f;
      const float cy = static_cast<float> (img_height) / 2.f; // - 0.5f;

      int min_u, min_v, max_u, max_v;
      min_u = img_width;
      min_v = img_height;
      max_u = max_v = 0;

      for(size_t m_pt_id=0; m_pt_id < model_aligned->points.size(); m_pt_id++)
      {
          const float x = model_aligned->points[m_pt_id].x;
          const float y = model_aligned->points[m_pt_id].y;
          const float z = model_aligned->points[m_pt_id].z;
          const int u = static_cast<int> (focal_length * x / z + cx);
          const int v = static_cast<int> (focal_length * y / z + cy);

          if (u >= img_width || v >= img_width || u < 0 || v < 0)
            continue;

          if(u < min_u)
              min_u = u;

          if(v < min_v)
              min_v = v;

          if(u > max_u)
              max_u = u;

          if(v > max_v)
              max_v = v;
      }

      cv::Point text_start;
      text_start.x = min_u;
      text_start.y = std::max(0, min_v - 10);
      cv::putText(annotated_img, models_verified_[j]->id_, text_start, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,255), 1, CV_AA);
      cv::rectangle(annotated_img, cv::Point(min_u, min_v), cv::Point(max_u, max_v), cv::Scalar( 0, 255, 255 ), 2);
    }

    sensor_msgs::PointCloud2 recognizedModelsRos;
    pcl::toROSMsg (*pRecognizedModels, recognizedModelsRos);
    recognizedModelsRos.header.frame_id = req.cloud.header.frame_id;
    vis_pc_pub_.publish(recognizedModelsRos);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", annotated_img).toImageMsg();
    image_pub_.publish(msg);

    return true;
}

template<typename PointT>
bool
RecognizerROS<PointT>::recognizeROS(squirrel_object_perception_msgs::Recognize::Request &req,
                                 squirrel_object_perception_msgs::Recognize::Response &response)
{
    scene_.reset(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg (req.cloud, *scene_);

    if( chop_z_ > 0)
    {
        pcl::PassThrough<PointT> pass;
        pass.setFilterLimits ( 0.f, chop_z_ );
        pass.setFilterFieldName ("z");
        pass.setInputCloud (scene_);
        pass.setKeepOrganized (true);
        pass.filter (*scene_);
    }

    rr_->setInputCloud (scene_);
    rr_->recognize();
    models_verified_ = rr_->getVerifiedModels();
    transforms_verified_ = rr_->getVerifiedTransforms();
    bool b = respondSrvCall(req, response);
    if(visualize_)
        rr_->visualize();
    return b;
}

template<typename PointT>
bool
RecognizerROS<PointT>::initialize (int argc, char ** argv)
{
    n_.reset( new ros::NodeHandle ( "~" ) );

    ROS_INFO("A");
    po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("visualize,v", po::bool_switch(&visualize_), "visualize recognition results")
        ("chop_z,z", po::value<double>(&chop_z_)->default_value(chop_z_, boost::str(boost::format("%.2e") % chop_z_) ), "points with z-component higher than chop_z_ will be ignored (low chop_z reduces computation time and false positives (noise increase with z)")
   ;
    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    po::store(parsed, vm);
    if (vm.count("help")) { std::cout << desc << std::endl; }
    try { po::notify(vm); }
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }

    ROS_INFO("B");
    rr_.reset( new v4r::MultiRecognitionPipeline<PointT> (argc, argv));
    vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "sv_recogniced_object_instances", 1 );
    recognize_  = n_->advertiseService ("sv_recognition", &RecognizerROS::recognizeROS, this);

    it_.reset(new image_transport::ImageTransport(*n_));
    image_pub_ = it_->advertise("sv_recogniced_object_instances_img", 1, true);

    ROS_INFO("C");
    return true;
}

}

int
main (int argc, char ** argv)
{
  std::cout << "kacke\n";
  ros::init (argc, argv, "squirrel_recognizer");
  ROS_INFO("0");
  v4r::RecognizerROS<pcl::PointXYZRGB> m;
  ROS_INFO("1");
  m.initialize (argc, argv);
  ros::spin ();
  return 0;
}
