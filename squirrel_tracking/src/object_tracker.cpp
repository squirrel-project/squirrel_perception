/**
 * This wraps the mono image object tracker from the v4r/KeypointSlam package by Hannes Prankl.
 *
 * @author Michael Zillich
 * @date 2015-03
 */

#include <v4r/keypoints/io.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <squirrel_tracking/object_tracker.hpp>

using namespace std;

SquirrelTrackingNode::SquirrelTrackingNode()
{
  n_ = 0;
  haveCameraInfo = false;
  startedTracking = false;
  intrinsic = cv::Mat::zeros(3, 3, CV_32F);
  dist = cv::Mat::zeros(4, 1, CV_32F);
}

SquirrelTrackingNode::~SquirrelTrackingNode()
{
  delete n_;
}

void SquirrelTrackingNode::initialize(int argc, char ** argv)
{
  ROS_INFO("initialize");
  ROS_INFO("ros::init called");
  n_ = new ros::NodeHandle("~");
  ROS_INFO("node handle created");
  n_->getParam("model_path", modelPath);
  startTrackingService_ = n_->advertiseService("/squirrel_start_object_tracking", &SquirrelTrackingNode::startTracking, this);
  stopTrackingService_ = n_->advertiseService("/squirrel_stop_object_tracking", &SquirrelTrackingNode::stopTracking, this);
  ROS_INFO("Ready to get service calls...");

  caminfoSubscriber = n_->subscribe("/kinect/rgb/camera_info", 1, &SquirrelTrackingNode::receiveCameraInfo, this);

  ros::spin();
}

bool SquirrelTrackingNode::startTracking(squirrel_object_perception_msgs::StartObjectTracking::Request &req, squirrel_object_perception_msgs::StartObjectTracking::Response &response)
{
  bool ret = false;
  if(haveCameraInfo)
  {
    if(!startedTracking)
    {
      v4r::ObjectTrackerMono::Parameter param;
      param.kt_param.plk_param.use_ncc = true;
      param.kt_param.plk_param.ncc_residual = .6;
      tracker.reset(new v4r::ObjectTrackerMono(param));
      tracker->setCameraParameter(intrinsic, dist);

      trackedObjectId = req.object_id.data;
      // HACK: actually we hage to get the model name from the scene database
      // for now, we just take the objectId as the model name
      string filename = modelPath + "/" + trackedObjectId + "/" + trackedObjectId + ".ao";
      ROS_INFO("SquirrelTrackingNode::startTracking: loading '%s'", filename.c_str());
      v4r::ArticulatedObject::Ptr model(new v4r::ArticulatedObject());
      if(v4r::io::read(filename, model))
      {
        tracker->setObjectModel(model);
        imageSubscriber = n_->subscribe("/kinect/rgb/image_rect_color", 5, &SquirrelTrackingNode::receiveImage, this);
        startedTracking = true;
        ret = true;
        ROS_INFO("SquirrelTrackingNode::startTracking: started");
      }
      else
      {
        ROS_ERROR("SquirrelTrackingNode::startTracking: failed to load object model '%s'", filename.c_str());
      }
    }
    else
    {
      ROS_ERROR("SquirrelTrackingNode::startTracking: I am already tracking an object, can only do one at a time.");
    }
  }
  else
  {
    ROS_ERROR("SquirrelTrackingNode::startTracking: missing camera calibration");
  }
  return ret;
}

bool SquirrelTrackingNode::stopTracking(squirrel_object_perception_msgs::StopObjectTracking::Request &req, squirrel_object_perception_msgs::StopObjectTracking::Response &response)
{
  if(startedTracking)
  {
    trackedObjectId = "";
    startedTracking = false;
    imageSubscriber.shutdown();
    ROS_INFO("SquirrelTrackingNode::stopTracking: stopped");
  }
  else
  {
    ROS_ERROR("SquirrelTrackingNode::stopTracking: currently not tracking an object");
  }
  return true;
}

void SquirrelTrackingNode::receiveCameraInfo(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
  intrinsic.at<float>(0, 0) = msg->K[0];
  intrinsic.at<float>(0, 2) = msg->K[2];
  intrinsic.at<float>(1, 1) = msg->K[4];
  intrinsic.at<float>(1, 2) = msg->K[5];
  intrinsic.at<float>(2, 2) = 1.;
  // NOTE: assume kinect distortion parameters to be 0
  haveCameraInfo = true;

  // we only need that once, so shutdown now
  caminfoSubscriber.shutdown();

  ROS_INFO("SquirrelTrackingNode::receiveCameraInfo: have camera parameters");
}

void SquirrelTrackingNode::receiveImage(const sensor_msgs::Image::ConstPtr &msg)
{
  Eigen::Matrix4f pose;
  double conf;
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
  cv::Mat image = cv_ptr->image;
  // ROS_INFO("squirrel_tracking: new image");
  if(tracker->track(image, pose, conf))
  {
    ROS_INFO("squirrel_tracking: conf %.3f, pos %.3f %.3f %.3f", conf, pose(0,3), pose(1,3), pose(2,3));
    tf::Transform transform;
    tf::Vector3 p(pose(0,3), pose(1,3), pose(2,3));
    tf::Matrix3x3 R;
    tf::Quaternion q;
    R[0][0] = pose(0, 0);
    R[0][1] = pose(0, 1);
    R[0][2] = pose(0, 2);
    R[1][0] = pose(1, 0);
    R[1][1] = pose(1, 1);
    R[1][2] = pose(1, 2);
    R[2][0] = pose(2, 0);
    R[2][1] = pose(2, 1);
    R[2][2] = pose(2, 2);
    R.getRotation(q);
    transform.setOrigin(p);
    transform.setRotation(q);
    tfBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect_rgb_optical_frame", trackedObjectId));
    // for debugging
    // cv::imwrite("tracking.jpg", image);
  }
  else
  {
    // ROS_INFO("squirrel_tracking: tracking failed for this image");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracker");
  SquirrelTrackingNode tracker;
  tracker.initialize(argc, argv);

  return 0;
}
