
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <squirrel_object_perception_msgs/LookAtImagePosition.h>
#include <squirrel_attention/AttentionMotion.h>

using namespace cv;
using namespace std;

AttentionMotion::AttentionMotion()
: it_(nh_)
{
  panning_ = false;
  tilting_ = false;
  moving_ = false;
  cameraSteady_ = true;
  imageSub_= it_.subscribe("/camera/rgb/image_raw", 1, &AttentionMotion::imageCallback, this);
  imagePub_ = it_.advertise("/attention_motion/saliency", 1);
  controllerSrv_ = nh_.serviceClient<squirrel_object_perception_msgs::LookAtImagePosition>("/attention/look_at_image_position");
  panStateSub_ = nh_.subscribe("/pan_controller/state", 2, &AttentionMotion::panStateCallback, this);
  tiltStateSub_ = nh_.subscribe("/tilt_controller/state", 2, &AttentionMotion::tiltStateCallback, this);
  steadyTimer_ = nh_.createTimer(ros::Duration(0.25), &AttentionMotion::cameraSteadyCallback, this, false, false);
}

AttentionMotion::~AttentionMotion()
{
}

void AttentionMotion::run()
{
  ros::spin();
}

void AttentionMotion::panStateCallback(const dynamixel_msgs::JointState::ConstPtr& panStateMsg)
{
  movingMutex_.lock();
  panning_ = panStateMsg->is_moving;
  checkMovement();
  movingMutex_.unlock();
}

void AttentionMotion::tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr& tiltStateMsg)
{
  movingMutex_.lock();
  tilting_ = tiltStateMsg->is_moving;
  checkMovement();
  movingMutex_.unlock();
}

void AttentionMotion::checkMovement()
{
  bool nowMoving = panning_ || tilting_;
  // if the camera just stopped moving, start the timer (if it is not already started)
  if(!nowMoving && moving_)
  {
    if(!steadyTimer_.hasPending())
      steadyTimer_.start();
  }
  // clear the steady bit as soon as the camera moves, and stop the timer
  if(nowMoving)
  {
    cameraSteady_ = false;
    if(steadyTimer_.hasPending())
      steadyTimer_.stop();
  }
  moving_ = nowMoving;
}

void AttentionMotion::cameraSteadyCallback(const ros::TimerEvent& event)
{
  movingMutex_.lock();
  cameraSteady_ = true;
  movingMutex_.unlock();
}

void AttentionMotion::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  const float MIN_FLOW = 3.;

  bool goAhead = false;
  movingMutex_.lock();
  goAhead = cameraSteady_;
  movingMutex_.unlock();

  if(!cameraSteady_)
    return;

  cv_bridge::CvImageConstPtr cvPtr;
  try
  {
    cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  resize(cvPtr->image, frame, Size(), 0.125, 0.125, INTER_NEAREST);
  cvtColor(frame, gray, COLOR_BGR2GRAY);

  if(prevgray.data)
    calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
  std::swap(prevgray, gray);

  int iMax = -1, jMax = -1;
  float flowMax = 0.;
  for(int i = 0; i < flow.rows; i++)
    for(int j = 0; j < flow.cols; j++)
    {
      float fx = flow.at<Vec2f>(i, j)[0];
      float fy = flow.at<Vec2f>(i, j)[1];
      float f = sqrt(fx*fx + fy*fy);
      if(f > MIN_FLOW && f > flowMax)
      {
        flowMax = f;
        iMax = i;
        jMax = j;
      }
    }

  if(flowMax > 0.)
  {
    squirrel_object_perception_msgs::LookAtImagePosition lookSrv;
    lookSrv.request.x = jMax*8. - 320.;
    lookSrv.request.y = iMax*8. - 240.;
    lookSrv.request.why = "motion";
    controllerSrv_.call(lookSrv);
  }

  // visualisation
  cflow.create(gray.rows, gray.cols,CV_8UC3);
  for(int i = 0; i < flow.rows; i++)
    for(int j = 0; j < flow.cols; j++)
    {
      float fx = flow.at<Vec2f>(i, j)[0];
      float fy = flow.at<Vec2f>(i, j)[1];
      float f = sqrt(fx*fx + fy*fy);
      f *= 30.;  // scale to be visible in a [0,255] range
      cflow.at<Vec3b>(i, j) = Vec3b((unsigned char)f, (unsigned char)f, (unsigned char)f);
    }
  if(flowMax > 0.)
    circle(cflow, Point(jMax, iMax), 5, Scalar(255, 0, 0), 1);
  cv_bridge::CvImagePtr outPtr(new cv_bridge::CvImage);
  outPtr->encoding = "bgr8";
  outPtr->image = cflow;
  imagePub_.publish(outPtr->toImageMsg());
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "attention_motion_node");
  AttentionMotion am;
  am.run();
  exit(0);
}
