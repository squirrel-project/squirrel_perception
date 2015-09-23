/**
 * AttentionMotion.cpp
 *
 * Attends to the "most" moving part in the image. What is considered "most"
 * depends on the optical flow method used. It might mean largest or fastest.
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#include <math.h>
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
  subsample_ = 8;
  imageSub_= it_.subscribe("/camera/rgb/image_raw", 1, &AttentionMotion::imageCallback, this);
  imagePub_ = it_.advertise("/attention_motion/saliency", 1);
  controllerSrv_ = nh_.serviceClient<squirrel_object_perception_msgs::LookAtImagePosition>("/attention/look_at_image_position");
  panStateSub_ = nh_.subscribe("/pan_controller/state", 2, &AttentionMotion::panStateCallback, this);
  tiltStateSub_ = nh_.subscribe("/tilt_controller/state", 2, &AttentionMotion::tiltStateCallback, this);
  steadyTimer_ = nh_.createTimer(ros::Duration(0.10), &AttentionMotion::cameraSteadyCallback, this, false, false);
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

bool AttentionMotion::calculateFlowCenter(float &cx, float &cy, float &mag, float &angle)
{
  // ignore flow lower than this as noise
  const float MIN_FLOW = 8./(float)subsample_;

  mag = 0.;
  cx = 0.;
  cy = 0.;
  angle = 0.;

  // NOTE: The flow image must be set to zero, as sometimes the flow
  // computation does not overwrite the output image, so the old "ghost"
  // flow persists and leasd to craze things.
  // NOTE: Is there a more efficient way to zero an exiting matrix?
  // FIX: actually this weird persistence can be a result of skipping the first
  // frame after movment! -> check that, and move all stuff within the if
  flow_ = Mat::zeros(gray_.rows, gray_.cols, CV_32FC2);
  calcOpticalFlowFarneback(prevgray_, gray_, flow_, 0.5, 3, 11, 3, 5, 1.2, 0);

  float sum = 0.;
  int flowCnt = 0;
  for(int i = 0; i < flow_.rows; i++)
    for(int j = 0; j < flow_.cols; j++)
    {
      float fx = flow_.at<Vec2f>(i, j)[0];
      float fy = flow_.at<Vec2f>(i, j)[1];
      float f = sqrt(fx*fx + fy*fy);
      if(f > MIN_FLOW)
        flowCnt++;
      sum += f;
      cx += f*(float)j;
      cy += f*(float)i;
    }
  if(isnormal(sum))
  {
    cx /= sum;
    cy /= sum;
    float fx = flow_.at<Vec2f>((int)cy, (int)cx)[0];
    float fy = flow_.at<Vec2f>((int)cy, (int)cx)[1];
    mag = sqrt(fx*fx + fy*fy);
  }

  ROS_INFO("flow %.2f, flow percentage: %.2f", mag, 100.*(float)flowCnt/((float)flow_.rows*(float)flow_.cols));

  return mag > MIN_FLOW && (float)flowCnt/((float)flow_.rows*(float)flow_.cols) < 0.10;
}

void AttentionMotion::visualiseFlow(float cx, float cy, float mag, float angle, bool validFlow)
{
  cflow_ = frame_;
  for(int i = 0; i < flow_.rows; i++)
    for(int j = 0; j < flow_.cols; j++)
    {
      float fx = flow_.at<Vec2f>(i, j)[0];
      float fy = flow_.at<Vec2f>(i, j)[1];
      float f = sqrt(fx*fx + fy*fy);
      //if(f > MIN_FLOW)
      {
        f *= 30.;  // scale to be visible in a [0,255] range
        if(f > 255.)
          f = 255.;
        cflow_.at<Vec3b>(i, j) = Vec3b(0, (unsigned char)f, 0);
      }
    }
  if(validFlow)
    circle(cflow_, Point(cx, cy), 5, Scalar(255, 0, 0), 1);
  cv_bridge::CvImagePtr outPtr(new cv_bridge::CvImage);
  outPtr->encoding = "bgr8";
  outPtr->image = cflow_;
  imagePub_.publish(outPtr->toImageMsg());
}

void AttentionMotion::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  static bool running = false;
  bool goAhead = false;

  movingMutex_.lock();
  goAhead = !moving_;
  // another possibility: wait a bit more until camera steady timer has expired
  // leads to an interrupted gaze behaviour, and also does not guarantee that
  // no "crappy" flow images happen.
  //goAhead = cameraSteady_;
  movingMutex_.unlock();

  if(!goAhead)
  {
    running = false;
    return;
  }

  cv_bridge::CvImageConstPtr cvPtr;
  float cx, cy, mag, angle;
  bool haveFlow = false;

  try
  {
    cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  resize(cvPtr->image, frame_, Size(), 1./(float)subsample_, 1./(float)subsample_);
  cvtColor(frame_, gray_, COLOR_BGR2GRAY);
  if(prevgray_.data)
  {
    if(running)
      haveFlow = calculateFlowCenter(cx, cy, mag, angle);
    else
      running = true;
  }
  std::swap(prevgray_, gray_);

  if(haveFlow)
  {
    squirrel_object_perception_msgs::LookAtImagePosition lookSrv;
    // HACK: Here I assume fixed camera parameters for a Kinect/Asus
    lookSrv.request.x = cx*subsample_ - 320.;
    lookSrv.request.y = cy*subsample_ - 240.;
    lookSrv.request.why = "motion";
    ROS_INFO("look at image position %.f %.f", lookSrv.request.x, lookSrv.request.y);
    controllerSrv_.call(lookSrv);
  }

  visualiseFlow(cx, cy, mag, angle, haveFlow);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "attention_motion_node");
  AttentionMotion am;
  am.run();
  exit(0);
}
