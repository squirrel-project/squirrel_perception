
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

void AttentionMotion::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  const int scale = 8;
  const float MIN_FLOW = 8./(float)scale;
  static bool havePrev = false;

  bool goAhead = false;
  int flowCnt = 0;

  movingMutex_.lock();
  goAhead = !moving_; //cameraSteady_;
  movingMutex_.unlock();

  if(!goAhead)
  {
    havePrev = false;
    return;
  }

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

  resize(cvPtr->image, frame, Size(), 1./(float)scale, 1./(float)scale);
  cvtColor(frame, gray, COLOR_BGR2GRAY);

  if(prevgray.data)
  {
    // NOTE: is there a more efficient way to zero an exiting matrix?
    flow = Mat::zeros(gray.rows, gray.cols, CV_32FC2);
    if(havePrev)
      calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 11, 3, 5, 1.2, 0);
    else
      havePrev = true;
  }
  std::swap(prevgray, gray);

  int iMax = -1, jMax = -1;
  float flowMax = 0.;
  float cx = 0.;
  float cy = 0.;
  float sum = 0.;
  for(int i = 0; i < flow.rows; i++)
    for(int j = 0; j < flow.cols; j++)
    {
      float fx = flow.at<Vec2f>(i, j)[0];
      float fy = flow.at<Vec2f>(i, j)[1];
      float f = sqrt(fx*fx + fy*fy);
      if(f > MIN_FLOW)
        flowCnt++;
      sum += f;
      cx += f*(float)j;
      cy += f*(float)i;
      /*if(f > MIN_FLOW && f > flowMax)
      {
        flowMax = f;
        iMax = i;
        jMax = j;
      }*/
    }
  if(isnormal(sum))
  {
    cx /= sum;
    cy /= sum;
    float fx = flow.at<Vec2f>((int)cy, (int)cx)[0];
    float fy = flow.at<Vec2f>((int)cy, (int)cx)[1];
    flowMax = sqrt(fx*fx + fy*fy);
    if(flowMax <= MIN_FLOW)
      flowMax = 0.;
  }

  ROS_INFO("max flow %.2f, flow percentage: %.2f", flowMax, 100.*(float)flowCnt/((float)flow.rows*(float)flow.cols));

  if(flowMax > 0. && (float)flowCnt/((float)flow.rows*(float)flow.cols) < 0.10) 
  {
    squirrel_object_perception_msgs::LookAtImagePosition lookSrv;
    lookSrv.request.x = cx*scale - 320.;
    lookSrv.request.y = cy*scale - 240.;
    ROS_INFO("look at image position %.f %.f", lookSrv.request.x, lookSrv.request.y);
    lookSrv.request.why = "motion";
    controllerSrv_.call(lookSrv);
  }

  // visualisation
  //cflow.create(gray.rows, gray.cols,CV_8UC3);
  cflow = frame;
  for(int i = 0; i < flow.rows; i++)
    for(int j = 0; j < flow.cols; j++)
    {
      float fx = flow.at<Vec2f>(i, j)[0];
      float fy = flow.at<Vec2f>(i, j)[1];
      float f = sqrt(fx*fx + fy*fy);
      if(f > MIN_FLOW)
      {
        f *= 30.;  // scale to be visible in a [0,255] range
        if(f > 255.)
          f = 255.;
        cflow.at<Vec3b>(i, j) = Vec3b(0, (unsigned char)f, 0);
      }
    }
  if(flowMax > 0.&& (float)flowCnt/((float)flow.rows*(float)flow.cols) < 0.10)
    circle(cflow, Point(cx, cy), 5, Scalar(255, 0, 0), 1);
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
