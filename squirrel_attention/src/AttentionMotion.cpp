
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <squirrel_attention/AttentionMotion.h>

using namespace cv;
using namespace std;

static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}

AttentionMotion::AttentionMotion()
: it_(nh_)
{
  imageSub_= it_.subscribe("/camera/rgb/image_raw", 1, &AttentionMotion::imageCallback, this);
  imagePub_ = it_.advertise("/attention_motion/saliency", 1);
}

void AttentionMotion::run()
{
  ros::spin();
}

AttentionMotion::~AttentionMotion()
{
}

void AttentionMotion::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  const float MIN_FLOW = 1.;

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
  {
    calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
    // visualisation
    cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
    drawOptFlowMap(flow, cflow, 8, 1.5, Scalar(0, 255, 0));
  }
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

  // visualisation
  if(flowMax > 0.)
    circle(cflow, Point(jMax,iMax), 5, Scalar(255, 0, 0), -1);
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
