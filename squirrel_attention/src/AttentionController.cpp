/**
 * AttentionController.cpp
 *
 * Gets to look to from various attention channels and decides how to move the
 * camera (and robot if necessary).
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#include <cmath>
#include <std_msgs/Float64.h>
#include <squirrel_attention/AttentionController.h>

AttentionController::AttentionController()
{
  // nh_.param("some", var_, defVal);
  panPub_ = nh_.advertise<std_msgs::Float64>("/pan_controller/command", 2);
  tiltPub_ = nh_.advertise<std_msgs::Float64>("/tilt_controller/command", 2);

  lookImageSrv_ = nh_.advertiseService("/attention/look_at_image_position", &AttentionController::lookAtImagePosition, this);
  lookSrv_ = nh_.advertiseService("/attention/look_at_position", &AttentionController::lookAtPosition, this);
  fixateSrv_ =  nh_.advertiseService("/attention/fixate_position", &AttentionController::fixatePosition, this);
  clearSrv_ = nh_.advertiseService("/attention/clear_fixation", &AttentionController::clearFixation, this);
  panStateSub_ = nh_.subscribe("/pan_controller/state", 2, &AttentionController::panStateCallback, this);
  tiltStateSub_ = nh_.subscribe("/tilt_controller/state", 2, &AttentionController::tiltStateCallback, this);
  pan_ = tilt_ = 0.;
}

AttentionController::~AttentionController()
{
}

void AttentionController::run()
{
  ros::spin();
}

void AttentionController::panStateCallback(const dynamixel_msgs::JointState::ConstPtr& panStateMsg)
{
  jointMutex_.lock();
  pan_ = panStateMsg->current_pos;
  jointMutex_.unlock();
}

void AttentionController::tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr& tiltStateMsg)
{
  jointMutex_.lock();
  tilt_ = tiltStateMsg->current_pos;
  jointMutex_.unlock();
}

bool AttentionController::lookAtImagePosition(squirrel_object_perception_msgs::LookAtImagePosition::Request &req,
                                             squirrel_object_perception_msgs::LookAtImagePosition::Response &res)
{
  jointMutex_.lock();
  std_msgs::Float64 panMsg, tiltMsg;
  // HACK: the focal length is hardcoded for the Kinect/Asus
  panMsg.data = pan_ - atan2(req.x, 525);
  tiltMsg.data = tilt_ + atan2(req.y, 525);
  //ROS_INFO("pan/tilt relative move move (deg): %.f %.f", -atan2(req.x, 525)*180./M_PI, atan2(req.y, 525*180./M_PI));
  if(std::isfinite(panMsg.data) && std::isfinite(tiltMsg.data))
  {
    panPub_.publish(panMsg);
    tiltPub_.publish(tiltMsg);
  }
  jointMutex_.unlock();
  return true;
}

bool AttentionController::lookAtPosition(squirrel_object_perception_msgs::LookAtPosition::Request &req,
                                        squirrel_object_perception_msgs::LookAtPosition::Response &res)
{
  
  return true;
}

bool AttentionController::fixatePosition(squirrel_object_perception_msgs::FixatePosition::Request &req,
                                        squirrel_object_perception_msgs::FixatePosition::Response &res)
{
  return true;
}

bool AttentionController::clearFixation(squirrel_object_perception_msgs::ClearFixation::Request &req,
                                       squirrel_object_perception_msgs::ClearFixation::Response &res)
{
  return true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "attention_controller_node");
  AttentionController ac;
  ac.run();
  exit(0);
}
