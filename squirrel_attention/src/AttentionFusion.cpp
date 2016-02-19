/**
 * AttentionFusion.cpp
 *
 * Attends to the legs in the laser data and faces in the camera data.
 * 
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @author Markus Bajones bajones@acin.tuwien.ac.at
 * @date Sept 2015
 */

#include <math.h>
#include <ios>
#include <squirrel_object_perception_msgs/LookAtPosition.h>
#include <squirrel_attention/AttentionFusion.h>
#include <people_msgs/People.h>
#include <tf/LinearMath/Transform.h>

using namespace std;

AttentionFusion::AttentionFusion()
{
  legsSub_ = nh_.subscribe("/laser_person", 2, &AttentionFusion::legsCallback, this);
  legsSub2_ = nh_.subscribe("/leg_persons", 2, &AttentionFusion::legsCallback2, this);
  controllerSrv_ = nh_.serviceClient<squirrel_object_perception_msgs::LookAtPosition>("/attention/look_at_position");
  timer = nh_.createTimer(ros::Duration(5.0), &AttentionFusion::observeTimerCallback, this);
  last_observation_ = ros::Time::now();
}

AttentionFusion::~AttentionFusion()
{
}

void AttentionFusion::run()
{
  ros::spin();
}

void AttentionFusion::observeTimerCallback(const ros::TimerEvent&)
{
  ros::Duration diff;
  observeMutex_.lock();
  diff = ros::Time::now() - last_observation_;
  if (diff > ros::Duration(2.0))
  {
    // rotate robot
    ROS_INFO("Turn camera towards the other person");
    
    squirrel_object_perception_msgs::LookAtPosition srv;
    srv.request.target = next_.point;
    srv.request.why = reason_;
    
    if (controllerSrv_.call(srv))
    {
      ROS_INFO("call service /attention/look_at_position with: %f %f %f", next_.point.x, next_.point.y, next_.point.z);
    }
    else
    {
      ROS_ERROR("Failed to call service /attention/look_at_position");
    }
  }
  else
  {
    // wait a little longer
    ROS_INFO("Do not move the camera just yet.");
  }
  observeMutex_.unlock();
}

void AttentionFusion::legsCallback(const std_msgs::String& msg)
{
  stringstream poss(msg.data);
  float x, y;
  poss >> x >> y;
  ROS_INFO("look at person at %.3f %.3f", x, y);
  observeMutex_.lock();
  observeMutex_.unlock();
}

void AttentionFusion::legsCallback2(const people_msgs::People& msg)
{
  ROS_INFO("legsCallback2");
  for (size_t i = 0; i < msg.people.size(); i++)
  {
    ROS_INFO("possible person at %.3f %.3f", msg.people[i].position.x, msg.people[i].position.y);
    if ((msg.people.size() == 1) && (abs(msg.people[i].position.y) < 0.2))
    {
      ROS_INFO("person right in front of the robot");
      continue;
    }
    geometry_msgs::PointStamped tmp_point;
    tmp_point.header.frame_id = msg.header.frame_id;
    tmp_point.header.stamp = msg.header.stamp;
    tmp_point.point.x = msg.people[i].position.x;
    tmp_point.point.y = msg.people[i].position.y;
    tmp_point.point.z = msg.people[i].position.z;
    try{
      // service expects a geometry_msgs::Point in base_link instead of kinect_rgb_optical_frame
      listener_.waitForTransform("base_link", "hokuyo_link", ros::Time::now(), ros::Duration(3.0));
      listener_.transformPoint("base_link", tmp_point, next_); 
      ROS_INFO("hokuyo_link: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
	       tmp_point.point.x, tmp_point.point.y, tmp_point.point.z, next_.point.x, next_.point.y, next_.point.z, next_.header.stamp.toSec());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    ROS_INFO("look at person at %.3f %.3f", next_.point.x, next_.point.y);
    observeMutex_.lock();
    reason_ = "leg detection";
    observeMutex_.unlock();
  }
}

void AttentionFusion::robotInFovCallback(const std_msgs::String& msg)
{
  stringstream fovs(msg.data);
  std::string fov;
  fovs >> fov;
  ROS_INFO("Robot in FoV of a user");
  last_observation_ = ros::Time::now();
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "attention_legs_node");
  AttentionFusion al;
  al.run();
  exit(0);
}
