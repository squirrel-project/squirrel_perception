#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "std_msgs/Float64.h"
#include <dynamixel_controllers/SetRelativePosition.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <geometry_msgs/PointStamped.h>

double rad2deg(double in)
{
  return in * 180.0 / M_PI;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fixate_on_3D_point_node");
  ros::NodeHandle nh;

  ros::Publisher pan_pub = nh.advertise<std_msgs::Float64>("/neck_pan_controller/rel_command", 0, false);
  ros::Publisher tilt_pub = nh.advertise<std_msgs::Float64>("/neck_tilt_controller/rel_command", 0, false);

  tf::TransformListener listener;
  geometry_msgs::PointStamped point, pan, tilt;

  point.header.stamp = ros::Time::now();
  point.header.frame_id = "/map";
  point.point.x = 1.0;
  point.point.y = 1.0;
  point.point.z = 0.0;

  ros::Rate r(25); // 25 hz
  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      ros::Time now = ros::Time(0);
      listener.waitForTransform("/pan_link", "/map", now, ros::Duration(3.0));
      listener.waitForTransform("/tilt_link", "/map", now, ros::Duration(3.0));
      listener.transformPoint("/pan_link", now, point, "/map", pan);
      listener.transformPoint("/tilt_link", now, point, "/map", tilt);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    double rel_pan, rel_tilt;
    std_msgs::Float64 panMsg, tiltMsg;
    rel_pan = atan2(pan.point.y, pan.point.x);
    rel_tilt = -atan2(tilt.point.y, tilt.point.x);

    if (fabs(rel_pan) > 0.001)
    {
        panMsg.data = rel_pan;
        pan_pub.publish(panMsg);
    }
    else
    {
        ROS_INFO("rel_pan: %f", rel_pan);
    }
    if (fabs(rel_tilt) > 0.001)
    {
        tiltMsg.data = rel_tilt;
        tilt_pub.publish(tiltMsg);
    }
    else
    {
        ROS_INFO("rel_tilt: %f", rel_tilt);
    }
    ros::spinOnce();
    r.sleep();
  }
}
