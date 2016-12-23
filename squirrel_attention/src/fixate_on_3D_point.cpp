#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <dynamixel_controllers/SetRelativePosition.h>
#include <geometry_msgs/PointStamped.h>


double rad2deg(double in)
{
	return in * 180.0 / M_PI;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fixate_on_3D_point_node");
	ros::NodeHandle nh;

	ros::ServiceClient pan_client = nh.serviceClient<dynamixel_controllers::SetRelativePosition>("/pan_controller/set_relative_position", true);
	ros::ServiceClient tilt_client = nh.serviceClient<dynamixel_controllers::SetRelativePosition>("/tilt_controller/set_relative_position", true);

	tf::TransformListener listener;
	geometry_msgs::PointStamped point, pan, tilt;

	point.header.stamp = ros::Time::now();
	point.header.frame_id = "/map";
	point.point.x = 1.0;
	point.point.y = 1.0;
	point.point.z = 0.0;


	while(nh.ok())
	{
		tf::StampedTransform transform;
		try 
		{
			ros::Time now = ros::Time(0);
    			listener.waitForTransform("/pan_link", "/map", now, ros::Duration(1.0));
    			listener.waitForTransform("/tilt_link", "/map", now, ros::Duration(1.0));
			listener.transformPoint("/pan_link", now,  point, "/map",  pan);
			listener.transformPoint("/tilt_link", now,  point, "/map",  tilt);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		double rel_pan, rel_tilt;
		rel_pan = atan2(pan.point.y, pan.point.x);
		rel_tilt = -atan2(tilt.point.y, tilt.point.x);

		dynamixel_controllers::SetRelativePosition srv;
  		srv.request.position = rel_pan;

		double start;
		start = ros::Time::now().toSec();
  		if (pan_client.call(srv))
  		{
  		  ROS_DEBUG("Service /pan_controller/set_relative_position called succesfully");
  		}
  		else
  		{
  		  ROS_ERROR("Failed to call service /pan_controller/set_relative_position");
  		}	
		double end;
		end = ros::Time::now().toSec();
		ROS_INFO("Service call took %f seconds", end - start); 
		start = ros::Time::now().toSec();
  		srv.request.position = rel_tilt;
  		if (tilt_client.call(srv))
  		{
  		  ROS_DEBUG("Service /tilt_controller/set_relative_position called succesfully");
  		}
  		else
  		{
  		  ROS_ERROR("Failed to call service /tilt_controller/set_relative_position");
  		}	
		end = ros::Time::now().toSec();
		ROS_INFO("Service call took %f seconds", end - start); 

		//ROS_DEBUG("Need to move pan by %f relative degree", rad2deg(rel_pan));
		//ROS_DEBUG("Need to move tilt by %f relative degree", rad2deg(rel_tilt));
		//ROS_DEBUG("--------------------------------------------------");	

		
		
	}
}
