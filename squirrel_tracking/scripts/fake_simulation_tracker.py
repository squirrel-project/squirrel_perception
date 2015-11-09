#!/usr/bin/python
#
# Tracks objects in the gazebo simulator by simple reading their pose values.
# This is adapted from:
# https://github.com/ipa320/cob_object_perception/blob/hydro_dev/cob_object_detection_fake/ros/scripts/object_detection_fake
#
# author: Michael Zillich
# date: Nov 2015

import rospy
import random
import tf
from tf.transformations import *
from numpy import *
from gazebo_msgs.srv import *
from squirrel_object_perception_msgs.srv import *

# HACK: just a dummy function to be able to define timer blow
# This is an ugly solution, but ok for now.
def nix(timer_event):
	return


rospy.init_node('object_tracker')
tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()
tracked_object_id = ""
timer = rospy.Timer(rospy.Duration(0.03), nix, True)


def track_object(timer_event):
	global tracked_object_id
	global tf_listener
	global tf_broadcaster

	# get world properties from gazebo, get all objects in gazebo (including ground_plance etc.)
	srv_get_world_properties = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
	res_world = srv_get_world_properties()

	# check if name is an object in gazebo, otherwise return empty response
	if tracked_object_id not in res_world.model_names:
		rospy.logerr("object %s not available in gazebo, available objects are %s", tracked_object_id, res_world.model_names)
		return

	# get model state
	srv_get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	res_model = srv_get_model_state(tracked_object_id,'robot::tilt_link')
	trans_o_h = (res_model.pose.position.x, res_model.pose.position.y, res_model.pose.position.z)
	rot_o_h = (res_model.pose.orientation.x, res_model.pose.orientation.y, res_model.pose.orientation.z, res_model.pose.orientation.w)

	frame_o_h = quaternion_matrix(rot_o_h)
	frame_o_h[0][3] = trans_o_h[0]
	frame_o_h[1][3] = trans_o_h[1]
	frame_o_h[2][3] = trans_o_h[2]

	tf_listener.waitForTransform('/kinect_rgb_optical_frame', '/tilt_link', rospy.Time(0), rospy.Duration(1))
	(trans_h_l, rot_h_l) = tf_listener.lookupTransform('/kinect_rgb_optical_frame', '/tilt_link', rospy.Time(0))

	frame_h_l = quaternion_matrix(rot_h_l)
	frame_h_l[0][3] = trans_h_l[0]
	frame_h_l[1][3] = trans_h_l[1]
	frame_h_l[2][3] = trans_h_l[2]

	# Transform
	frame_o_l = numpy.dot(frame_h_l, frame_o_h)
	rot_o_l = quaternion_from_matrix(frame_o_l)
	tmp_mat = hsplit(frame_o_l, [3])
	trans_o_l = vsplit(tmp_mat[1], [3])[0]

	# this was for debugging
	# print "x,y,z: ", trans_o_l[0], trans_o_l[1], trans_o_l[2]

	tf_broadcaster.sendTransform(trans_o_l, rot_o_l, rospy.Time.now(), tracked_object_id, "/kinect_rgb_optical_frame")


def start_tracking(req):
	global tracked_object_id
	global timer;

	resp = StartObjectTrackingResponse
	if tracked_object_id == "":
		tracked_object_id = req.object_id.data
		# start tracking with ~30 Hz
		timer = rospy.Timer(rospy.Duration(0.03), track_object)
		rospy.loginfo("start_tracking: %s", tracked_object_id)
	else:
		rospy.logerr("start_tracking: I am already tracking an object, can only do one at a time.");
	return resp;


def stop_tracking(req):
	global tracked_object_id
	global timer

	resp = StopObjectTrackingResponse
	if not tracked_object_id == "":
		tracked_object_id = ""
		timer.shutdown()
		rospy.loginfo("stopTracking: stopped");
	else:
		rospy.logerr("stopTracking: currently not tracking an object");
	return resp;


if __name__ == "__main__":
	rospy.sleep(2)
	s1 = rospy.Service("/squirrel_start_object_tracking", StartObjectTracking, start_tracking);
	s2 = rospy.Service("/squirrel_stop_object_tracking", StopObjectTracking, stop_tracking);
	rospy.loginfo("Ready to get service calls...");
	rospy.spin()

