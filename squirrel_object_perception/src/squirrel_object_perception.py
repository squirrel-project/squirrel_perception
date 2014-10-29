#! /usr/bin/env python

import rospy
import actionlib
import squirrel_object_perception_msgs.msg
from sensor_msgs.msg import PointCloud2


class LookForObjectAction(object):
    # create messages that are used to publish feedback/result
    _feedback = squirrel_object_perception_msgs.msg.LookForObjectFeedback()
    _result = squirrel_object_perception_msgs.msg.LookForObjectResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            squirrel_object_perception_msgs.msg.LookForObjectAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        self.point_cloud = None

    def set_publish_feedback(self, phase, status, percent):
        self._feedback.current_phase = phase
        self._feedback.current_status = status
        self._feedback.percent_completed = percent
        self._as.publish_feedback(self._feedback)
        return

    def execute_cb(self, goal):
        # helper variables
        success = True

        # initialize feedback
        self.set_publish_feedback('init', 'done', 5)

        # publish info to the console for the user
        rospy.loginfo('Executing. Acquired pointcloud data.\
                      Start recognition service call')

        # start executing the action
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            return
            # set result. Stop excecution

        # Start getting data from /camera/depth_registered/points
        # Set feedback to data acquisition succeeded and set percentage
        cloud_sub = rospy.Subscriber(
            # TODO: change topic name to cloud_in and do a remap
            '/camera/depth_registered/points',
            PointCloud2,
            callback=self.pointcloud_cb
        )
        cloud_sub.unregister()

        # publish the feedback
        self.set_publish_feedback('receive_data', 'done', 10)
        self.set_publish_feedback('recognition', 'started', 11)
        # call Aitor's recognition service
        # check output.
        # set feedback and percentage
        self.set_publish_feedback('recognition', 'done', 50)

        # Call Kate's segmentation and Walter's classification
        # check output
        # set feedback and percentage
        self.set_publish_feedback('attention', 'started', 51)
        self.set_publish_feedback('attention', 'done', 60)
        self.set_publish_feedback('segmentation', 'started', 61)
        self.set_publish_feedback('segmentation', 'done', 75)
        self.set_publish_feedback('classification', 'started', 76)
        self.set_publish_feedback('classification', 'done', 95)
        self.set_publish_feedback('database_update', 'started', 96)
        self.set_publish_feedback('database_update', 'done', 97)
        self.set_publish_feedback('cleanup', 'started', 98)
        success = True
        self.set_publish_feedback('cleanup', 'done', 99)

        if success:
            self.set_publish_feedback('finish', 'done', 100)
            self._result.result_status = 'success'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def pointcloud_cb(self, data):
        self.point_cloud = data
        return


if __name__ == '__main__':
    rospy.init_node('squirrel_object_perception')
    LookForObjectAction(rospy.get_name())
    rospy.spin()
