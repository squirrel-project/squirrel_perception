#! /usr/bin/env python

import rospy
import actionlib
import squirrel_object_perception_msgs.msg
from squirrel_object_perception_msgs.srv import ObjectRecognizer
from sensor_msgs.msg import PointCloud2


class LookForObjectAction(object):
    # create messages that are used to publish feedback/result
    _feedback = squirrel_object_perception_msgs.msg.LookForObjectFeedback()
    _result = squirrel_object_perception_msgs.msg.LookForObjectResult()
    _point_cloud = None
    _objects = None

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            squirrel_object_perception_msgs.msg.LookForObjectAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        self.recognizer = rospy.ServiceProxy(
            'mp_recognition', ObjectRecognizer)

    def set_publish_feedback(self, phase, status, percent):
        self._feedback.current_phase = phase
        self._feedback.current_status = status
        self._feedback.percent_completed = percent
        self._as.publish_feedback(self._feedback)
        return

    def execute_cb(self, goal):
        # initialize feedback
        self.set_publish_feedback('init', 'done', 5)

        # start executing the action
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return

        # Start getting data from /camera/depth_registered/points
        # Set feedback to data acquisition succeeded and set percentage
        self._point_cloud = rospy.wait_for_message(
            '/camera/depth_registered/points',
            PointCloud2,
            timeout=5
        )
        if not self._point_cloud:
            self.set_publish_feedback('receive_data', 'failed', 6)
            self._result.result_status = 'aborted'
            rospy.loginfo('Aborted' % self._action_name)
            self._as.set_aborted(self._result)
            return

        # publish the feedback
        self.set_publish_feedback('receive_data', 'done', 10)
        self.set_publish_feedback('recognition', 'started', 11)
        # call Aitor's recognition service
        try:
            rospy.wait_for_service('mp_recognition', timeout=5)
            self.set_publish_feedback('recognition', 'done', 50)
        except rospy.ROSException as e:
            self.set_publish_feedback('recognition', 'service call failed', 11)
            rospy.logdebug('mp_recognition: %s' % str(e))

        # Start attention, segmentation classification pipeline
        self.set_publish_feedback('attention', 'started', 51)
        self.set_publish_feedback('attention', 'done', 60)
        self.set_publish_feedback('segmentation', 'started', 61)
        self.set_publish_feedback('segmentation', 'done', 75)
        self.set_publish_feedback('classification', 'started', 76)
        self.set_publish_feedback('classification', 'done', 97)

        # check for recognized or classified objects. Push them into database
        if not self._objects:
            self.set_publish_feedback('Pipeline results', 'empty', 99)
            self._result.result_status = 'aborted'
            rospy.loginfo('%s: Aborted' % self._action_name)
            self._as.set_aborted(self._result)
            return

        self.set_publish_feedback('database_update', 'started', 98)
        self.set_publish_feedback('database_update', 'done', 99)

        self.set_publish_feedback('finish', 'done', 100)
        self._result.result_status = 'success'
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
        return

if __name__ == '__main__':
    rospy.init_node('squirrel_object_perception')
    LookForObjectAction(rospy.get_name())
    rospy.spin()
