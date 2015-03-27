#!/usr/bin/env python
#
# Implements the whole look for objects action:
#
# author: Markus Bajones, Michael Zillich
#

import rospy
from roslib import message
import actionlib
import dynamic_reconfigure.server
from squirrel_object_perception.cfg import \
    squirrel_look_for_objectsConfig as ConfigType
from squirrel_object_perception_msgs.srv import \
    Recognize, SegmentInit, SegmentOnce, GetSaliencyItti,\
    SegmentVisualizationInit, SegmentVisualizationOnce, Classify,\
    SegmentsToObjects
from squirrel_object_perception_msgs.msg import \
    LookForObjectsAction, LookForObjectsFeedback, LookForObjectsResult
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from squirrel_planning_knowledge_msgs.srv import AddObjectService, \
    AddObjectServiceRequest, UpdateObjectService, UpdateObjectServiceRequest


class Object:
    _id = None
    _category = None
    _pose = None
    _points = []
    _point_indices = []
  
class SquirrelLookForObjectsImpl:
    _feedback = LookForObjectsFeedback()
    _result = LookForObjectsResult()
    _point_cloud = None
    _objects = []
    _saliency_map = None
    _segment_result = []
    _id_cnt = 1

    def __init__(self):
        pass

    def configure(self):
        pass

    def update(self):
        pass

    def get_unique_object_id(self):
        id = self._id_cnt
        self._id_cnt = self._id_cnt + 1
        return "object" + str(id)

    def set_publish_feedback(self, phase, status, percent):
        self._feedback.current_phase = phase
        self._feedback.current_status = status
        self._feedback.percent_completed = percent
        self.as_squirrel_object_perception.publish_feedback(self._feedback)
        print(self._feedback)
        return

    def do_recognition(self):
        recognizer = rospy.ServiceProxy('mp_recognition', ObjectRecognizer)
        result = None
        self.set_publish_feedback('recognition', 'started', 11)
        try:
            rospy.wait_for_service('mp_recognition', timeout=5)
            result = recognizer(self._point_cloud)
            self.set_publish_feedback('recognition', 'done', 50)
        except (rospy.ROSException, rospy.ServiceException):
            self.set_publish_feedback('recognition', 'service call failed', 11)
            rospy.logdebug('mp_recognition: failed')
        return result

    def get_saliency_map(self):
        do_saliency = rospy.ServiceProxy(
            '/squirrel_attention_itti', GetSaliencyItti)
        try:
            rospy.wait_for_service('/squirrel_attention_itti', timeout=5)
            # NOTE: loccation type 0 = CENTER
            result = do_saliency(self._point_cloud)
            self._saliency_map = result.saliency_map
            self.set_publish_feedback('attention', 'done', 50)
        except (rospy.ROSException, rospy.ServiceException):
            self.set_publish_feedback('attention', 'service call failed', 11)
            rospy.logdebug('attention failed')

    def setup_visualization(self):
        init_segmenter = rospy.ServiceProxy(
            'squirrel_segmentation_visualization_init',
            SegmentVisualizationInit)
        try:
            rospy.wait_for_service(
                'squirrel_segmentation_visualization_init', timeout=5)
            init_segmenter(self._point_cloud, self._saliency_map)
            self.set_publish_feedback('setup_visualization', 'done', 50)
            return True
        except (rospy.ROSException, rospy.ServiceException):
            self.set_publish_feedback('setup_visualization',
                                      'service call failed', 11)
            rospy.logdebug('setup_visualization failed')
            return False

    def setup_segmentation(self):
        init_segmenter = rospy.ServiceProxy(
            'squirrel_segmentation_incremental_init', SegmentInit)
        try:
            rospy.wait_for_service(
                'squirrel_segmentation_incremental_init', timeout=5)
            init_segmenter(self._point_cloud, self._saliency_map)
            self.set_publish_feedback('setup_segmentation', 'done', 50)
            return True
        except (rospy.ROSException, rospy.ServiceException):
            self.set_publish_feedback('setup_segmentation',
                                      'service call failed', 11)
            rospy.logdebug('setup_segmentation failed')
            return False

    def run_segmenter_once(self):
        do_segment = rospy.ServiceProxy(
            'squirrel_segmentation_incremental_once', SegmentOnce)
        do_objects = rospy.ServiceProxy(
            'squirrel_segments_to_objects', SegmentsToObjects)
        try:
            rospy.wait_for_service(
                'squirrel_segmentation_incremental_once', timeout=5)
            seg_result = do_segment()
            ## START DEBUG PRINT
            #data_out = pc2.read_points(self._point_cloud, field_names=None, skip_nans=True, uvs=[[self._point_cloud.width / 2, self._point_cloud.height / 2]])
            #int_data = next(data_out)
            #rospy.loginfo("int_data_after: " + str(int_data))
            ## END
            rospy.wait_for_service(
                'squirrel_segments_to_objects', timeout=5)
            #obj_result = do_objects(self._point_cloud, seg_result.clusters_indices)
            #print "found " + str(len(obj_result.poses)) + " object(s)"
            print "found " + str(len(seg_result.poses)) + " object(s)"
            #print "object 0 has " + str(len(seg_result.clusters_indices[0].data)) + " points"
            # segment once always returns 1 or 0 objects
            if len(seg_result.poses) > 0:
              print "appending object 0"
              obj = Object()
              obj._id = self.get_unique_object_id()
              obj._category = "thing"
              obj._point_indices = seg_result.clusters_indices[0]
              obj._points = seg_result.points[0]
              obj._pose = seg_result.poses[0]
              self._objects.append(obj)
            self.set_publish_feedback('segment_once', 'done', 50)
        except (rospy.ROSException, rospy.ServiceException):
            self.set_publish_feedback('segment_once',
                                      'service call failed', 11)
            rospy.logdebug('segment_once failed')

    def run_visualization_once(self):
        do_visualize = rospy.ServiceProxy(
            'squirrel_segmentation_visualization_once',
            SegmentVisualizationOnce)
        try:
            rospy.wait_for_service(
                'squirrel_segmentation_visualization_once', timeout=5)
            result = do_visualize(self._segment_result[-1].clusters_indices)
            self.set_publish_feedback('segment_visualization_once', 'done', 50)
        except (rospy.ROSException, rospy.ServiceException):
            self.set_publish_feedback('segment_visualization_once',
                                      'service call failed', 11)
            rospy.logdebug('segment_visualization_once failed')

    def most_confident_class(self, classification):
        max_conf = max(classification.confidence)
        max_index = classification.confidence.index(max_conf)
        return classification.class_type[max_index].data

    def run_classifier(self):
        do_classify = rospy.ServiceProxy(
            'squirrel_classify', Classify)
        try:
            rospy.wait_for_service(
                'squirrel_classify', timeout=5)
            # classify the last segmented object
            points_indices = []
            points_indices.append(self._objects[-1]._point_indices)
            result = do_classify(self._point_cloud, points_indices)
            # NOTE: Classify outputs a list of classifications. In our case this should
            # be of size 1, as we only input one cluster.
            if len(result.class_results) == 1:
                # NOTE: At this point we just take the most confident class and ignore the rest.
                # At a later stage we might use the actual probability distribution over class labels.
                self._objects[-1]._category = self.most_confident_class(result.class_results[0])
            else:
                rospy.logdebug('classification error: one object in, more than one (or 0) objects out')
            self.set_publish_feedback('classification', 'done', 50)
        except (rospy.ROSException, rospy.ServiceException):
            self.set_publish_feedback('classification',
                                      'service call failed', 11)
            rospy.logdebug('classification failed')

    def add_object_to_db(self, obj):
        add_object = rospy.ServiceProxy('/kcl_rosplan/add_object', AddObjectService)
        try:
            rospy.wait_for_service('/kcl_rosplan/add_object', timeout=3)
            request = AddObjectServiceRequest()
            request.id = obj._id
            request.category = obj._category
            request.pose = obj._pose
            request.cloud = obj._points
            resp = add_object(request)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            resp = False
        return resp

    def update_object_in_db(self, obj):
        update_object = rospy.ServiceProxy('/kcl_rosplan/update_object', UpdateObjectService)
        try:
            rospy.wait_for_service('/kcl_rosplan/update_object')
            request = UpdateObjectServiceRequest()
            request.id = obj._id
            request.category = obj._category
            request.pose = obj._pose
            request.cloud = obj._points
            resp = update_object(request)
            return resp
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            resp = False
        return resp

    def execute_squirrel_object_perception_cb(self, goal):
        # initialize feedback
        self.set_publish_feedback('init', 'done', 5)

        # clear results
        self._objects = []

        # check that preempt has not been requested by the client
        if self.as_squirrel_object_perception.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self.as_squirrel_object_perception.set_preempted()
            return

        # Start getting data from /kinect/depth_registered/points
        # Set feedback to data acquisition succeeded and set percentage
        try:
            self._point_cloud = rospy.wait_for_message(
                '/kinect/depth_registered/points',
                PointCloud2,
                timeout=5
            )
            self.set_publish_feedback('receive_data', 'done', 10)
        except rospy.ROSException:
            self.set_publish_feedback('receive_data', 'failed', 6)
            self._result.result_status = 'aborted'
            self.as_squirrel_object_perception.set_aborted(self._result)
            rospy.logdebug('receive_data failed')
            return

        # call Aitor's recognition service
        # TODO: for now skip this
        #self.do_recognition()
        # Start attention, segmentation classification pipeline
        self.set_publish_feedback('attention', 'started', 51)
        self.get_saliency_map()
        self.set_publish_feedback('attention', 'done', 60)
        self.set_publish_feedback('segmentation', 'started', 61)
        self.set_publish_feedback('classification', 'started', 76)
        self.setup_segmentation()
        #self.setup_visualization()
        # TODO: find a reasonable number of times to run here
        # for now: just once
        for i in xrange(1, 2):
            self.run_segmenter_once()
            #self.run_visualization_once()
            #self.run_classifier() TODO: this fails and hangs
        self.set_publish_feedback('segmentation', 'done', 75)
        self.set_publish_feedback('classification', 'done', 97)

        # check for recognized or classified objects. Push them into database
        if not self._objects:
            self.set_publish_feedback('Pipeline results', 'empty', 99)
            self._result.result_status = 'aborted'
            rospy.loginfo('%s: Aborted' % self._action_name)
            self.as_squirrel_object_perception.set_aborted(self._result)
            return

        self.set_publish_feedback('database_update', 'started', 98)
        for obj in self._objects:
            # TODO: the semantics of this check vs. explore is not clear yet!
            #if goal.look_for_object == 0:# check
            #    if not goal.category.lower() == category.lower():
            #        break
            #    self.update_object_in_db(category)
            #elif goal.look_for_object == 1: #explore
            #    self.add_object_to_db(category)
            self.add_object_to_db(obj)
        self.set_publish_feedback('database_update', 'done', 99)

        self.set_publish_feedback('finish', 'done', 100)
        self._result.result_status = 'success'
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self.as_squirrel_object_perception.set_succeeded(self._result)
        return


class SquirrelLookForObjects:
    def __init__(self):
        self.impl = SquirrelLookForObjectsImpl()
        self.impl._action_name = 'look_for_objects'
        self_dynrecon_server = dynamic_reconfigure.server.Server(
            ConfigType, self.config_callback)
        self.impl.as_squirrel_object_perception = actionlib.SimpleActionServer(
            'look_for_objects',
            LookForObjectsAction,
            execute_cb=self.impl.execute_squirrel_object_perception_cb,
            auto_start=False)
        self.impl.as_squirrel_object_perception.start()

    def run(self):
        self.impl.update()

    def config_callback(self, config, level):
        return config

if __name__ == "__main__":
    try:
        rospy.init_node('squirrel_look_for_objects')
        n = SquirrelLookForObjects()
        n.impl.configure()
        while not rospy.is_shutdown():
            n.run()
            rospy.sleep(10.0)

    except rospy.ROSInterruptException:
        print "Exit"
