#!/usr/bin/env python

import rospy
import actionlib
import dynamic_reconfigure.server
from squirrel_object_perception.cfg import \
    squirrel_look_for_objectsConfig as ConfigType
from squirrel_object_perception_msgs.srv import \
    Recognize, SegmentInit, SegmentOnce, GetSaliency3DSymmetry,\
    SegmentVisualizationInit, SegmentVisualizationOnce, Classify
from squirrel_object_perception_msgs.msg import \
    LookForObjectsAction, LookForObjectsFeedback, LookForObjectsResult
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

    def get_unique_object_id():
        id = _id_cnt
        _id_cnt = _id_cnt + 1
        return str(id)

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
        get_saliency_map = rospy.ServiceProxy(
            '/squirrel_attention_3Dsymmetry', GetSaliency3DSymmetry)
        try:
            rospy.wait_for_service('/squirrel_attention_3Dsymmetry', timeout=5)
            result = get_saliency_map(self._point_cloud)
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
            rospy.wait_for_service(
                'squirrel_segments_to_objects', timeout=5)
            obj_result = do_objects(self._point_cloud, seg_result.clusters_indices)
            obj = Object()
            obj._id = self.get_unique_object_id()
            obj._category = "thing"
            obj._point_indices = seg_result.clusters_indices
            obj._points = obj_result.points
            obj._pose = obj_result.pose
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

    def most_confident_class(classification):
        max_conf = max(classification.confidence)
        max_index = object.classification.confidence.index(max_conf)
        return classification.class_type[max_index].data

    def run_classifier(self):
        do_classify = rospy.ServiceProxy(
            'squirrel_classify', Classify)
        try:
            rospy.wait_for_service(
                'squirrel_classify', timeout=5)
            # classify the last segmented object
            result = do_classify(self._point_cloud,
                                 self._objects[-1].point_indices)
            # NOTE: Classify outputs a list of classifications. In our case this should
            # be of size 1, as we only input one cluster.
            if len(res) == 1:
                # NOTE: At this point we just take the most confident class and ignore the rest.
                #At a later stage we might use the actual probability distribution over class labels.
                obj._category = most_confident_class(result.class_results)
            else:
                rospy.logdebug('classification error: one object in, more than one (or 0) objects out')
            print(result)
            self.set_publish_feedback('classification', 'done', 50)
        except (rospy.ROSException, rospy.ServiceException):
            self.set_publish_feedback('classification',
                                      'service call failed', 11)
            rospy.logdebug('classification failed')

    def add_object_to_db(self, category):
        rospy.wait_for_service('/kcl_rosplan/add_object')
        new_object = rospy.ServiceProxy('/kcl_rosplan/add_object', AddObjectService, timeout=3)
        request = AddObjectServiceRequest()
        request.id = category
        request.category = category
        request.pose = None
        request.cloud = None
        try:
          resp = new_object(request)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            resp = False
        return resp

    def update_object_in_db(self, category):
        rospy.wait_for_service('/kcl_rosplan/update_object')
        update_object = rospy.ServiceProxy('/kcl_rosplan/update_object', UpdateObjectService, timeout=3)
        request = UpdateObjectServiceRequest()
        request.id = category
        request.category = category
        request.pose = None
        request.cloud = None
        try:
          resp = update_object(request)
          return resp
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            resp = False
        return resp

    def execute_squirrel_object_perception_cb(self, goal):
        # initialize feedback
        self.set_publish_feedback('init', 'done', 5)

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
        self.do_recognition()
        # Start attention, segmentation classification pipeline
        self.set_publish_feedback('attention', 'started', 51)
        self.get_saliency_map()
        self.set_publish_feedback('attention', 'done', 60)
        self.set_publish_feedback('segmentation', 'started', 61)
        self.set_publish_feedback('classification', 'started', 76)
        self.setup_segmentation()
        self.setup_visualization()
        for i in xrange(1, 5):
            self.run_segmenter_once()
            self.run_visualization_once()
            self.run_classifier()
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
        for object in self._objects:
            # TODO: the semantics of this check vs. explore is not clear yet!
            if goal.look_for_object == 0:# check
                if not goal.category.lower() == category.lower():
                    break
                self.update_object_in_db(category)
            elif goal.look_for_object == 1: #explore
                self.add_object_to_db(category)
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
