#!/usr/bin/env python
#
# Pipeline:
# get point cloud
# send to haf_grasping
# transform result to pose in /odom frame
# send pose to moveit motion planning and execution
#
# author: Markus Bajones
#

import rospy
import actionlib
import tf
import dynamic_reconfigure.server
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Vector3, PoseStamped
from sensor_msgs.msg import PointCloud2
#from squirrel_object_perception.cfg import \
#    squirrel_grasp_objectsConfig as ConfigType
from squirrel_object_perception_msgs.msg import \
    GraspObjectAction, GraspObjectFeedback, GraspObjectResult
from haf_grasping.msg import CalcGraspPointsServerAction, CalcGraspPointsServerGoal
from kclhand_control.srv import graspCurrent, graspPreparation
import moveit_commander 


class SquirrelGraspObjectImpl:
    _feedback = GraspObjectFeedback()
    _result = GraspObjectResult()
    _grasp_pose = Pose()
    _graspsearchcenter = Point()
    _approach_vector = Vector3()
    def __init__(self):
        tmp_graspsearchcenter = rospy.get_param('grasp_search_center')
        self._graspsearchcenter.x = tmp_graspsearchcenter[0]
        self._graspsearchcenter.y = tmp_graspsearchcenter[1]
        self._graspsearchcenter.z = tmp_graspsearchcenter[2]
        tmp_approach_vector = rospy.get_param('gripper_approach_vector')
        self._approach_vector.x = tmp_approach_vector[0]
        self._approach_vector.y = tmp_approach_vector[1]
        self._approach_vector.z = tmp_approach_vector[2]
        self._grasp_search_size_x = rospy.get_param('grasp_search_size_x')
        self._grasp_search_size_y = rospy.get_param('grasp_search_size_y')
        self._grasp_calculation_time_max = rospy.Duration.from_sec(40)
        self._show_only_best_grasp = False
        self._gripper_opening_width = 1
        self._closeFinger = rospy.ServiceProxy('hand_controller/closeFinger', graspCurrent)
        self._openFinger = rospy.ServiceProxy('hand_controller/openFinger', graspPreparation)
        self._group = moveit_commander.MoveGroupCommander("arm")
        pass

    def configure(self):
        pass

    def update(self):
        pass

    def look_down(self):
        look_down = rospy.ServiceProxy('/tilt_controller/resetPosition', Empty)
        try:
            rospy.wait_for_service('/tilt_controller/resetPosition', timeout=10)
            look_down()
        except (rospy.ROSException, rospy.ServiceException):
            rospy.logdebug('looking down failed')
        rospy.sleep(1.0)

    def transform_pose(self):
        tmp = PoseStamped()
        tmp.header.frame_id = "base_link"
        tmp.pose.position = self._grasp_points.graspOutput.graspPoint1
        print(tmp)
        tmp.pose.position.z = tmp.pose.position.z + 0.2
        print(tmp)
        listener = tf.TransformListener()
        try:
            listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration.from_sec(60))
            self._grasp_pose = listener.transformPose("odom", tmp)
            print(self._grasp_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            pass
        #return
 
        self._grasp_pose.pose.orientation.x = 0.0
        self._grasp_pose.pose.orientation.y = 1.0
        self._grasp_pose.pose.orientation.z = 0.0
        self._grasp_pose.pose.orientation.w = 1.0

    def send_pose_to_motion_planner(self):
        self._group.clear_pose_targets()
        self._group.set_start_state_to_current_state()
        self._group.set_pose_reference_frame("/odom")
        self._group.set_pose_target(self._grasp_pose)
        plan = self._group.plan()
        #group.go(wait=True)
        rospy.sleep(5.0)

    def move_arm_straight(self, direction='up'):
        # move up or down by 0.1 meter
        if direction == 'up':
            diff = 0.1
        elif direction == 'down':
            diff = -0.1
        else:
            return
    
        waypoints = []
        # start with the current pose
        waypoints.append(group.get_current_pose().pose)
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.x = waypoints[0].orientation.x
        wpose.orientation.y = waypoints[0].orientation.y
        wpose.orientation.z = waypoints[0].orientation.z
        wpose.orientation.w = waypoints[0].orientation.w
        wpose.position.x = waypoints[0].position.x + diff
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z
        waypoints.append(copy.deepcopy(wpose))
        (plan1, fraction) = self._group.compute_cartesian_path(waypoints, 0.01, 0.0)
        rospy.sleep(5.0)

    def open_close_gripper(self, open=True):
        if open:
            print("open fingers")
            self._openFinger()
        else:
            print("close fingers")
            self._closeFinger(1.0)

    def get_grasp_points(self):
        client = actionlib.SimpleActionClient('calc_grasppoints_svm_action_server', \
            CalcGraspPointsServerAction)
        client.wait_for_server()

        goal = CalcGraspPointsServerGoal()
        goal.graspinput.input_pc = self._point_cloud
        goal.graspinput.grasp_area_center = self._graspsearchcenter
        goal.graspinput.approach_vector = self._approach_vector
        goal.graspinput.grasp_area_length_x = self._grasp_search_size_x+14
	goal.graspinput.grasp_area_length_y = self._grasp_search_size_y+14
        goal.graspinput.max_calculation_time = self._grasp_calculation_time_max
        goal.graspinput.show_only_best_grasp = self._show_only_best_grasp
        goal.graspinput.gripper_opening_width = self._gripper_opening_width
        client.send_goal(goal)
        client.wait_for_result()
        self._grasp_points = client.get_result()
        

    def execute_squirrel_object_perception_cb(self, goal):
        # check that preempt has not been requested by the client
        if self.as_squirrel_object_perception.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self.as_squirrel_object_perception.set_preempted()
            return

        # first look down on the floor
        # self.look_down()

        # Start getting data from /kinect/depth_registered/points
        # Set feedback to data acquisition succeeded and set percentage
        rospy.loginfo('started')
        try:
            self._point_cloud = rospy.wait_for_message(
                '/kinect/depth_registered/points',
                PointCloud2,
                timeout=5
            )
            rospy.loginfo('Got point cloud')
            self.get_grasp_points()
            self.transform_pose()
            #self.send_pose_to_motion_planner()
            #self.open_close_gripper(open=True)
            #self.move_arm_straight(direction='down')
            #self.open_close_gripper(open=False)
            #self.move_arm_straight(direction='up')
        except rospy.ROSException:
            self.as_squirrel_object_perception.set_aborted(self._result)
            rospy.logdebug('receive_data failed')
            return

        rospy.loginfo('%s: Succeeded' % self._action_name)
        self.as_squirrel_object_perception.set_succeeded(self._result)
        return


class SquirrelGraspObject:
    def __init__(self):
        self.impl = SquirrelGraspObjectImpl()
        self.impl._action_name = 'grasp_objects'
        #self_dynrecon_server = dynamic_reconfigure.server.Server(
        #    ConfigType, self.config_callback)
        self.impl.as_squirrel_object_perception = actionlib.SimpleActionServer(
            'grasp_objects',
            GraspObjectAction,
            execute_cb=self.impl.execute_squirrel_object_perception_cb,
            auto_start=False)
        self.impl.as_squirrel_object_perception.start()

    def run(self):
        self.impl.update()

    def config_callback(self, config, level):
        return config

if __name__ == "__main__":
    try:
        rospy.init_node('squirrel_grasp_objects')
        n = SquirrelGraspObject()
        n.impl.configure()
        while not rospy.is_shutdown():
            n.run()
            rospy.sleep(10.0)

    except rospy.ROSInterruptException:
        print "Exit"
