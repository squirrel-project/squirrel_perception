#!/usr/bin/env python
import roslib; roslib.load_manifest('squirrel_sensing_coordinator')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *
from smach_ros import ActionServerWrapper
from bride_tutorials.msg import TriggerPublishAction, TriggerPublishGoal


# protected region customHeaders on begin #
# protected region customHeaders end #



class sensor_testing_loop_impl:
	
	def	__init__(self):
		self.TriggerPublisher_goal = TriggerPublishGoal()
		genpy.message.fill_message_args(self.TriggerPublisher_goal, [rospy.get_param('trigger_proximity')])
	
		# protected region initCode on begin #
		# protected region initCode end #
		pass
	
	def	configure(self):
		sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys = ['action_feedback'], output_keys = ['action_feedback'])
		sis = smach_ros.IntrospectionServer('sensor_testing_loop', sm0, '/sensor_testing_loop_sm')
		sis.start()
		with sm0:
			smach.StateMachine.add('TriggerPublisher', smach_ros.SimpleActionState('TriggerPublisher', TriggerPublishAction, self.TriggerPublisher_goal), {
				"succeeded":"succeeded",
			})
	

	
		sm0.set_initial_state(['TriggerPublisher'])

		# Execute

		#sm0.set_initial_state()
		outcome = sm0.execute()
	
		# protected region configureCode on begin #
		# protected region configureCode end #
		pass
	
	def	update(self):
		# protected region updateCode on begin #
		# protected region updateCode end #
		pass
		
	

class sensor_testing_loop:
	def __init__(self):
		self.impl = sensor_testing_loop_impl()

	
		
	def run(self):
		self.impl.update()

if __name__ == "__main__":
	try:
		rospy.init_node('sensor_testing_loop')
		r = rospy.Rate(10)
		n = sensor_testing_loop()
		n.impl.configure()
		while not rospy.is_shutdown():
			n.run()
			r.sleep()
			
	except rospy.ROSInterruptException:
		print "Exit"



