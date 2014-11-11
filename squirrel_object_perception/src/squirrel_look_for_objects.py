#!/usr/bin/env python
import roslib; roslib.load_manifest('squirrel_object_perception')
import rospy
import actionlib
import dynamic_reconfigure.server
from squirrel_object_perception.cfg import squirrel_look_for_objectsConfig as ConfigType



# protected region customHeaders on begin #
# protected region customHeaders end #

from squirrel_object_perception_msgs.msg import LookForObjectsAction, LookForObjectsFeedback, LookForObjectsResult


class squirrel_look_for_objects_impl:

	def	__init__(self):



		# protected region initCode on begin #
		# protected region initCode end #
		pass

	def	configure(self):
		# protected region configureCode on begin #
        # protected region configureCode end #
		pass

	def	update(self):
		# protected region updateCode on begin #
        # protected region updateCode end #
		pass


	def	execute_squirrel_object_perception_cb(self, goal):
		# Examples:	self.as_squirrel_object_perception.set_succeeded(_result)
		#			self.as_squirrel_object_perception.set_aborted()
		#			self.as_squirrel_object_perception.publish_feedback(_feedback)
		_feedback = LookForObjectsFeedback()
		_result = LookForObjectsResult()
		# protected region user implementation of action callback for squirrel_object_perception on begin #
		# protected region user implementation of action callback for squirrel_object_perception end #
		pass


class squirrel_look_for_objects:
	def __init__(self):
		self.impl = squirrel_look_for_objects_impl()
		self_dynrecon_server = dynamic_reconfigure.server.Server(ConfigType, self.config_callback)
		self.impl.as_squirrel_object_perception = actionlib.SimpleActionServer('squirrel_object_perception', LookForObjectsAction, execute_cb=self.impl.execute_squirrel_object_perception_cb, auto_start=False)
		self.impl.as_squirrel_object_perception.start()


	def run(self):
		self.impl.update()

	def config_callback(self, config, level):
		return config

if __name__ == "__main__":
	try:
		rospy.init_node('squirrel_look_for_objects')
		n = squirrel_look_for_objects()
		n.impl.configure()
		while not rospy.is_shutdown():
			n.run()
			rospy.sleep(10.0)

	except rospy.ROSInterruptException:
		print "Exit"
