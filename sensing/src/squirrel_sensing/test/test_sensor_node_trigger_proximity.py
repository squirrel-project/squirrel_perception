#!/usr/bin/env python
import roslib; roslib.load_manifest('squirrel_sensing')
import rospy

import actionlib
import yaml
from bride_tutorials.msg import *

test_trigger_proximity_goal = open("test/testdata_sensor_node_trigger_proximity", 'r')

def trigger_proximity_client():
    client = actionlib.SimpleActionClient('/trigger_proximity', TriggerPublishAction)
    client.wait_for_server()
    
    goal =TriggerPublishGoal()
    genpy.message.fill_message_args(goal, yaml.load(test_trigger_proximity_goal))

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":
    rospy.init_node('trigger_proximity_test')
    print trigger_proximity_client()
