#!/usr/bin/env python
#
# Simple node to inject an object into the database
# For testing purposes.
#
# Jan 2016, Michael Zillich <michael.zillich@tuwien.ac.at>

import getopt, sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from squirrel_planning_knowledge_msgs.srv import AddObjectService, \
    AddObjectServiceRequest, UpdateObjectService, UpdateObjectServiceRequest

def add_object_to_db(id, category, pose, size):
    add_object = rospy.ServiceProxy('/kcl_rosplan/add_object', AddObjectService)
    try:
        rospy.wait_for_service('/kcl_rosplan/add_object', timeout=3)
        request = AddObjectServiceRequest()
        request.id = id
        request.category = category
        request.pose = pose
        request.size = size
        # note: we leave the cloud empty: request.cloud
        resp = add_object(request)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        resp = False
    return resp

def usage():
    print "Injects an object with ID and category/class into the scene database."
    print "-i id -c category"

if __name__ == '__main__':

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hi:c:s:", ["help", "id=", "category=", "size="])

        id = ""
        category = ""
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.y = 0
        pose.pose.orientation.x = pose.pose.orientation.y = pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        size = 0.0

        for o, a in opts:
            if o == "-i":
                id = a
            elif o == "-c":
                category = a
            elif o == "-s":
                size = float(a)
            elif o in ("-h", "--help"):
                usage()
                sys.exit()
            else:
                assert False, "unhandled option"
        print "injecting object '" + str(id) + "' of category '" + category + "' with size " + str(size)
        add_object_to_db(id, category, pose, size)

    except getopt.GetoptError as err:
        # print help information and exit:
        print str(err) # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
    except rospy.ROSInterruptException:
        pass
