#!/usr/bin/env python
import rospy
import sys
from math import sqrt
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import PoseStamped
from rosplan_knowledge_msgs.srv import GetInstanceService
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue
import numpy as np

def print_matrix(d):
    for r in d:
        for i, el in enumerate(r):
            if (i != 0):
                sys.stdout.write(",\t")
            sys.stdout.write(str(el))
        sys.stdout.write("\n")


def distance(a, b):
    a = a.pose.position
    b = b.pose.position
    return sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2 )


def compute_distances(poses):
    n = len(poses)
    d = [[0]*n for _ in range(n)]  # Init matrix
    d[1][0] = 3
    for i in xrange(n):
        for j in xrange(i):
            d[i][j] = d[j][i] = distance(poses[i], poses[j])
    return d


def get_kb_update(instances, d):
    kua = KnowledgeUpdateServiceArrayRequest()
    n = len(instances)
    assert(n == len(d) and n == len(d[0]))
    for i in xrange(n):
        for j in xrange(n):
            ki = KnowledgeItem()
            ki.knowledge_type = ki.FUNCTION
            ki.attribute_name = 'distance'
            kv = KeyValue()
            kv.key = 'a'
            kv.value = instances [i]
            ki.values.append(kv)
            kv = KeyValue()
            kv.key = 'b'
            kv.value = instances [j]
            ki.values.append(kv)
            ki.function_value = d[i][j]
            kua.update_type += np.array(kua.ADD_KNOWLEDGE).tostring()
            kua.knowledge.append(ki)
    return kua


if __name__ == "__main__":
    rospy.init_node('kcl_turtlebot_init', anonymous=True)
    msproxy = MessageStoreProxy()

    try:
        get_instances = rospy.ServiceProxy('/rosplan_knowledge_base/state/instances', GetInstanceService)
        rospy.loginfo("Waiting for the instances service to be ready...")
        get_instances.wait_for_service(60)
        instances = get_instances('waypoint').instances
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s", e)
        sys.exit(-1)

    poses = []
    for i in instances:
        poses.append(msproxy.query_named(i, PoseStamped._type)[0])

    d = compute_distances(poses)

    try:
        msgs = get_kb_update(instances, d)
        update_kb = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
        res = update_kb(msgs)
        if res.success:
            print "Done!"
        else:
            print "Something went wrong!"
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        sys.exit(-1)
