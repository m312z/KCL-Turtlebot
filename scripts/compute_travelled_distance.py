#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry
from math import sqrt

class DistanceCompute:
    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.started = False
        self.total_distance = 0

    def distance(self, a, b):  # a and b are Points
        return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def odom_cb(self, msg):
        if not self.started:
            self.started = True
        else:
            self.total_distance += self.distance(self.previous_pose, msg.pose.pose.position)
        self.previous_pose = msg.pose.pose.position


if __name__ == "__main__":
    rospy.init_node('kcl_turtlebot_distance_compute', anonymous=False)
    d = DistanceCompute()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except:
            pass
    print 'Total distance traveled is:', d.total_distance, 'meters'
