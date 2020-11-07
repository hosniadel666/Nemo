#!/usr/bin/env python
import random
import math
import rospy
from hw_sw_interfacing.msg import DEPTH
from std_msgs.msg import Float32MultiArray

# def test_cb(data):
# 	for i in range(0, 3):
# 		rospy.loginfo("Nemo Depth: %f", data.data[i])
# 		rospy.loginfo("----------------------")


def depth_cb(data):
	rospy.loginfo("Nemo Depth: %f", data.z)
	rospy.loginfo("----------------------")


def depth_listen():
	rospy.init_node("depth_client")
	sub1 = rospy.Subscriber('/nemo/depth', DEPTH, depth_cb) 
	#sub1 = rospy.Subscriber('/test', Float32MultiArray, test_cb) 
	rospy.spin()


if __name__ == "__main__":
    try:
    	depth_listen()
    except rospy.ROSInterruptException:
        rospy.logerr('could not start')
