#!/usr/bin/env python
import random
import math
import rospy
from hw_sw_interfacing.msg import DVL


def dvl_cb(data):
	rospy.loginfo("Vx = %d | Vy = %d | Vz = %d", data.bottom_velocity.x, data.bottom_velocity.y, data.bottom_velocity.z)
	rospy.loginfo("----------------------------")

def dvl_listen():
	rospy.init_node("dvl_client")
	sub1 = rospy.Subscriber('/nemo/dvl', DVL, dvl_cb) 
	rospy.spin()


if __name__ == "__main__":
    try:
    	dvl_listen()
    except rospy.ROSInterruptException:
        rospy.logerr('could not start')
