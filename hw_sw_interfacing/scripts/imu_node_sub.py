#!/usr/bin/env python
import random
import math
import rospy
from hw_sw_interfacing.msg import IMU


def imu_cb(data):
	rospy.loginfo("Angular velocities : u = %d | v = %d | r = %d", data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
	rospy.loginfo("linear accelerations : ax = %d | ay = %d | az = %d", data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
	rospy.loginfo("--------------------------------------------------")


def imu_listen():
	rospy.init_node("imu_client")
	sub1 = rospy.Subscriber('/nemo/imu', IMU, imu_cb) 
	rospy.spin()


if __name__ == "__main__":
    try:
    	imu_listen()
    except rospy.ROSInterruptException:
        rospy.logerr('could not start')
