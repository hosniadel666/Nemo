#!/usr/bin/env python
import random
import math
import rospy
from slam.msg import filteredData


def fd_cb(data):
	rospy.loginfo("Vx = %d | Vy = %d | Vz = %d", data.bottom_velocity.x, data.bottom_velocity.y, data.bottom_velocity.z)
	rospy.loginfo("Angular velocities : u = %d | v = %d | r = %d", data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
	rospy.loginfo("linear accelerations : ax = %d | ay = %d | az = %d", data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
	rospy.loginfo("Nemo Depth: %f", data.z)
	rospy.loginfo("--------------------------------------------------")

def ekf_listen():
	rospy.init_node("ekf_client")
	sub1 = rospy.Subscriber('ekf_data', filteredData, fd_cb) 
	rospy.spin()


if __name__ == "__main__":
    try:
    	ekf_listen()
    except rospy.ROSInterruptException:
        rospy.logerr('could not start')
