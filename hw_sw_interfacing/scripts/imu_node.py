#!/usr/bin/env python
import random
import math
import rospy
from hw_sw_interfacing.msg import IMU

class IMU_CLASS(object):

	def __init__(self):
		rospy.init_node("IMU_node")
		self.pub = rospy.Publisher("/nemo/imu", IMU, queue_size=1)

	   	self.imu_data = IMU()
	   	self.rate = rospy.Rate(10) # 10hz

	   	self.IMU_driver()

	def IMU_driver(self):
		while not rospy.is_shutdown():
			self.imu_data.angular_velocity.x = random.randint(10,50) 
			self.imu_data.angular_velocity.y = random.randint(10,50) 
			self.imu_data.angular_velocity.z = random.randint(10,50)
			self.imu_data.linear_acceleration.x = random.randint(10,50) 
			self.imu_data.linear_acceleration.y = random.randint(10,50) 
			self.imu_data.linear_acceleration.z = random.randint(10,50) 
			
			self.pub.publish(self.imu_data)
			self.rate.sleep()



if __name__ == "__main__":
    try:
        IMU_CLASS()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')