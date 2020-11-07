#!/usr/bin/env python
import random
import math
import rospy
from hw_sw_interfacing.msg import DEPTH
from std_msgs.msg import Float32MultiArray

class Pressure(object):
	

	def __init__(self):
		rospy.init_node("Depth_node")
		self.pub = rospy.Publisher("/nemo/depth", DEPTH, queue_size=1)
		self.pub2 = rospy.Publisher("/test", Float32MultiArray, queue_size=1)

	   	self.depth_data = DEPTH()
	   	self.rate = rospy.Rate(10) # 10hz
	   	#self.arr = Float32MultiArray()
	   	#self.data = []

	   	self.IMU_driver()

	def IMU_driver(self):
		while not rospy.is_shutdown():
			self.depth_data.z = random.randint(10,50) 

			#self.arr.data.append(1)		
			self.pub.publish(self.depth_data)
			#self.pub2.publish(self.arr)
			self.rate.sleep()



if __name__ == "__main__":
    try:
        Pressure()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')