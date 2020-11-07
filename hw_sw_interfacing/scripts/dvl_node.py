#!/usr/bin/env python
import random
import math
import rospy
from hw_sw_interfacing.msg import DVL

class DVL_CLASS(object):

	def __init__(self):
		rospy.init_node("dvl_node")
		self.pub = rospy.Publisher("/nemo/dvl", DVL, queue_size=1)
	    #port = rospy.get_param("~port")
	    #baudrate = rospy.get_param("~baudrate")
	    #frame_id = rospy.get_param("~frame")
	    #timeout = rospy.get_param("~timeout")
	   	self.dvl_data = DVL()
	   	self.rate = rospy.Rate(10) # 10hz

	   	self.DVL_driver()

	def DVL_driver(self):
		while not rospy.is_shutdown():
			self.dvl_data.bottom_velocity.x = random.randint(10,50) 
			self.dvl_data.bottom_velocity.y = random.randint(10,50) 
			self.dvl_data.bottom_velocity.z = random.randint(10,50) 
			
			self.pub.publish(self.dvl_data)
			self.rate.sleep()



if __name__ == "__main__":
    try:
        DVL_CLASS()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')