#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PerceptionModule(object):
	def __init__(self):

		# OpenCV Bridge
		self.bridge = CvBridge()
		rospy.init_node('Perception_node')

		# Subscribers
		sub1 = rospy.Subscriber('/usb_cam_node/image_raw', Image, self.perception_cb) 

		rospy.spin()


	def perception_cb(self, msg):
		print("Received Image.")
		#rospy.loginfo("Received Image.")
		#do some processing on the image and publish data to slam module (ekf)




if __name__ == '__main__':
	try:
		PerceptionModule()
	except rospy.ROSInterruptException:
		rospy.logerr('Error')
