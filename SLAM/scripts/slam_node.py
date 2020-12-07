#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from slam.msg import filteredData
from slam.msg import stateCovar
from hw_sw_interfacing.msg import IMU
from hw_sw_interfacing.msg import DVL
from hw_sw_interfacing.msg import DEPTH


class slam:
    def __init__(self):
        rospy.init_node('slam_node')
        self.pubs = rospy.Publisher('state_covariance', stateCovar, queue_size=10)
        self.pubs = rospy.Publisher('ekf_data', filteredData, queue_size=10)
        rospy.Subscriber("/nemo/imu", IMU, self.imu_callback)
        rospy.Subscriber("/nemo/depth", DEPTH, self.depth_callback)
        rospy.Subscriber("/nemo/dvl", DVL, self.dvl_callback)
        rospy.Subscriber("/nemo/mission", Image, self.percept_callback)
        self.rate = rospy.Rate(10) # 10hz    
        self.filteredData = filteredData()
        while not rospy.is_shutdown():
            self.pubs.publish(self.filteredData)
            self.rate.sleep()



    def depth_callback(self, data):
        self.filteredData.z = data.z

    def imu_callback(self, data):
        self.filteredData.linear_acceleration.x = data.linear_acceleration.x
        self.filteredData.linear_acceleration.y = data.linear_acceleration.y
        self.filteredData.linear_acceleration.z = data.linear_acceleration.z
        self.filteredData.angular_velocity.x = data.angular_velocity.x
        self.filteredData.angular_velocity.y = data.angular_velocity.y
        self.filteredData.angular_velocity.z = data.angular_velocity.z


    def dvl_callback(self, data):
        self.filteredData.bottom_velocity.x = data.bottom_velocity.x
        self.filteredData.bottom_velocity.y = data.bottom_velocity.y
        self.filteredData.bottom_velocity.z = data.bottom_velocity.z

    def percept_callback(self, data):
        pass
        #self.msg1.orientation = data.perception



if __name__ == "__main__":
    try:
        slam()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')
       
