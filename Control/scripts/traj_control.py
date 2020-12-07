#!/usr/bin/env python
import rospy
import numpy as np
from hw_sw_interfacing.msg import filteredData
from control.msg import trajAction
from slam.msg import trajData

#set points are set to zero by default unless it subscribed from mission planning

class traj_control(object):
    def __init__(self):
        rospy.init_node('traj_control')

        self.pub = rospy.Publisher('traj_action', trajAction, queue_size=10)
        rospy.Subscriber("ekf_data", filteredData, self.feedback_callback)
        rospy.Subscriber("trajectory", trajData , self.setpoint_callback)
        self.trajAction = trajAction()

        rospy.spin()


    def feedback_callback(self, data):
        pass
        # set the values of the parameters of algorithm to the input data

        #self.dp_control_action.thrusters_vr = np.ones(8,10)
        #it publishes a trajectory of torques 
        #for every thruster(8 thrusters) 10 commands

    def setpoint_callback(self, data):
        #call the control algorithm function and publish to control node
        #the thrusters torque required 
        pass


if __name__ == "__main__":
    try:
        traj_control()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')