#!/usr/bin/env python
import rospy
from control.msg import controlAction
from control.msg import depthAction
from control.msg import trajAction


#in this node, it should sum up the control that comes from two diff control nodes
#one for depth and stability control, and the other from trajctory control
#it takes the torque of every thruster and should translate it to 
#pwm signal, that should be published to the pixhawk controller and then to the thrusters


class control(object):
    def __init__(self):
        rospy.init_node('control_cominator')
        self.pub = rospy.Publisher('action', controlAction, queue_size=10)
        #rospy.Subscriber("ekf_data", filtered_data, feedback_callback)
        rospy.Subscriber("traj_action", trajAction , self.setpoints_traj_callback)
        rospy.Subscriber("depth_action", depthAction, self.setpoints_depth_callback)
        
        self.controlAction = controlAction()

        rospy.spin()


    #def feedback_callback():

    def setpoints_traj_callback(self, data):
        pass

    def setpoints_depth_callback(self, data):
        pass
        #publish here

    


if __name__ == "__main__":
    try:
        control()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')

