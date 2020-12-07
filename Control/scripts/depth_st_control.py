#!/usr/bin/env python
import rospy
from hw_sw_interfacing.msg import filteredData
from hw_sw_interfacing.msg import DEPTH
from control.msg import depthAction

#set points are set to zero (for roll and pitch) 
# but for z it remains the same unless 
#control subscribed from mission planning again


class depth_st_control(object):
    def __init__(self):
        rospy.init_node('depth_st_control')

        self.pub = rospy.Publisher('depth_action', depthAction, queue_size=10)


        rospy.Subscriber("ekf_data", filteredData, self.feedback_callback)
        rospy.Subscriber("depth_data", DEPTH , self.depthpoint_callback)
        rospy.spin()
        #while not rospy.is_shutdown():

            #self.pub.publish(self.dp_control_action)
            #self.rate.sleep()

    def feedback_callback(self, data):
        pass
        #call control function for depth and stability
        #self.dp_control_action.thrusters_hr = [1 2 3]   #torques of horizontal thrusters
        
        #publish but in a rate slower than the feedback
        
        #self.pub.publish(self.dp_control_action)



    def depthpoint_callback(self, data):
        pass



if __name__ == "__main__":
    try:
        depth_st_control()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')
