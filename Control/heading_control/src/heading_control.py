#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64

class HeadingControl(object):
    def __init__(self):
        rospy.init_node('heading_control')
        self.desired_heading = rospy.get_param('~desired_heading', 0.0)
        self.error_tolerance = rospy.get_param('~error_tolerance', 6.0)
        self.pub = rospy.Publisher('/heading_control/output_signal', Float32, queue_size=1)
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.CompassCallback)
        rospy.spin()
    
    def CompassCallback(self, msg):
        if((self.desired_heading - msg) > self.error_tolerance):
            # Move Left
            self.pub.publish(1)
        elif((self.desired_heading - msg) < -self.error_tolerance):
            # Move Right
            self.pub.publish(-1)
        else:
            # Move Forward
            self.pub.publish(0)

if __name__ == "__main__":
    try:
        HeadingControl()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start heading_control node.')
    