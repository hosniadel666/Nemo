#!/usr/bin/env python
import rospy
from slam.msg import filteredData
from slam.msg import trajData
from slam.msg import stateCovar
from slam.srv import path
from slam.srv import pathRequest
from slam.srv import pathResponse
from geometry_msgs.msg import Vector3


class mission_planning():


    def __init__(self):
        self.c = Vector3()
        self.d = Vector3()
        self.c.x = 1
        self.c.y = 2
        self.c.z = 3
        self.d.x = 1
        self.d.y = 2 
        self.d.z = 3

        rospy.init_node('mission_planning')
        rospy.Subscriber("state_covariance", stateCovar, self.stateCovar_callback)
        #rospy.Subscriber("ekf_data", filteredData, self.feedback_callback)
        self.pub = rospy.Publisher('trajectory', trajData, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.control_msg = trajData()
        rospy.spin()

    def stateCovar_callback(self):
        pass
    def feedback_callback(self, data):

        #extract states and state variance
        #call mission planner function
        self.mission_planning()

    def mission_planning(self):
        #, state, state_variance):
        #if condition specifies if we are in a mission or not
        #if we need path planning, request service from path planning
        # and returns trajectory
        #and publish this trajectory to the respective control node
        #according to the perception variance, object is detected or not
        rospy.wait_for_service('path_planning_service')
        try:
            trajectory = rospy.ServiceProxy('path_planning_service', path)
            traj = trajectory(self.c, self.d)
            return traj.v_traj, traj.r_traj
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        '''
        rospy.wait_for_service('path_planning_service')
        
        try:
            path_fn = rospy.ServiceProxy('path_planning_service', path)
            traj = path_fn(self.c, self.d)
            self.control_msg.v_traj = tarj.v_traj
            self.control_msg.r_traj = tarj.r_traj
            self.pub.publish(control_msg)
        except rospy.ServiceException, e:
            print "Service call Failed : %s" %e    
        '''





if __name__ == "__main__":
    try:
        mission_planning()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')
            


