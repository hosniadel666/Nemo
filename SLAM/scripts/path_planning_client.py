#!/usr/bin/env python

import sys
import rospy
from slam.srv import path
from slam.srv import pathRequest
from slam.srv import pathResponse
from geometry_msgs.msg import Vector3

def path_planner_client(x, y):
    rospy.wait_for_service('path_planning_service')
    try:
        trajectory = rospy.ServiceProxy('path_planning_service', path)
        traj = trajectory(x, y)
        return traj.v_traj, traj.r_traj
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    c = Vector3()
    d = Vector3()
    c.x = 1
    c.y = 2
    c.z = 3
    d.x = 1
    d.y = 2 
    d.z = 3
    v, r = path_planner_client(c, d)
    print "%s and %s"% (v, r)
    
