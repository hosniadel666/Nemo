import rospy
#from sw_hw.msg import filtered_data
#from sw_hw.msg import trajectory_data

from slam.srv import path
from slam.srv import pathRequest
from slam.srv import pathResponse

class path_planning:

    def __init__(self):

        rospy.init_node('path_planning')
        #self.pubp = rospy.Publisher('trajectory', trajectory_data, queue_size=10)
        #rospy.Subscriber("ekf_data", filtered_data, feedback_callback)
        #rospy.Subscriber("targets", target_data, mission_callback)
        s = rospy.Service('path_planning_service', path, self.path_planner_handler)
        rospy.spin()


    def path_planner_handler(self, req):
        # we do processing for the req (current_pose and target_pose)
        # and return 2 arrays of trajectory to mission planner
        v_traj = "v_trajectory list"
        r_traj = "r_trajectory _list"
        return pathResponse(v_traj, r_traj)





if __name__ == "__main__":
    try:
        path_planning()

    except rospy.ROSInterruptException:
        rospy.logerr('could not start')

        