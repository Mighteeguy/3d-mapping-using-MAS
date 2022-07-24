#!/usr/bin/env python
import sys
import rospy
from exploration.core import Core, State
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, Pose #edit
from nav_msgs.msg import Odometry
from caltech_samaritan.msg import uav_pose 
import math

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print("enter correct uav name")
    else:
        uav_prefix = sys.argv[1]
        
    rospy.loginfo('Initialising core...')
    rospy.init_node('exploration_core', anonymous=True)
    core = Core()
    rospy.loginfo('Done!')

    rospy.loginfo('Control loop is running.')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        next_state = core.step()
        if next_state == State.Done:
            rospy.loginfo('Exploration script has finished!')
            break
        rate.sleep()
