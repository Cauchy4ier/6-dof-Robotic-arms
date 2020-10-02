#!/usr/bin/env python

import sys
import rospy
import tf
import moveit_commander 
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi

pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('movetopoint',anonymous=True)
group = moveit_commander.MoveGroupCommander("manipulator")  # ur3 moveit group name: manipulator

      
pose_goal.orientation.w = 0.0 # move to point[w,x,y,z]=[0.0,0.4,0.2,0.4]
pose_goal.position.x = 0.4  
pose_goal.position.y = 0.2 
pose_goal.position.z = 0.4 
group.set_pose_target(pose_goal)
group.go(wait=True)
    

rospy.sleep(5)
      


moveit_commander.roscpp_shutdown()
