#!/usr/bin/env python

import sys
import rospy
import tf
import moveit_commander
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi,cos,sin

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('circle',anonymous=True)

group = moveit_commander.MoveGroupCommander("manipulator")   # ur3 moveit group name: manipulator

#get the position matrix of a circle with a radius of 0.1
waypoints=[]
init_pose=group.get_current_pose().pose
waypoints.append(init_pose)
centerY=init_pose.position.y
centerZ=init_pose.position.z
radius=0.1
mu=0.0
while mu<=6.28:
  init_pose.position.y=centerY+radius*cos(mu)
  init_pose.position.y=centerZ+radius*sin(mu)
  #print(init_pose)
  waypoints.append(init_pose)
  mu+=0.01


# use Cartesian path to plan the trajectory
fraction=0.0
attempts=0
while fraction<1 and attempts<20:    
  (plan, fraction) = group.compute_cartesian_path(waypoints,   # waypoints to follow
                                                  0.01,        # end_effector_step, Cartesian path to be interpolated at a step of 0.01
                                                  0.0,         # jump_threshold
                                                  avoid_collisions=True) 
  attempts+=1


if(fraction==1):
  group.execute(plan)
      

rospy.sleep(10)
moveit_commander.roscpp_shutdown()
