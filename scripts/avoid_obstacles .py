#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

class MoveItObstaclesDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_obstacles_demo',anonymous=True)
        scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        rospy.sleep(1)
        group = moveit_commander.MoveGroupCommander("manipulator")
        end_effector_link = group.get_end_effector_link()
        
        # set the tolerance
        group.set_goal_position_tolerance(0.01)
        group.set_goal_orientation_tolerance(0.05)
        group.allow_replanning(True)
        
        # set the base link
        reference_frame = 'base_link'
        group.set_pose_reference_frame(reference_frame)
        
        #add the obstacles
        box1_id = 'box1'
        box2_id = 'box2'
             
        # back to the'home' postion
        group.set_named_target('home')
        group.go(wait=True)
        rospy.sleep(2)
        
               
        box1_size = [0.1, 0.01, 0.2]
        box2_size = [0.1, 0.01, 0.2]
        
        # delopy boxes
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = 0.3
        box1_pose.pose.position.y = 0.15
        box1_pose.pose.position.z = 0.4
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = reference_frame
        box2_pose.pose.position.x = 0.3
        box2_pose.pose.position.y = 0
        box2_pose.pose.position.z = 0.2
        box2_pose.pose.orientation.w = 1.0   
        scene.add_box(box2_id, box2_pose, box2_size)

        # dye the boxes 
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
        self.sendColors()    
        
        # set the departure and termination point
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.3
        target_pose.pose.orientation.w = 1.0
        
        
        group.set_pose_target(target_pose, end_effector_link)
        group.go(wait=True)
        rospy.sleep(2)

        target_pose2 = PoseStamped()
        target_pose2.header.frame_id = reference_frame
        target_pose2.pose.position.x = 0.3
        target_pose2.pose.position.y = 0
        target_pose2.pose.position.z =  0.4
        target_pose2.pose.orientation.w = 1.0
        
        group.set_pose_target(target_pose2, end_effector_link)
        group.go(wait=True)
        rospy.sleep(2)
        

        group.set_named_target('home')
        group.go(wait=True)
        
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
    def setColor(self, name, r, g, b, a = 0.9):

        color = ObjectColor()        
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color


    def sendColors(self):
        p = PlanningScene()  
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItObstaclesDemo()
    except KeyboardInterrupt:
        raise
