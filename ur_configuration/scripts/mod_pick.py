#!/usr/bin/env python2
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import math 
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur_msgs.srv import *

moveit_commander.roscpp_initialize(sys.argv)
#start ros node
rospy.init_node('Move_group_info',anonymous=True)

# import robot commander
robot= moveit_commander.RobotCommander()

#import planning scene
scene = moveit_commander.PlanningSceneInterface()

# moveit move group name 
arm_name = 'arm'
arm_group = moveit_commander.MoveGroupCommander(arm_name)

gripper_name = 'hand'
gripper_group = moveit_commander.MoveGroupCommander(gripper_name)

#publisher to visualize planned trajectory 
DisplayTraj = rospy.Publisher('/move_group/display_planned_path' ,
                            moveit_msgs.msg.DisplayTrajectory,
                            queue_size=20)



current_pose = robot.get_current_state()
planning_frame = arm_group.get_planning_frame()
planning_fram_g = gripper_group.get_planning_frame()
eef_link = arm_group.get_end_effector_link()
groups = robot.get_group_names()


print('robot pose :', current_pose )
# print('planning frame :' , planning_frame)
# print(planning_fram_g)
# print('eef_link :', eef_link)
# print('planning groups :', groups)

print(arm_group.get_current_pose())
# print(arm_group.get_goal_tolerance())

current_pose = arm_group.get_current_pose()
orientation = [
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w]

rot = tf.transformations.euler_matrix(-math.pi/2,0,0,'sxyz')
print(rot)
rot1 = tf.transformations.quaternion_matrix(orientation)

rot3 = np.matmul(rot1,rot)
print(rot3)

new_or = tf.transformations.quaternion_from_matrix(rot3)
print(new_or)

pose = geometry_msgs.msg.PoseStamped()
pose.header.frame_id = planning_frame
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 0
normal=(0,0,1)
offset=0

scene.add_plane('floor', pose, normal, offset)

pose = [-0.7157, 0.044,0.37588]
# orientation = tf.transformations.quaternion_from_euler(0,0,1.0,'sxyz')

orientation = [0.646,0.286,-0.2858,0.6467]
orientation = [
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w]


grasp_pose = geometry_msgs.msg.PoseStamped()
grasp_pose.header.frame_id = 'world'
grasp_pose.pose.position.x = pose[0]
grasp_pose.pose.position.y = pose[1]
grasp_pose.pose.position.z = pose[2]
grasp_pose.pose.orientation.x = orientation[1]
grasp_pose.pose.orientation.y = orientation[2]
grasp_pose.pose.orientation.z = orientation[3]
grasp_pose.pose.orientation.w = orientation[0]

arm_group.set_named_target('home')
flag, plan, time, error = arm_group.plan()
arm_group.execute(plan)

arm_group.set_pose_target(grasp_pose)
flag, plan, time, error = arm_group.plan()
arm_group.execute(plan)


rot = tf.transformations.euler_matrix(-math.pi/2,0,0,'sxyz')
print(rot)
rot1 = tf.transformations.quaternion_matrix(orientation)

rot3 = np.matmul(rot1,rot)
print(rot3)

orientation = tf.transformations.quaternion_from_matrix(rot3)
print(new_or)

current_pose = arm_group.get_current_pose()
orientation = [
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w]

grasp_pose.pose.orientation.x = orientation[1]
grasp_pose.pose.orientation.y = orientation[2]
grasp_pose.pose.orientation.z = orientation[3]
grasp_pose.pose.orientation.w = orientation[0]

arm_group.set_pose_target(grasp_pose)
flag, plan, time, error = arm_group.plan()
arm_group.execute(plan)
# print('target pose', grasp_pose)
# print('current pose', arm_group.get_current_pose())