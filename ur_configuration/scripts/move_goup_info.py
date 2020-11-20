#!/usr/bin/env python

from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import math 
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
print('planning frame :' , planning_frame)
print(planning_fram_g)
print('eef_link :', eef_link)
print('planning groups :', groups)

print(arm_group.get_current_pose())
print(arm_group.get_goal_tolerance())

def pre_grasp_pose(target_pose, side = 2 , shift=0.1):
    ''' side (0..2, x,y,z)''' 
    shift= -shift
    x = target_pose.pose.position.x
    y = target_pose.pose.position.y
    z = target_pose.pose.position.z 
    orientation = [
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w]

    new_pose = geometry_msgs.msg.PoseStamped()
    
    tf_orientation = tf.transformations.euler_from_quaternion(orientation)
    # print(shift* math.cos(tf_orientation[2]), shift* math.sin(tf_orientation[2]))

    x_shift = (shift * math.cos(tf_orientation[2]))
    y_shift = (shift * math.sin(tf_orientation[2]))

    #to pick from point closer to robot base
    if x < 0: 
        new_shift_x = x + x_shift
    else:
        new_shift_x = x - x_shift

    if y < 0: 
        new_shift_y = y + y_shift
    else:
        new_shift_y = y - y_shift

    new_pose.pose.position.x = new_shift_x
    new_pose.pose.position.y = new_shift_y
    new_pose.pose.position.z = z
    new_pose.pose.orientation = target_pose.pose.orientation
    
    new_pose.header.frame_id = 'world'


    return new_pose

def setio_client(pin_no, state_s=24):
    
    rospy.wait_for_service('ur_hardware_interface/set_io')
    # rospy.init_node('open_gripper')
    try:
        setio = rospy.ServiceProxy('ur_hardware_interface/set_io', SetIO)
        resp1 = setio(fun=1, pin=pin_no, state=state_s)
        rospy.sleep(1)
        resp1 = setio(fun=1, pin= pin_no, state=0)
        rospy.sleep(1)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# arm_group.set_pose_target([0.5,0.5,1,1,1,1])
# flag, plan, time, error = arm_group.plan()
# raw_input()
# arm_group.execute(plan)

arm_group.set_named_target("home")
flag, plan, time, error = arm_group.plan()
raw_input()
arm_group.execute(plan)
print('at home')


# arm_group.shift_pose_target(axis=1, value=0.1)
# arm_group.set_pose_target([0,0,0.1,0,0,0])
# arm_group.set_pose_reference_frame('wrist_3_link')
# flag, plan, time, error = arm_group.plan()
# arm_group.execute(plan)
# print('at pose')


# arm_group.set_named_target("home")
# arm_group.shift_pose_target(axis=2, value=0.1)
# flag, plan, time, error = arm_group.plan()
# arm_group.execute(plan)

pose = [-0.887,-0.69,0.225]
orientation = tf.transformations.quaternion_from_euler(0,0,1.0,'sxyz')

grasp_pose = geometry_msgs.msg.PoseStamped()
grasp_pose.header.frame_id = 'world'
grasp_pose.pose.position.x = pose[0]
grasp_pose.pose.position.y = pose[1]
grasp_pose.pose.position.z = pose[2]
grasp_pose.pose.orientation.x = orientation[1]
grasp_pose.pose.orientation.y = orientation[2]
grasp_pose.pose.orientation.z = orientation[3]
grasp_pose.pose.orientation.w = orientation[0]

print(grasp_pose)
new_pose = pre_grasp_pose(grasp_pose)

print(new_pose)
arm_group.set_pose_target(new_pose)
flag, plan, time, error = arm_group.plan()
raw_input()
arm_group.execute(plan)


print('grasp_pose',grasp_pose)
arm_group.set_pose_target(grasp_pose)
flag, plan, time, error = arm_group.plan()
raw_input()
arm_group.execute(plan)

setio_client(pin_no=16)

arm_group.set_pose_target(new_pose)
flag, plan, time, error = arm_group.plan()
raw_input()
arm_group.execute(plan)

print('at pose')
print(arm_group.get_current_pose())
