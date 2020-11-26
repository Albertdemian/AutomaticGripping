#!/usr/bin/env python2.7

from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf 
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur_msgs.srv import *

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

#initialize moveit commander
moveit_commander.roscpp_initialize(sys.argv)
#start ros node
rospy.init_node('Move_group_gripping',anonymous=True)

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


print(arm_group.get_current_pose())
robot_pose = arm_group.get_current_pose()
orientation = [
            robot_pose.pose.orientation.w,
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z]
print(tf.transformations.euler_from_quaternion(orientation))

'''
#retrieve current robot pose
current_pose = robot.get_current_state()
planning_frame = move_group.get_planning_frame()
eef_link = move_group.get_end_effector_link()
groups = robot.get_group_names()

print(planning_frame, eef_link, groups)

 #actuator groups that can be commanded by movit

arm_group.set_named_target("home")
flag, plan, time, error = arm_group.plan()
arm_group.execute(plan)


gripper_group.set_named_target("gripper_open")
gripper_group.go()

setio_client(pin_no=17)

arm_group.set_pose_target([0.5,0.5,1,1,1,1])
flag, plan, time, error = arm_group.plan()
arm_group.execute(plan)


gripper_group.set_named_target("gripper_closed")
gripper_group.go()

setio_client(pin_no=16)'''