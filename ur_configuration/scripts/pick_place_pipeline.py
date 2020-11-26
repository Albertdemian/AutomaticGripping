#!/usr/bin/env python 
from __future__ import print_function #has to be first line
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf 
import math
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def setio_client(pin_no, state_s=24):
    '''
    service call to control the gripper
    pin_no: 'Integer' pin number 
    state_s: 0,12 or 24 V
    '''
    
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



class pick_place(object):
    def __init__(self): 
        super(pick_place,self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_place_operation',anonymous=True)

        #import robot commander
        robot= moveit_commander.RobotCommander()

        #import planning scene
        scene = moveit_commander.PlanningSceneInterface()

        # moveit move group name 
        arm_name = 'arm'
        arm_group = moveit_commander.MoveGroupCommander(arm_name)
        arm_group.set_num_planning_attempts(50)
        arm_group.set_planning_time(10)
        arm_group.allow_replanning(True)

        gripper_name = 'hand'
        gripper_group = moveit_commander.MoveGroupCommander(gripper_name)

        #publisher to visualize planned trajectory 
        DisplayTraj = rospy.Publisher('/move_group/display_planned_path' ,
                                moveit_msgs.msg.DisplayTrajectory,
                                queue_size=20)

        #retrieve robot information
        current_pose = robot.get_current_state()
        planning_frame = arm_group.get_planning_frame()
        eef_link = arm_group.get_end_effector_link()
        groups = robot.get_group_names()


        self.robot = robot
        self.scene = scene
        self.arm = arm_group
        self.gripper = gripper_group
        self.DisplayTrajectory = DisplayTraj
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = groups 
        self.grasp_pose = geometry_msgs.msg.PoseStamped()
        self.place_pose = geometry_msgs.msg.PoseStamped()
        self.pre_grasp_pose = geometry_msgs.msg.PoseStamped()
        self.pre_place_pose = geometry_msgs.msg.PoseStamped()

    def add_box(self,pose,orient=[0,0,0,1], size=(0.1,0.1,0.1),box_name = "box1", timeout=4):
    
        #define scene 
        scene = self.scene

        #define pose type object
        box_pose = geometry_msgs.msg.Pose()
        #transform orientation from euler to Quanternion
        # orientation = tf.transformations.quaternion_from_euler(orient[0],orient[1],orient[2],'sxyz')

        box_pose.orientation.x = orient[0]
        box_pose.orientation.y = orient[1]
        box_pose.orientation.z = orient[2]
        box_pose.orientation.w = orient[3]

        box_pose.position.x = pose[0]
        box_pose.position.y = pose[1]
        box_pose.position.z = pose[2] 

        scene.add_box(box_name, box_pose, size=size)

    def add_plane(self, name, plane_pose, normal=(0,0,1), offset=0):
        '''
        Add plane constrain to the scene
        name: 'string' name of plane
        plane_pose: array of point on the plane [x,y,z]
        normal: 'tuble' normal vector (x,y,z)
        offset: offset distance along normal vector
        '''
        scene = self.scene
        
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = plane_pose[0]
        pose.pose.position.y = plane_pose[1]
        pose.pose.position.z = plane_pose[2]

        scene.add_plane(name, pose, normal, offset)

    def set_grasp_pose(self,pose, orientation, angle_coordinate = 0): 
        ''' 
        takes orientation in [roll, pitch, yaw]
        or quaternion [x,y,z,w ]
        angel coordinates 
        0: for euler 
        1: for quaternion
         '''

        grasp_pose = self.grasp_pose
        grasp_pose.header.frame_id = self.planning_frame
        grasp_pose.pose.position.x = pose[0]
        grasp_pose.pose.position.y = pose[1]
        grasp_pose.pose.position.z = pose[2]

        if angle_coordinate == 0:
            orientation = tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2])

        grasp_pose.pose.orientation.x = orientation[0]
        grasp_pose.pose.orientation.y = orientation[1]
        grasp_pose.pose.orientation.z = orientation[2]
        grasp_pose.pose.orientation.w = orientation[3] 



    def _side_pre_grasp_pose(self, shift=0.1):
        '''
        set top posture for vertical gripping approach 
        shift: distance away from object before approach
        '''
        target_pose = self.grasp_pose

        shift= -shift
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
        z = target_pose.pose.position.z 
        orientation = [
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w]

        new_pose = self.pre_grasp_pose
        
        tf_orientation = tf.transformations.euler_from_quaternion(orientation)
        # print(shift* math.cos(tf_orientation[2]), shift* math.sin(tf_orientation[2]))

        x_shift = (shift * math.sin(tf_orientation[2]))
        y_shift = (shift * math.cos(tf_orientation[2]))
        print(x_shift, y_shift)

        #to pick from point closer to robot base
        if x < 0: 
            new_shift_x = x + x_shift
        else:
            new_shift_x = x - x_shift

        if y < 0: 
            new_shift_y = y + y_shift
        else:
            new_shift_y = y - y_shift

        new_pose.header.frame_id = self.planning_frame
        new_pose.pose.position.x = new_shift_x
        new_pose.pose.position.y = new_shift_y
        new_pose.pose.position.z = z
        new_pose.pose.orientation = target_pose.pose.orientation
        
        new_pose.header.frame_id = 'world'


    
    def _top_pre_grasp_pose(self, shift=0.1):
        '''
        set top posture for vertical gripping approach
        shift: distance away from object before approach

        =========== FIX ME ==========
        Needs further checks about orientation
        '''
        target_pose  = self.grasp_pose
        new_pose  = self.pre_grasp_pose

        new_pose.pose.position = target_pose.pose.position
        new_pose.pose.position.z  -= shift
        new_pose.pose.orientation = target_pose.pose.orientation

    def set_pre_grasp_pose(self, side=0):
        '''
        sides:
        0: for top pick
        1: for side pick '''

        if side ==0: 
            self._top_pre_grasp_pose()
        elif side == 1: 
            self._side_pre_grasp_pose() 



    def _expand_quanterion(self,pose): 
        ''' takes type geometry_msgs.msg.PoseStamped()'''

        orientation = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w]

        return orientation

    def _put_back_quaternion(self,pose, angles): 
        '''
        Takes
        pose msg of type PoseStamped()
        angles: list of quaternion angles [x,y,z,w]        
        '''
        pose.pose.orientation.x = angles[0]
        pose.pose.orientation.y = angles[1]
        pose.pose.orientation.z = angles[2]
        pose.pose.orientation.w = angles[3]



    def closer_orientation(self,target_pose): 
        '''
        finds out if the target orientation of the target pose is closer 
        or if a value of 2*pi was subtracted
        '''

        arm_group = self.arm 
        current = arm_group.get_current_pose()

        #extract orientation vector from pose msg
        current_orientation = self._expand_quanterion(current)
        target_orientation = self._expand_quanterion(target_pose)

        #convert to euler angles
        current_orientation = tf.transformations.euler_from_quaternion(current_orientation,'sxyz')
        target_orientation = tf.transformations.euler_from_quaternion(target_orientation,'sxyz')

        
        #subtract pi from target orientation
        refined = np.subtract(target_orientation, 2*math.pi)
       
        #calculate angle distance to current orientation
        target_distance = np.absolute(np.subtract(current_orientation, target_orientation))
        refined_distance = np.absolute(np.subtract(current_orientation, refined))

        
        #a vector to take selected values
        new_target = [0,0,0]
        
        #check which is closer

        if target_distance[0] > refined_distance[0]:
            new_target[0] = refined[0]
        else: 
            new_target[0] =  target_orientation[0]

        if target_distance[1] > refined_distance[1]:
            new_target[1] = refined[1]
        else: 
            new_target[1] =  target_orientation[1]

        
        if target_distance[2] > refined_distance[2]:
            new_target[2] = refined[2]
        else: 
            new_target[2] =  target_orientation[2]
        
        #convert back to quaternion
        refined_orientation = tf.transformations.quaternion_from_euler(new_target[0], new_target[1], new_target[2],'sxyz')
        
        #update target pose
        self._put_back_quaternion(target_pose, refined_orientation)

    def go_to_pose(self,pose): 
        '''
        plan and execute trajectory to target pose
        '''

        arm_group = self.arm
        error = -1
        while error == -1:
            arm_group.set_pose_target(pose)
            flag, plan, time, error = arm_group.plan()
            arm_group.execute(plan)
        print(error)
    
    def go_home(self):
        '''
        to go to named "home" position
        '''
        arm_group = self.arm

        arm_group.set_named_target('home')
        flag, plan, time, error = arm_group.plan()
        arm_group.execute(plan)

    
    def get_current_pose(self):
        '''
        returns the current robot pose
        '''
        arm = self.arm
        pose = arm.get_current_pose()

        return pose

    # def plan_grasp(self, pose, orientation,angles_coordinate=1, grasp_side=1):
        
    #     self.set_grasp_pose(pose,orientation, angle_coordinate=angles_coordinate)
    #     self.set_pre_grasp_pose(side=grasp_side)
    #     arm_group = self.arm

    #     print('grasp pose \n', self.grasp_pose)
    #     arm_group.set_pose_target(self.grasp_pose)
    #     flag, plan1, time, error = arm_group.plan()
    #     arm_group.execute(plan1)
    #     # rospy.sleep(5)
    #     # arm_group.set_pose_target(self.grasp_pose)
    #     # flag, plan2, time, error = arm_group.plan()
        
    #     # rospy.sleep(5)
    #     # arm_group.execute(plan2)

    #     # print(arm_group.get_target_pose())
        


try: 
    grip_op = pick_place()
    grip_op.add_plane('floor', [0,0,0])
    
    pose = [-0.7157, 0.044,0.37588]
    # orientation = tf.transformations.quaternion_from_euler(0.5*math.pi,0,0,'sxyz')
    orientation = [0.646,0.286,-0.2858,0.6467]
    # grip_op.plan_grasp(pose, orientation, angles_coordinate=1, grasp_side=1)
    grip_op.go_home()
    grip_op.set_grasp_pose(pose, orientation,angle_coordinate=1)
    
    grip_op.set_pre_grasp_pose(side=1)
    grip_op.go_to_pose(grip_op.pre_grasp_pose)
    grip_op.go_to_pose(grip_op.grasp_pose)
    arm_group = grip_op.arm

    print('time', arm_group.get_planning_time())
    print(grip_op.grasp_pose)
    print(grip_op.pre_grasp_pose)
    print('current', grip_op.get_current_pose())

except rospy.ROSInterruptException:
    pass
except KeyboardInterrupt:
    pass



# try:
#     grip_op = pick_place()
#     grip_op.add_plane('floor', [0,0,0])

#     pose = [-0.887,-0.69,0.5]
#     orientation = tf.transformations.quaternion_from_euler(0,0,1.0,'sxyz')

#     grasp_pose = geometry_msgs.msg.PoseStamped()
#     grasp_pose.header.frame_id = 'world'
#     grasp_pose.pose.position.x = pose[0]
#     grasp_pose.pose.position.y = pose[1]
#     grasp_pose.pose.position.z = pose[2]
#     grasp_pose.pose.orientation.x = orientation[1]
#     grasp_pose.pose.orientation.y = orientation[2]
#     grasp_pose.pose.orientation.z = orientation[3]
#     grasp_pose.pose.orientation.w = orientation[0]

#     grip_op.closer_orientation(grasp_pose)
#     print(grasp_pose)
#     grip_op.go_to_pose(grasp_pose)

# except rospy.ROSInterruptException:
#     pass
# except KeyboardInterrupt:
#     pass

