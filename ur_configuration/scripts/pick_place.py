#!/usr/bin/env python2.7

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

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True



  
  

class GripOP(object):
  def __init__(self):
    super(GripOP,self).__init__()

    #initialize moveit commander
    moveit_commander.roscpp_initialize(sys.argv)
    #start ros node
    rospy.init_node('Move_group_gripping',anonymous=True)

    # import robot commander
    robot= moveit_commander.RobotCommander()

    #import planning scene
    scene = moveit_commander.PlanningSceneInterface()

    # moveit move group name 
    group_name = 'manipulator'
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_num_planning_attempts(5)

    #publisher to visualize planned trajectory 
    DisplayTraj = rospy.Publisher('/move_group/display_planned_path' ,
                                moveit_msgs.msg.DisplayTrajectory,
                                queue_size=20)

    #retrieve current robot pose
    current_pose = robot.get_current_state()
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    groups = robot.get_group_names()

    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.DisplayTrajectory = DisplayTraj
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = groups

    self.initial_pose = current_pose
    self.collision_objs =[]
    self.grasps = moveit_msgs.msg.Grasp()
    self.grasp_retreat = moveit_msgs.msg.Grasp()
    self.place = moveit_msgs.msg.PlaceLocation()
    self.base_link = 'base_link'

  def _add_box(self,pose,orient=[0,0,0], size=(0.1,0.1,0.1),box_name = "box1", timeout=4):
    
    #define scene 
    scene = self.scene

    #define pose type object
    box_pose = geometry_msgs.msg.PoseStamped()
    #transform orientation from euler to Quanternion
    orientation = tf.transformations.quaternion_from_euler(orient[0],orient[1],orient[2],'sxyz')

    box_pose.header.frame_id = self.base_link
    box_pose.pose.orientation.x = orientation[0]
    box_pose.pose.orientation.y = orientation[1]
    box_pose.pose.orientation.z = orientation[2]
    box_pose.pose.orientation.w = orientation[3]

    box_pose.pose.position.x = pose[0]
    box_pose.pose.position.y = pose[1]
    box_pose.pose.position.z = pose[2] 

    scene.add_box(box_name, box_pose, size=size)


  def _add_plane(self, name, plane_pose, normal=(0,0,1), offset=0):
    scene = self.scene
    
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = self.base_link
    pose.pose.position.x = plane_pose[0]
    pose.pose.position.y = plane_pose[1]
    pose.pose.position.z = plane_pose[2]

    scene.add_plane(name, pose, normal, offset)

  def add_collision_obj(self, object_type ,name, dimensions, pose, orientation):
    '''
    Creates a collision object in scene
    arguments: 
    object_type : type of object specified below 
      to add box, keys are: 'BOX','Box','box', 'B', 'b'
      to add plane, keys are: 'plane', 'Plane', 'PLANE', 'P', 'p' 

    name: specify name for this object
    dimensions: 
      in case of box: it represents the three dimensions of the box
      in case of plane: it represents the normal vector of the plane

    pose: center pose of either of these objects
    orientation: orientation of object
    '''
    
    box_keys = ['BOX','Box','box', 'B', 'b']
    plane_keys = ['plane', 'Plane', 'PLANE', 'P', 'p']

    if object_type in  box_keys:
      self._add_box(pose=pose, orient=orientation, size=dimensions, box_name=name )

    elif object_type in plane_keys: 
      self._add_plane(name, pose, dimensions)




  def Grasp(self, pose, orientation, shift=0.1):
    print("_________",pose)
    grasp = self.grasps

    print(self.move_group.get_current_pose().pose)
    
    orientation = tf.transformations.quaternion_from_euler(pi/2,pi/2,-pi/4,'sxyz')
    print('orien', orientation)
    grasp_pose = geometry_msgs.msg.PoseStamped()
    grasp_pose.header.frame_id = self.base_link
    grasp_pose.pose.position.x = pose[0]
    grasp_pose.pose.position.y = pose[1]
    grasp_pose.pose.position.z = pose[2]
    grasp_pose.pose.orientation.x = orientation[0]
    grasp_pose.pose.orientation.y = orientation[1]
    grasp_pose.pose.orientation.z = orientation[2]
    grasp_pose.pose.orientation.w = orientation[3]

    grasp.grasp_pose = grasp_pose

  def pre_grasp_approach(self):
    grasp = self.grasps
    grasp.pre_grasp_approach.direction.header.frame_id = self.base_link
    grasp.pre_grasp_approach.direction.vector.z = -1
    # grasp.pre_grasp_approach.direction.vector.y = 0.5
    grasp.pre_grasp_approach.desired_distance = 0.1


  def post_grasp_retreat(self):
    grasp = self.grasps
    grasp.post_grasp_retreat.direction.header.frame_id = self.base_link
    grasp.post_grasp_retreat.direction.vector.z = 1
    grasp.post_grasp_retreat.desired_distance = 0.1


  def place_pose(self, pose, orientation, shift=0.1): 
    place = self.place
    place.place_pose.header.frame_id = self.base_link
    place.place_pose.pose.position.x=pose[0]
    place.place_pose.pose.position.y=pose[1]
    place.place_pose.pose.position.z=pose[2] 

  def pre_place_approach(self):
    place =self.place
    place.pre_place_approach.direction.header.frame_id=self.base_link
    place.pre_place_approach.direction.vector.z = 1
    place.pre_place_approach.desired_distance = 0.1

  def post_place_retreat(self):
    place = self.place
    place.post_place_retreat.direction.header.frame_id = self.base_link
    place.post_place_retreat.direction.vector.z = 1
    place.post_place_retreat.desired_distance = 0.2



  

  def plan_grasp(self, grasp_id, pose, orientation ):
    
    self.grasps.id = grasp_id
    self.Grasp(pose,1)
    self.pre_grasp_approach()
    self.post_grasp_retreat()
  
  def plan_place(self, place_id ,pose, orientation):
    self.place.id = place_id
    self.place_pose(pose, 1)
    self.pre_place_approach()
    self.post_place_retreat()

  def plan_path(self):
    move_group = self.move_group
    
    move_group.pick("box2", self.grasps, plan_only=False)
    move_group.attach_object('box2')
    
    move_group.place("box2", self.place, plan_only=False)
    move_group.detach_object("box2")


    


def grip_and_place_demo():
  try:
    grip_operation = GripOP()
    box_pose = [0.5,0.5,0]
    # add_box(box_pose grip_operation.scene)
    grip_operation.add_collision_obj('plane', 'floor', pose=[0,0,0], dimensions = (0,0,1), orientation=1)
    # grip_operation.add_collision_obj('box', 'box1', pose=[0.5,0.1,-0.05],orientation=[0,0,pi/2], dimensions=[0.8,0.8,0.1] )
    grip_operation.add_collision_obj('box', 'box2', pose=[0.3,-0.5,0.1],orientation=[0,0,pi/2], dimensions=[0.1,0.1,0.1] )
    rospy.sleep(0.1)


    grip_operation.plan_grasp('gripbox', [0.3,-0.5,0.2],1)
    rospy.sleep(1)
    grip_operation.plan_place('placebox',[0, 0.5,0.7],1)
    print(grip_operation.place)
    grip_operation.plan_path()
   
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return






if __name__ == "__main__":

  grip_and_place_demo()
  












