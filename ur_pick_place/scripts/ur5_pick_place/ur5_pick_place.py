#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import random
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
import tf
from obj_detection.srv import GetObject
from builtins import input


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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    model_coordinates = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()
    print("============ End effector link: %s" % eef_link)
    print("============ Planning frame: %s" % planning_frame)
    print("============ Available Planning Groups:", robot.get_group_names())
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("============ Printing robot state")
    current_pose = move_group.get_current_pose(eef_link).pose
    print(current_pose)
    print("")

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.model_coordinates = model_coordinates
    self.sphere_img = []
    self.sphere_img_pose = PoseStamped().pose


  def go_to_joint_state(self,joint_goal):
    move_group = self.move_group

    move_group.go(joint_goal, wait=True)
    
    # ensure no residual movement
    move_group.stop()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self,x,y,z,yaw=0):
    move_group = self.move_group

    roll_angle = 0
    pitch_angle = 1.57
    yaw_angle = yaw
    quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    # ensure no residual movement
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_goal(self,x,y,z,yaw=0):
    move_group = self.move_group

    roll_angle = 0
    pitch_angle = 1.5708
    yaw_angle = yaw
    quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.plan()
    return plan 


  def plan_cartesian_path(self, scale=1):

    move_group = self.move_group

    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_box(self, *args):
    timeout=4
    scene = self.scene

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"

    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = args[0]
    box_pose.pose.position.y = args[1]
    box_pose.pose.position.z = 0.05
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def add_bbox(self, timeout=4):
    # Create boundary box environment
    box_name = self.box_name
    scene = self.scene

    wall1_pose = geometry_msgs.msg.PoseStamped()
    wall1_pose.header.frame_id = "world"

    wall2_pose = geometry_msgs.msg.PoseStamped()
    wall2_pose.header.frame_id = "world"

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"

    wall1_pose.pose.orientation.w = 1.0
    wall1_pose.pose.position.x = -0.25
    wall1_pose.pose.position.y = -0.5
    wall1_pose.pose.position.z = 0.5 # right wall

    wall2_pose.pose.orientation.w = 1.0
    wall2_pose.pose.position.x = -0.65
    wall2_pose.pose.position.y = 0.0
    wall2_pose.pose.position.z = 0.5 # back wall

    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = -0.05 # base

    wall1_name = "box1"
    wall2_name = "box2"
    box_name = "base"

    scene.add_box(wall1_name, wall1_pose, size=(1, 0.2, 1))
    scene.add_box(wall2_name, wall2_pose, size=(0.2, 0.8, 1))
    scene.add_box(box_name, box_pose, size=(1.5, 1.5, 0.1))

    self.box_name=box_name
    self.box_names = [box_name, wall1_name, wall2_name]
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = 'endeffector'
    touch_links = robot.get_link_names(group=grasping_group)

    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    # wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)

    # wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object("box")

    # wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


  def transf_pose_arr(self,pose_arr):
    tf_pose_array = []
    pose = PoseStamped()
    pose.header.frame_id = "camera_depth_optical_frame"

    for p in pose_arr.poses:
      pose.pose = p
      tf_pose = listener.transformPose("base_link",pose)
      tf_pose.pose.orientation.w = p.orientation.w 
      tf_pose_array.append(tf_pose.pose)

    print(tf_pose_array)
    return tf_pose_array

def trigger_pick_and_place(data):
  print("-------- Started --------")
  try:
    pose = obj_srv(data.data)
    print(pose.poses.poses)

    if pose.poses.poses:
      pose_tf = tutorial.transf_pose_arr(pose.poses)

      for p in pose_tf:
        tutorial.go_to_pose_goal(p.position.x, p.position.y,0.18,p.orientation.w-1.5708)
        rospy.sleep(2.0)
      tutorial.go_to_joint_state(observe_goal)

    print("-------- Finished --------")
    rospy.sleep(2.0)
  except Exception as e:
    print(e)


if __name__ == '__main__':

  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

  listener = tf.TransformListener()
  listener.waitForTransform("/base_link", "/camera_link", rospy.Time(0),rospy.Duration(4.0))

  # TODO: move into class
  # Initialization
  zero_goal = [0, -pi/2, 0, -pi/2, 0, 0]
  observe_goal = [-0.27640452940659355, -1.5613947841166143, 0.8086120509001136, -0.8173772811698496, -1.5702185440399328, -0.2754254250487067]

  tutorial = MoveGroupPythonIntefaceTutorial()
  tutorial.detach_box()
  tutorial.remove_box()

  print("============ Press `Enter` to move to zero position (joint state goal) ...")
  input()
  tutorial.go_to_joint_state(zero_goal)

  print("============ adding bounding box to the planning scene ...")
  tutorial.add_bbox()

  tutorial.go_to_joint_state(observe_goal)

  print("============ waiting for obj detect service ...")
  rospy.wait_for_service('/get_obj_clr')
  obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)

  rospy.Subscriber("object_colour", String, trigger_pick_and_place)

  print("Ready to perform pick and place.")

  while not rospy.is_shutdown():
    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      exit()
    except KeyboardInterrupt:
      exit()
