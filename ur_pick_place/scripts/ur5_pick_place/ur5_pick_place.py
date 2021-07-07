#!/usr/bin/env python3
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
from gazebo_msgs.srv import GetModelState, GetWorldProperties
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
    get_all_models = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)

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
    self.get_all_models = get_all_models
    self.sphere_img = []
    self.sphere_img_pose = PoseStamped().pose

    # get_all_models variable
    self.ignored_models = ['ground_plane','ConfTable','robot']
    self.min_dist = 0.5*0.5

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
    pitch_angle = 3.14
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

  def plan_goal_pose(self,pose_goal):
    move_group = self.move_group

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.plan()
    return plan 

  def plan_goal(self,x,y,z,yaw=0):
    move_group = self.move_group

    # roll_angle = 0
    # pitch_angle = 1.5708
    # yaw_angle = yaw
    roll_angle = 0
    pitch_angle = 3.14
    yaw_angle = 0
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

  def def_pose(self,x,y,z,ox,oy,oz,ow):
    pose = geometry_msgs.msg.Pose()
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    return pose

  def plan_pick(self,end_pose):
    move_group = self.move_group

    waypoints = []
    spot_a = copy.deepcopy(end_pose)
    spot_a.position.z +=0.40 # change: relative to obj depth

    # change: pitch is 3.14 instead of 1.57 why?
    q = quaternion_from_euler(0, 3.14, end_pose.orientation.w) #RPY
    spot_a.orientation.x = q[0]
    spot_a.orientation.y = q[1]
    spot_a.orientation.z = q[2]
    spot_a.orientation.w = q[3]

    waypoints.append(copy.deepcopy(spot_a))

    print(end_pose.position.z)
    end_pose.orientation = spot_a.orientation
    end_pose.position.z +=0.20 # change: relative to obj depth

    waypoints.append(copy.deepcopy(end_pose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    return plan, fraction

  def plan_place(self,end_pose):
    move_group = self.move_group

    waypoints = []
    spot_a = move_group.get_current_pose().pose
    # spot_a = end_pose
    spot_a.position.z = 0.5

    waypoints.append(copy.deepcopy(spot_a))
    waypoints.append(copy.deepcopy(end_pose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    return plan, fraction

  def plan_cartesian(self,end_pose,limit_z=False,rot_w=False):

    move_group = self.move_group

    waypoints = []
    wpose = move_group.get_current_pose().pose

    wpose.position.x = end_pose.position.x
    wpose.position.y = end_pose.position.y
    if limit_z:
      wpose.position.z = 0.30
    else:
      wpose.position.z = end_pose.position.z

    if rot_w:
      yaw = end_pose.orientation.w-1.5708
      roll_angle = 0
      pitch_angle = 1.5708
      quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw)
      wpose.orientation.x = quaternion[0]
      wpose.orientation.y = quaternion[1]
      wpose.orientation.z = quaternion[2]
      wpose.orientation.w = quaternion[3]

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction

  def insert_eject(self, insert=True, scale=1):

    move_group = self.move_group

    waypoints = []

    wpose = move_group.get_current_pose().pose
    if not insert:
      scale *= -1
    wpose.position.z -= scale * 0.2
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
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
    return move_group.execute(plan, wait=True)


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
    box_pose.header.frame_id = "base_link"

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
    wall1_pose.header.frame_id = "base_link"

    wall2_pose = geometry_msgs.msg.PoseStamped()
    wall2_pose.header.frame_id = "base_link"

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"

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


  def transf_pose_arr(self,pose_arr,listener):
    tf_pose_array = []
    pose = PoseStamped()
    pose.header.frame_id = "camera_depth_frame"

    for p in pose_arr.poses:
      pose.pose = p
      tf_pose = listener.transformPose("base_link",pose)
      tf_pose.pose.orientation.w = p.orientation.w 
      tf_pose_array.append(tf_pose.pose)

    print(tf_pose_array)
    return tf_pose_array

  def get_closest_coordinate(self,target_pose):
    all_model = self.get_all_models()
    corrected_target_pose = Pose()
    correct_object = None
    closest_distance = 9999.0
    diff = 0
    for object in all_model.model_names:
      if object in self.ignored_models:
        continue
      object_pose = self.model_coordinates(object,"").pose
      diff = (object_pose.position.x - target_pose.position.x)**2 + (object_pose.position.y - target_pose.position.y)**2
      if diff < closest_distance:
        closest_distance = diff
        corrected_target_pose = object_pose
        correct_object = object
    if diff >self.min_dist:
      return target_pose
    print("Corrected object:",correct_object)
    return corrected_target_pose

def trigger_pick_and_place(data):
  print("-------- Started --------")
  try:
    pose = obj_srv(data.data)
    print(pose.poses.poses)

    if pose.poses.poses:
      pose_tf = tutorial.transf_pose_arr(pose.poses)

      for p in pose_tf:
        previous_orientation = p.orientation.w
        p = tutorial.get_closest_coordinate(p)
        tutorial.go_to_pose_goal(p.position.x, p.position.y,p.position.z,previous_orientation-1.5708)
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
  observe_goal = [-0.27640452940659355, -1.5613947841166143, 0.8086120509001136, -0.8173772811698496, -1.5702185440399328, -0.1404254250487067]

  tutorial = MoveGroupPythonIntefaceTutorial()
  tutorial.detach_box()
  tutorial.remove_box()

  print("============ Press `Enter` to move to zero position (joint state goal) ...")
  input()
  # tutorial.go_to_joint_state(zero_goal)

  print("============ adding bounding box to the planning scene ...")
  tutorial.add_bbox()

  tutorial.go_to_joint_state(observe_goal)

  print("============ waiting for obj detect service ...")
  rospy.wait_for_service('/get_obj_clr')
  obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)

  # rospy.Subscriber("object_colour", String, trigger_pick_and_place)

  print("Ready to perform pick and place.")
  input()
  # tutorial.go_to_pose_goal(0.2771494191599483,-0.1684270975009395,0.35190190868870065)
  
  pose_array = obj_srv("red")
  print(pose_array)

  poses_tf = tutorial.transf_pose_arr(pose_array.poses,listener)

  if poses_tf:
    print(poses_tf)
    target_pose = poses_tf[0]
    target_pose.orientation.w = target_pose.orientation.w # keep orientation
    print("TRANFORMED POSE IS {}".format(target_pose))
    plan,_ = tutorial.plan_pick(target_pose)
    tutorial.execute_plan(plan)


                

  while not rospy.is_shutdown():
    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      exit()
    except KeyboardInterrupt:
      exit()
