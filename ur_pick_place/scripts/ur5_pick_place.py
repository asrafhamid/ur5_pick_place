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
# from six.moves import input
from builtins import input

## END_SUB_TUTORIAL


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

#TODO: 
class Fruit:
  def __init__(self,name):
    self._name = name

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    model_coordinates = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
    # rospy.Subscriber('/camera/object_track', PoseStamped, self.obj_track)



    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())

    print("============ Printing robot state")
    current_pose = move_group.get_current_pose(eef_link).pose
    print(current_pose)
    print("")
    ## END_SUB_TUTORIAL

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

    self.sphere_img_orien = 0

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

    #  Pose Orientation - Fixed
    roll_angle = 0
    pitch_angle = 1.57
    yaw_angle = yaw
    quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)

    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.z = 1.4
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def constraint_goal(self,x=0.4,y=-0.1,z=0.4):
    move_group = self.move_group

    self.constraints = Constraints()
    ocm = OrientationConstraint()

    ocm.link_name = "panda_link7"
    ocm.header.frame_id = "panda_link0"
    ocm.orientation.w = 1.0
    ocm.absolute_x_axis_tolerance = 0.1
    ocm.absolute_y_axis_tolerance = 0.1
    ocm.absolute_z_axis_tolerance = 0.1
    ocm.weight = 1.0

    self.constraints.orientation_constraints.append(ocm)

    move_group.set_path_constraints(self.constraints)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    move_group.set_planning_time(10.0)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    
    move_group.clear_path_constraints()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def plan_goal(self,x,y,z):
    move_group = self.move_group

    #  Pose Orientation
    roll_angle = 0
    pitch_angle = 1.5708
    yaw_angle = 0
    quaternion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)

    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.z = 1.4
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.4
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.plan()
    return plan 

  def plan_cartesian(self,end_pose,scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group
    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []
    #split_path()
    wpose = move_group.get_current_pose().pose

    # waypoints = self.split_path(wpose,end_pose,5)

    wpose.position.x = end_pose.position.x
    wpose.position.y = end_pose.position.y
    wpose.position.z = end_pose.position.z
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
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

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan):
    
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.

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
    ## END_SUB_TUTORIAL

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

  def split_path(self,p1,p2,n=5):
    # redundant as cartesian paths splits path already

    wpose = copy.deepcopy(p1)

    x1,y1,z1 = p1.position.x,p1.position.y,p1.position.z
    x2,y2,z2 = p2.position.x,p2.position.y,p2.position.z

    points = []
    for i in range(1, n):
        a = float(i) / n             # rescale 0 < i < n --> 0 < a < 1
        x = (1 - a) * x1 + a * x2    # interpolate x coordinate
        y = (1 - a) * y1 + a * y2    # interpolate y coordinate
        z = (1 - a) * z1 + a * z2    # interpolate y coordinate

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        points.append(copy.deepcopy(wpose))
    
    return points


  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object("box")

    # wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


  # object tracking Callback 
  def obj_track(self,msg):
      # print(msg)
      try:
        msg.header.frame_id = "camera_depth_optical_frame"
        p=listener.transformPose("base_link",msg)
      except tf.TransformException as e:
        print(type(e))
      
      # print(p)
      self.sphere_img_pose = p.pose
      # euler = tf.transformations.euler_from_quaternion(self.sphere_img_pose.orientation)
      # print("obj angle: {}".format(self.sphere_img_pose.orientation.w))
      self.sphere_img_orien = msg.pose.orientation.w

  def transf_pose_arr(self,pose_arr):
    tf_pose_array = []
    pose = PoseStamped()
    # pose.header = pose_arr.header
    pose.header.frame_id = "camera_depth_optical_frame"

    for p in pose_arr.poses:
      pose.pose = p
      tf_pose = listener.transformPose("base_link",pose)
      tf_pose.pose.orientation.w = p.orientation.w 
      tf_pose_array.append(tf_pose.pose)

    print(tf_pose_array)
    return tf_pose_array




def main():
  zero_goal = [0, -pi/2, 0, -pi/2, 0, 0]
  observe_goal = [-0.27640452940659355, -1.5613947841166143, 0.8086120509001136, -0.8173772811698496, -1.5702185440399328, -0.2754254250487067]


  # FLAGS
  ex_plan = 'y'
  pick_only = True


  try:

    tutorial = MoveGroupPythonIntefaceTutorial()
    tutorial.detach_box()
    tutorial.remove_box()

    print("============ Press `Enter` to move to zero position (joint state goal) ...")
    input()
    tutorial.go_to_joint_state(zero_goal)

    observe_pose = (0.4, 0.0,0.7)
    place_pose = (-0.1,0.4,0.3)

    print("============ adding bounding box to the planning scene ...")
    tutorial.add_bbox()

    tutorial.go_to_joint_state(observe_goal)

    print("============ waiting for obj detect service ...")
    rospy.wait_for_service('/get_obj_clr')
    obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)
    pose = Pose()


    if pick_only:
      while not rospy.is_shutdown():

        tutorial.go_to_joint_state(observe_goal)

        print("============ Press `Enter` to move to ball ...")
        input()
        rospy.sleep(0.1)
        # sphere_pose = tutorial.model_coordinates("cricket_ball","").pose

        pose = obj_srv("blue")
        print(pose.poses.poses)

        if pose.poses.poses:
          pose_tf = tutorial.transf_pose_arr(pose.poses)

          for p in pose_tf:
            tutorial.go_to_pose_goal(p.position.x, p.position.y,0.18,p.orientation.w-1.5708)
            rospy.sleep(2.0)

        # print("(model state) sphere at x: {}, y: {}, z: {}".format(sphere_pose.position.x,sphere_pose.position.y,sphere_pose.position.z))
        # print("(visual) sphere at x: {}, y: {}, z: {}".format(tutorial.sphere_img_pose.position.x,tutorial.sphere_img_pose.position.y,tutorial.sphere_img_pose.position.z))
        # print("obj angle: {}".format(tutorial.sphere_img_orien))
        # eef_orien = tutorial.sphere_img_orien - 1.5708

        # tutorial.go_to_pose_goal(tutorial.sphere_img_pose.position.x, tutorial.sphere_img_pose.position.y,0.2,eef_orien)

        rospy.sleep(2.0)

    else:
      # ------- pick ------ START

      while not rospy.is_shutdown():

        # print("============ Press `Enter` to move to observe pose ..."
        # input()
        # print("============ moving to observe pose ..."
        # tutorial.go_to_pose_goal(0.4, 0.0,0.7)

        tutorial.go_to_joint_state(observe_goal)

        print("============ Press `Enter` to retrieve ball coordinates ...")
        input()
        rospy.sleep(0.1)

        # actual sphere coordinates
        sphere_pose = tutorial.model_coordinates("cricket_ball","").pose

        print("(model state) sphere at x: {}, y: {}, z: {}".format(sphere_pose.position.x,sphere_pose.position.y,sphere_pose.position.z))
        print("(visual) sphere at x: {}, y: {}, z: {}".format(tutorial.sphere_img_pose.position.x,tutorial.sphere_img_pose.position.y,tutorial.sphere_img_pose.position.z))

        print("obj angle: {}".format(tutorial.sphere_img_pose.orientation.w))

        sphere_pose.position.x = tutorial.sphere_img_pose.position.x
        sphere_pose.position.y = tutorial.sphere_img_pose.position.y

        print("============ adding a box to ball ...")

        tutorial.add_box(sphere_pose.position.x,sphere_pose.position.y)

        plan, fraction = tutorial.plan_cartesian(sphere_pose)
        tutorial.display_trajectory(plan)

        ex_plan = input("============ Execute plan? (y/n) ============ \n")
        print(ex_plan)

        # tutorial.execute_plan(plan) if ex_plan=='y' else sys.exit(0)
        tutorial.go_to_pose_goal(sphere_pose.position.x,sphere_pose.position.y,0.1)

        print("============ attaching Box to the robot ...")
        tutorial.attach_box()

        print("============ moving to place pose ...")
        tutorial.go_to_pose_goal(place_pose[0],place_pose[1],place_pose[2])

        print("============ detaching box from the robot ...")
        tutorial.detach_box()

        print("============ removing the box from the planning scene ...")
        tutorial.remove_box()

        # ------- pick ------ END

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':

  rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

  listener = tf.TransformListener()
  listener.waitForTransform("/base_link", "/camera_link", rospy.Time(0),rospy.Duration(4.0))

  main()
  rospy.spin()


