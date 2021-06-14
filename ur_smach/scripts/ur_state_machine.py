#!/usr/bin/env python

import roslib
import rospy
import threading
import smach
import smach_ros
from std_msgs.msg import String
import rosservice
from obj_detection.srv import GetObject
import tf
from math import pi
import time
from geometry_msgs.msg import PointStamped, PoseStamped
from ur5_pick_place.ur5_pick_place import MoveGroupPythonIntefaceTutorial
from control_msgs.msg import FollowJointTrajectoryActionResult

# define state CheckRobotArm
class CheckRobotArm(smach.State):
    def __init__(self,listener):
        smach.State.__init__(self, outcomes=['init_robot_arm_state','check_robot_arm_state'])
        self.listener = listener

    def execute(self, userdata):
        rospy.loginfo('Executing state CheckRobotArm')
        self.listener.waitForTransform("/base_link", "/camera_link", rospy.Time(0),rospy.Duration(4.0))

        # rospy.wait_for_service('/get_obj_clr')
        if '/arm_controller/query_state' in rosservice.get_service_list():
            return 'init_robot_arm_state'
        else:
            return 'check_robot_arm_state'

# define state InitRobotArm
class InitRobotArm(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['observe_robot_arm_state','init_robot_arm_state'])
        rospy.Subscriber("arm_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.result_cb)
        self.result = -1
        self.move_grp = move_grp
        self.zero_goal = [0, -pi/2, 0, -pi/2, 0, 0]

    def execute(self, userdata):
        rospy.loginfo('Executing state InitRobotArm')
        self.move_grp.go_to_joint_state(self.zero_goal)
        
        if self.result == 3:
            return 'observe_robot_arm_state'
        else:
            return 'init_robot_arm_state'

    def result_cb(self,data):
        print(data.status.status)
        self.result = data.status.status

# define state ObserveRobotArm
class ObserveRobotArm(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['trigger_pick_place_task','observe_robot_arm_state'])
        rospy.Subscriber("arm_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.result_cb)
        self.result = -1
        self.move_grp = move_grp
        self.observe_goal = [-0.27640452940659355, -1.5613947841166143, 0.8086120509001136, -0.8173772811698496, -1.5702185440399328, -0.2754254250487067]

    def execute(self, userdata):
        rospy.loginfo('Executing state ObserveRobotArm')
        self.move_grp.go_to_joint_state(self.observe_goal)
        
        if self.result == 3:
            return 'trigger_pick_place_task'
        else:
            return 'observe_robot_arm_state'

    def result_cb(self,data):
        print("result: "+str(data.status.status))
        self.result = data.status.status


# define state TriggerPickAndPlace
class TriggerPickAndPlace(smach.State):
    def __init__(self,move_grp,listener):
        smach.State.__init__(self, outcomes=['plan_pick_state','trigger_pick_place_task'],output_keys=['target_pose','color'])
        self.obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)
        self.move_grp = move_grp
        self.pose = None
        self.color = ""
        self.poses_tf = []
        self.listener = listener
        rospy.Subscriber("object_colour", String, self.trigger_pick_and_place)

    def execute(self, userdata):
        rospy.loginfo('Executing state TriggerPickAndPlace')

        self.trigger = False
        time.sleep(10)

        if self.poses_tf:

            target_pose = self.move_grp.get_closest_coordinate(self.poses_tf[0])
            userdata.target_pose = target_pose
            userdata.color = self.color

            print("color: "+self.color)
            print("poses: "+str(self.poses_tf))
            return 'plan_pick_state'
        else:
            return 'trigger_pick_place_task'
    
    def trigger_pick_and_place(self,data):
        self.color = data.data
        print("color is  "+self.color)
        self.pose = self.obj_srv(self.color)
        self.poses_tf = self.move_grp.transf_pose_arr(self.pose.poses,self.listener)


# define state PlanPick
class PlanPick(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['pick_robot_arm_state','waiting_state'],
                                    input_keys=['target_pose','plan'],
                                    output_keys=['plan'])
        self.move_grp = move_grp
        self.pose = []

    def execute(self, userdata):
        rospy.loginfo('Executing state PlanPick')

        if userdata.target_pose:
            plan, fraction = self.move_grp.plan_cartesian(userdata.target_pose, True, True)
            p = userdata.target_pose
            p_orient = p.orientation.w

            if plan.joint_trajectory.points:
                print("plan is succesfull!")
                userdata.plan = plan
                return 'pick_robot_arm_state'
            else:
                print("plan failed!")
                return 'waiting_state' 
        else:
            return 'waiting_state'

# define state PickRobotArm
class PickRobotArm(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['pick_robot_arm_state','grab_gripper_state'],
                                    input_keys=['target_pose','plan'],)
        rospy.Subscriber("arm_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.pick_cb)
        self.result = -1
        self.move_grp = move_grp
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PickRobotArm')

        if userdata.target_pose:
            p = userdata.target_pose
            plan = userdata.plan
            p_orient = p.orientation.w

            self.move_grp.execute_plan(plan)
            # self.move_grp.go_to_pose_goal(p.position.x, p.position.y,0.20,p_orient-1.5708)
            self.move_grp.insert_eject(True)

        if self.counter >= 1:
            self.counter = 0
            return 'grab_gripper_state'
        else:
            return 'pick_robot_arm_state'

    def pick_cb(self,data):
        print("result: "+str(data.status.status))
        self.result = data.status.status
        if self.result == 3:
            self.counter += 1

# define state GrabGripper
class GrabGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grab_gripper_state','plan_place_state'])
        # rospy.Subscriber("arm_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.result_cb)
        # self.result = -1
        self.trigger = True

    def execute(self, userdata):
        rospy.loginfo('Executing state GrabGripper')

        if self.trigger:
            return 'plan_place_state'
        else:
            return 'grab_gripper_state'

    # def result_cb(self,data):
    #     print("result: "+str(data.status.status))
    #     self.result = data.status.status
    #     if self.result == 3:
    #         self.counter += 1

class ReleaseGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['release_gripper_state','ending_state'])
        # rospy.Subscriber("arm_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.result_cb)
        # self.result = -1
        self.trigger = True

    def execute(self, userdata):
        rospy.loginfo('Executing state ReleaseGripper')

        if self.trigger:
            return 'ending_state'
        else:
            return 'release_gripper_state'

    # def result_cb(self,data):
    #     print("result: "+str(data.status.status))
    #     self.result = data.status.status
    #     if self.result == 3:
    #         self.counter += 1

# define state PlanPlace
class PlanPlace(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['place_robot_arm_state','waiting_state'],
                                    input_keys=['place_poses','color'],
                                    output_keys=['plan'])
        self.move_grp = move_grp
        self.pose = []

    def execute(self, userdata):
        rospy.loginfo('Executing state PlanPick')

        if userdata.color!="na":
            place_pose = userdata.place_poses[userdata.color]
            print(place_pose)
            plan = self.move_grp.plan_goal_pose(place_pose)

            if plan.joint_trajectory.points:
                print("plan is succesfull!")
                userdata.plan = plan
                return 'place_robot_arm_state'
            else:
                print("plan failed!")
                return 'waiting_state' 
        else:
            return 'waiting_state'

# define state PlaceRobotArm
class PlaceRobotArm(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['place_robot_arm_state','release_gripper_state'],
                                    input_keys=['plan'],)
        rospy.Subscriber("arm_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.place_cb)
        self.result = -1
        self.move_grp = move_grp
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceRobotArm')

        plan = userdata.plan
        # print(plan)

        eject_plan = self.move_grp.insert_eject(False)
        self.move_grp.execute_plan(eject_plan)
        print("done eject")
        self.move_grp.execute_plan(plan)

        if self.counter >= 2:
            self.counter = 0
            return 'release_gripper_state'
        else:
            return 'place_robot_arm_state'

    def place_cb(self,data):
        print("result: "+str(data.status.status))
        print("counter: "+str(self.counter))
        self.result = data.status.status
        if self.result == 3:
            self.counter += 1

def main():
    rospy.init_node('ur_smach_state_machine')

    listener = tf.TransformListener()
    move_grp = MoveGroupPythonIntefaceTutorial()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.color = "na"
    sm.userdata.target_pose = None
    sm.userdata.plan = None 

    red_place = move_grp.def_pose(0.40,0.004,0.7,0.0,0.707,0.0,0.707)
    blue_place = move_grp.def_pose(0.214,-0.338,0.7,0.341,0.62,-0.34,0.62)
    sm.userdata.place_poses = {'red':red_place,'blue':blue_place}

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CheckRobotArm', CheckRobotArm(listener), 
                               transitions={'init_robot_arm_state':'InitRobotArm', 
                                            'check_robot_arm_state':'CheckRobotArm'})

        smach.StateMachine.add('InitRobotArm', InitRobotArm(move_grp), 
                               transitions={'observe_robot_arm_state':'ObserveRobotArm', 
                                            'init_robot_arm_state':'InitRobotArm'})

        smach.StateMachine.add('ObserveRobotArm', ObserveRobotArm(move_grp), 
                               transitions={'trigger_pick_place_task':'TriggerPickAndPlace', 
                                            'observe_robot_arm_state':'ObserveRobotArm'})                                
                                            
        smach.StateMachine.add('TriggerPickAndPlace', TriggerPickAndPlace(move_grp, listener), 
                               transitions={'plan_pick_state':'PlanPick',
                               'trigger_pick_place_task':'TriggerPickAndPlace'})
                    
        smach.StateMachine.add('PlanPick', PlanPick(move_grp), 
                               transitions={'pick_robot_arm_state':'PickRobotArm',
                               'waiting_state':'PlanPick'})

        smach.StateMachine.add('PickRobotArm', PickRobotArm(move_grp), 
                               transitions={'grab_gripper_state':'GrabGripper',
                               'pick_robot_arm_state':'PickRobotArm'})

        smach.StateMachine.add('GrabGripper', GrabGripper(), 
                               transitions={'plan_place_state':'PlanPlace',
                               'grab_gripper_state':'GrabGripper'})

        smach.StateMachine.add('PlanPlace', PlanPlace(move_grp), 
                               transitions={'place_robot_arm_state':'PlaceRobotArm',
                               'waiting_state':'PlanPlace'})

        smach.StateMachine.add('PlaceRobotArm', PlaceRobotArm(move_grp), 
                               transitions={'release_gripper_state':'ReleaseGripper',
                               'place_robot_arm_state':'PlaceRobotArm'})

        smach.StateMachine.add('ReleaseGripper', ReleaseGripper(), 
                               transitions={'ending_state':'ObserveRobotArm',
                               'release_gripper_state':'ReleaseGripper'})


    sis = smach_ros.IntrospectionServer('robot_arm_task_manager', sm, '/robot_arm_task_manager')
    sis.start()

    # Execute SMACH plan
    # outcome = sm.execute()
    # print("End : {}".format(outcome))

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target = sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    sis.stop()

if __name__ == '__main__':
    main()