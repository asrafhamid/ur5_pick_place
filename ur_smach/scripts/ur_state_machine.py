#!/usr/bin/env python3

from numpy.core.fromnumeric import resize
import copy
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
from ur5_pick_place.ur5_pick_place import MoveGroupPythonIntefaceTutorial, GripperController
from control_msgs.msg import FollowJointTrajectoryActionResult, GripperCommandAction,GripperCommandGoal
import actionlib
from rochu_gripper_msgs.msg import GripperRequest, GripperMode, GripperState

# define state CheckRobotArm
class CheckRobotArm(smach.State):
    def __init__(self,listener):
        smach.State.__init__(self, outcomes=['init_robot_arm_state','check_robot_arm_state'])
        self.listener = listener

    def execute(self, userdata):
        rospy.loginfo('Executing state CheckRobotArm')
        self.listener.waitForTransform("/base_link", "/camera_link", rospy.Time(0),rospy.Duration(4.0))

        rospy.wait_for_service('/get_obj_clr')
        

        # if '/arm_controller/query_state' in rosservice.get_service_list():
        if '/scaled_pos_joint_traj_controller/query_state' in rosservice.get_service_list():
            return 'init_robot_arm_state'
        else:
            return 'check_robot_arm_state'

# define state InitRobotArm
class InitRobotArm(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['observe_robot_arm_state','init_robot_arm_state'])
        self.result = -1
        self.move_grp = move_grp
        self.zero_goal = [0, -pi/2, 0, -pi/2, 0, 0]

    def execute(self, userdata):
        rospy.loginfo('Executing state InitRobotArm')
        self.move_grp.detach_box()
        self.move_grp.remove_box()

        self.move_grp.add_bbox()
        # plan = self.move_grp.go_to_joint_state(self.zero_goal)
        
        if True:
            return 'observe_robot_arm_state'
        else:
            return 'init_robot_arm_state'


# define state ObserveRobotArm
class ObserveRobotArm(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['trigger_pick_place_task','observe_robot_arm_state'])
        self.result = -1
        self.move_grp = move_grp

        #TODO: move this observe_goal to move_grp.def_pose
        self.observe_goal = [-0.27640452940659355, -1.5613947841166143, 0.8086120509001136, -0.8173772811698496, -1.5702185440399328, -0.1404254250487067]

    def execute(self, userdata):
        rospy.loginfo('Executing state ObserveRobotArm')
        plan = self.move_grp.go_to_joint_state(self.observe_goal)
        
        if plan:
            return 'trigger_pick_place_task'
        else:
            return 'observe_robot_arm_state'

# define state GetObjPos
class GetObjPos(smach.State):
    def __init__(self,move_grp,listener):
        smach.State.__init__(self, outcomes=['trigger_pick_place_task','get_obj_pos','plan_pick_state'],
                                    input_keys=['color'],
                                    output_keys=['target_pose','color'])
        self.move_grp = move_grp
        self.listener = listener
        self.all_clr =['red','blue']
        self.obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)
        self.poses = None
        self.poses_tf = []

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjPos')

        if not userdata.color or userdata.color not in self.all_clr:
            return 'trigger_pick_place_task'
        
        data = self.obj_srv(userdata.color)
        poses_tf = self.move_grp.transf_pose_arr(data.poses,self.listener)

        if poses_tf:
            # print(poses_tf)
            target_pose = poses_tf[0]
            target_pose.position.z += 0.18 # HARDCODED GRIPPER HEIGHT 
            userdata.target_pose = copy.deepcopy(target_pose)
            # userdata.color = clr
            return 'plan_pick_state'
        else:
            return 'get_obj_pos'

        

# define state TriggerPickAndPlace
class TriggerPickAndPlace(smach.State):
    def __init__(self,move_grp,listener):
        smach.State.__init__(self, outcomes=['plan_pick_state','trigger_pick_place_task','exit'],
                                    input_keys=['target_pose','color','phase_temp'],
                                    output_keys=['target_pose','color'])
        self.obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)
        self.move_grp = move_grp
        self.poses = None
        self.user_clr = ""
        self.arm_clr = []
        self.all_clr =['red','blue']
        self.poses_tf = []
        self.listener = listener
        self.counter = 0
        rospy.Subscriber("object_colour", String, self.trigger_pick_and_place)

    def execute(self, userdata):
        rospy.loginfo('Executing state TriggerPickAndPlace')

        # self.trigger = False
        # Question: Why sleep in every execute? to save cpu usage? [Tested: from around 85% to 35%]
        time.sleep(5)

        # Question: Why terminate in this way?
        # if self.counter >= 5:
        #     return 'exit'
        
        if self.user_clr:
            for clr in self.arm_clr:
                data = self.obj_srv(clr)
                if not data.success:
                    continue
                
                poses_tf = self.move_grp.transf_pose_arr(data.poses,self.listener)
                if poses_tf:
                    # print(poses_tf)
                    target_pose = poses_tf[0]
                    target_pose.position.z += 0.18 # HARDCODED GRIPPER HEIGHT 
                    userdata.target_pose = copy.deepcopy(target_pose)
                    userdata.color = clr
                    self.counter = 0
                    return 'plan_pick_state'
            self.user_clr = ""
            self.arm_clr = []
        self.counter+=1
        print(self.counter)
        return 'trigger_pick_place_task'
                    
    
    def trigger_pick_and_place(self,data):
        self.user_clr = data.data
        print("color is "+self.user_clr)

        if self.user_clr == 'all':
            self.arm_clr.extend(self.all_clr)
        elif self.user_clr in self.all_clr:
            self.arm_clr.append(self.user_clr)
        else:
            print("Invalid color choice!")


# define state PlanPick
class PlanPick(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['pick_robot_arm_state','waiting_state'],
                                    input_keys=['target_pose','plan','phase_temp'],
                                    output_keys=['plan','target_pose','phase_temp'])
        self.move_grp = move_grp
        self.pose = []

    def execute(self, userdata):
        rospy.loginfo('Executing state PlanPick')
        rospy.loginfo('target z is {}'.format(userdata.target_pose.position.z))

        curr_pose = self.move_grp.move_group.get_current_pose().pose

        curr_pose.position.x = userdata.target_pose.position.x
        curr_pose.position.y = userdata.target_pose.position.y
        curr_pose.orientation = userdata.target_pose.orientation

        if curr_pose.position.z > 0.18:
            curr_pose.position.z -= 0.1

        plan = None

        if userdata.target_pose:
            
            if userdata.phase_temp > 0:
                plan = self.move_grp.plan_goal_pose(curr_pose)[1]
            elif userdata.phase_temp == 0:
                plan, fraction = self.move_grp.plan_pick(userdata.target_pose)
            else:
                print("invalid state!")
                return 'waiting_state'

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
    def __init__(self,move_grp,listener):
        smach.State.__init__(self, outcomes=['pick_robot_arm_state','grab_gripper_state','plan_pick_state'],
                                    input_keys=['target_pose','plan','color','phase_temp','phase_fix'],output_keys=['target_pose','color','phase_temp'])
        self.obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)
        self.move_grp = move_grp
        self.counter = 0
        self.phase = 0
        self.listener = listener

    def execute(self, userdata):
        rospy.loginfo('Executing state PickRobotArm')
        pick = False

        if userdata.target_pose:
            plan = userdata.plan
            pick = self.move_grp.execute_plan(plan)

        print("pick_arm in phase: {}".format(userdata.phase_temp))
        if pick :
            if  userdata.phase_temp > 0:
                userdata.phase_temp -= 1
                 
                # get obj position again after moving arm
                data = self.obj_srv(userdata.color)
                poses_tf = self.move_grp.transf_pose_arr(data.poses,self.listener)
                if poses_tf:
                    target_pose = poses_tf[0]
                    target_pose.position.z += 0.18 # HARDCODED GRIPPER HEIGHT 
                    userdata.target_pose = copy.deepcopy(target_pose)

                return 'plan_pick_state'
            elif userdata.phase_temp == 0:
                userdata.phase_temp = userdata.phase_fix # reset
                return 'grab_gripper_state'
            else:
                return 'pick_robot_arm_state'
        else:
            return 'pick_robot_arm_state'


# define state GrabGripper
class GrabGripper(smach.State):
    def __init__(self,gripper_client):
        smach.State.__init__(self, outcomes=['grab_gripper_state','plan_place_state'])
        # rospy.Subscriber("arm_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.result_cb)
        # self.result = -1
        self.trigger = True
        self.gripper_controller = gripper_client

    def execute(self, userdata):
        rospy.loginfo('Executing state GrabGripper')
        
        conn,mode = self.gripper_controller.gripper_state()

        if conn:
            res=self.gripper_controller.gripper_request("grab",100)
            if res:
                return 'plan_place_state'
            else:
                return 'grab_gripper_state'


class ReleaseGripper(smach.State):
    def __init__(self,gripper_client):
        smach.State.__init__(self, outcomes=['release_gripper_state','ending_state'])
        self.trigger = True
        self.gripper_controller = gripper_client
        # self.init = True

    def execute(self, userdata):
        rospy.loginfo('Executing state ReleaseGripper')
        
        conn,mode = self.gripper_controller.gripper_state()
        if conn:
            res=self.gripper_controller.gripper_request("release",100)
            if res:
                return 'ending_state'
            else:
                return 'release_gripper_state'


# define state PlanPlace
class PlanPlace(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['place_robot_arm_state','waiting_state'],
                                    input_keys=['place_poses','color'],
                                    output_keys=['plan'])
        self.move_grp = move_grp
        self.pose = []

    def execute(self, userdata):
        rospy.loginfo('Executing state PlanPlace')

        if userdata.color!="na":
            place_pose = userdata.place_poses[userdata.color]
            plan,fraction = self.move_grp.plan_place(place_pose)

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
        self.result = -1
        self.move_grp = move_grp
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceRobotArm')

        plan = userdata.plan
        place = self.move_grp.execute_plan(plan)
        # print(plan)

        if place:
            return 'release_gripper_state'
        else:
            return 'place_robot_arm_state'
        

def main():
    rospy.init_node('ur_smach_state_machine')

    listener = tf.TransformListener()
    move_grp = MoveGroupPythonIntefaceTutorial()
    gripper_controller = GripperController()
    time.sleep(1) #hax for gripper_controller bug

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.color = "na"
    sm.userdata.target_pose = None
    sm.userdata.plan = None 
    sm.userdata.phase_fix = 2
    sm.userdata.phase_temp = sm.userdata.phase_fix

    blue_place = move_grp.def_pose(0.44,-0.35,0.65,0.5,0.5,-0.5,0.5)
    red_place = move_grp.def_pose(0.86,-0.55,0.65,-0.5,0.5,0.5,0.5)
    sm.userdata.place_poses = {'red':red_place,'blue':blue_place}

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CheckRobotArm', CheckRobotArm(listener), 
                               transitions={'init_robot_arm_state':'InitRobotArm', 
                                            'check_robot_arm_state':'CheckRobotArm'})

        smach.StateMachine.add('InitRobotArm', InitRobotArm(move_grp), 
                               transitions={'observe_robot_arm_state':'ReleaseGripper', 
                                            'init_robot_arm_state':'InitRobotArm'})

        smach.StateMachine.add('ObserveRobotArm', ObserveRobotArm(move_grp), 
                               transitions={'trigger_pick_place_task':'TriggerPickAndPlace', 
                                            'observe_robot_arm_state':'ObserveRobotArm'})                                
                                            
        smach.StateMachine.add('TriggerPickAndPlace', TriggerPickAndPlace(move_grp, listener), 
                               transitions={'plan_pick_state':'PlanPick',
                               'trigger_pick_place_task':'TriggerPickAndPlace',
                               'exit':'outcome4'})
                    
        smach.StateMachine.add('PlanPick', PlanPick(move_grp), 
                               transitions={'pick_robot_arm_state':'PickRobotArm',
                               'waiting_state':'PlanPick'})

        smach.StateMachine.add('PickRobotArm', PickRobotArm(move_grp,listener), 
                               transitions={'grab_gripper_state':'GrabGripper',
                               'pick_robot_arm_state':'PickRobotArm',
                               'plan_pick_state':'PlanPick'})

        smach.StateMachine.add('GrabGripper', GrabGripper(gripper_controller), 
                               transitions={'plan_place_state':'PlanPlace',
                               'grab_gripper_state':'GrabGripper'})

        smach.StateMachine.add('PlanPlace', PlanPlace(move_grp), 
                               transitions={'place_robot_arm_state':'PlaceRobotArm',
                               'waiting_state':'PlanPlace'})

        smach.StateMachine.add('PlaceRobotArm', PlaceRobotArm(move_grp), 
                               transitions={'release_gripper_state':'ReleaseGripper',
                               'place_robot_arm_state':'PlaceRobotArm'})

        smach.StateMachine.add('ReleaseGripper', ReleaseGripper(gripper_controller), 
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