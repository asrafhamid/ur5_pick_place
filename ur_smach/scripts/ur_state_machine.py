#!/usr/bin/env python3

from numpy.core.fromnumeric import resize
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
        plan = self.move_grp.go_to_joint_state(self.zero_goal)
        
        if plan:
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


# define state TriggerPickAndPlace
class TriggerPickAndPlace(smach.State):
    def __init__(self,move_grp,listener):
        smach.State.__init__(self, outcomes=['plan_pick_state','trigger_pick_place_task','exit'],output_keys=['target_pose','color'])
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
                    print(poses_tf)
                    target_pose = poses_tf[0]
                    target_pose.orientation.w = target_pose.orientation.w # keep orientation
                    print("TRANFORMED POSE IS {}".format(target_pose))
                    userdata.target_pose = target_pose
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
                                    input_keys=['target_pose','plan'],
                                    output_keys=['plan'])
        self.move_grp = move_grp
        self.pose = []

    def execute(self, userdata):
        rospy.loginfo('Executing state PlanPick')

        if userdata.target_pose:
            plan, fraction = self.move_grp.plan_pick(userdata.target_pose)
            p = userdata.target_pose
            # p_orient = p.orientation.w
            # print(p_orient)

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
        self.move_grp = move_grp
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PickRobotArm')

        if userdata.target_pose:
            plan = userdata.plan
            pick = self.move_grp.execute_plan(plan)

        if pick:
            return 'grab_gripper_state'
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
        smach.State.__init__(self, outcomes=['release_gripper_state','ending_state','observe_robot_arm_state'])
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
            # place_pose = userdata.place_poses["blue"]
            # print(place_pose)
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

# class GripperController:
#     def __init__(self):
#         print("GripperController")
#         self.connected = False
#         self.g_mode = GripperMode()
#         self.c_mode = 1 #current mode
#         self.g_req = GripperRequest()
#         self.modes = {"grap":0,"idle":1,"release":2}
#         self.gripper_pub = rospy.Publisher('/rochu/request',
#                                                    GripperRequest,
#                                                    queue_size=10)
#         self.g_sub = rospy.Subscriber("/rochu/state", GripperState, self.state_callback)
    
#     def gripper_request(self,mode,effort):

#         if self.connected:
#             rospy.loginfo("Sending {} request to gripper".format(mode))
#             self.g_mode.value = self.modes[mode]
#             self.g_req.name ="1"
#             self.g_req.effort = effort
#             self.g_req.request_mode = self.g_mode

#             self.gripper_pub.publish(self.g_req)
#             time.sleep(1)
#             rospy.loginfo("Gripper now in {} mode".format(self.c_mode))
#             return True
#         else:
#             rospy.loginfo("Gripper not connected: failed to send request".format(mode))
#             return False
    
#     def state_callback(self,data):
#         self.connected = data.connected
#         self.c_mode = data.current_mode.value

#     def gripper_state(self):
#         return self.connected, self.c_mode
        

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

        smach.StateMachine.add('PickRobotArm', PickRobotArm(move_grp), 
                               transitions={'grab_gripper_state':'GrabGripper',
                               'pick_robot_arm_state':'PickRobotArm'})

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