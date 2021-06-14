#!/usr/bin/env python

import roslib
import rospy
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
    def __init__(self):
        smach.State.__init__(self, outcomes=['init_robot_arm_state','check_robot_arm_state'])
        self.listener = tf.TransformListener()

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
        print("result: "+data.status.status)
        self.result = data.status.status


# define state TriggerPickAndPlace
class TriggerPickAndPlace(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['plan_pick_state','trigger_pick_place_task'],output_keys=['color'])
        self.obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)
        self.move_grp = move_grp
        self.pose = None
        self.color = ""
        self.poses_tf = []
        rospy.Subscriber("object_colour", String, self.trigger_pick_and_place)

    def execute(self, userdata):
        rospy.loginfo('Executing state TriggerPickAndPlace')

        self.trigger = False
        time.sleep(10)

        if self.poses_tf:

            target_pose = self.move_grp.get_closest_coordinate(self.pose_tf[0])
            userdata.target_pose = target_pose

            print("color: "+self.color)
            print("poses: "+str(self.pose_tf))
            return 'plan_pick_state'
        else:
            return 'trigger_pick_place_task'
    
    def trigger_pick_and_place(self,data):
        self.color = data.data
        self.pose = self.obj_srv(self.color)
        self.poses_tf = self.move_grp.transf_pose_arr(self.pose.poses)


# define state PlanPick
class PlanPick(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['pick_robot_arm_state','waiting_state'],output_keys=['color'])
        self.move_grp = move_grp
        self.pose = []

    def execute(self, userdata):
        rospy.loginfo('Executing state PlanPick')

        if userdata.target_pose:
            plan, fraction = self.move_grp.plan_cartesian(userdata.target_pose)
            p = userdata.target_pose
            p_orient = p.orientation.w

            if plan.joint_trajectory.points:
                print("plan is succesfull!")
                userdata.plan = plan
                # self.move_grp.execute_plan(plan)
                # self.move_grp.go_to_pose_goal(p.position.x, p.position.y,0.20,p_orient-1.5708)
                return 'pick_robot_arm_state'
            else:
                print("plan failed!")
                return 'waiting_state' 
        else:
            return 'waiting_state'

# define state PickRobotArm
class PickRobotArm(smach.State):
    def __init__(self,move_grp):
        smach.State.__init__(self, outcomes=['pick_robot_arm_state','gripper_pick_state'])
        rospy.Subscriber("arm_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, self.result_cb)
        self.result = -1
        self.move_grp = move_grp
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PickRobotArm')

        if userdata.target_pose:
            p = userdata.target_pose
            p_orient = p.orientation.w

            self.move_grp.execute_plan(self.plan)
            self.move_grp.go_to_pose_goal(p.position.x, p.position.y,0.20,p_orient-1.5708)

        if self.counter == 2:
            return 'pick_robot_arm_state'
        else:
            return 'gripper_pick_state'

    def result_cb(self,data):
        print("result: "+data.status.status)
        self.result = data.status.status
        self.counter += 1
        

def main():
    rospy.init_node('ur_smach_state_machine')

    # listener = tf.TransformListener()
    move_grp = MoveGroupPythonIntefaceTutorial()
    

    zero_goal = [0, -pi/2, 0, -pi/2, 0, 0]
    observe_goal = [-0.27640452940659355, -1.5613947841166143, 0.8086120509001136, -0.8173772811698496, -1.5702185440399328, -0.2754254250487067]


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.color = ""
    sm.userdata.target_pose = None
    sm.userdata.plan = None

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CheckRobotArm', CheckRobotArm(), 
                               transitions={'init_robot_arm_state':'InitRobotArm', 
                                            'check_robot_arm_state':'CheckRobotArm'})

        smach.StateMachine.add('InitRobotArm', InitRobotArm(move_grp), 
                               transitions={'observe_robot_arm_state':'ObserveRobotArm', 
                                            'init_robot_arm_state':'InitRobotArm'})

        smach.StateMachine.add('ObserveRobotArm', ObserveRobotArm(move_grp), 
                               transitions={'trigger_pick_place_task':'TriggerPickAndPlace', 
                                            'observe_robot_arm_state':'ObserveRobotArm'})                                
                                            
        smach.StateMachine.add('TriggerPickAndPlace', TriggerPickAndPlace(move_grp), 
                               transitions={'plan_pick_state':'PlanPick',
                               'trigger_pick_place_task':'TriggerPickAndPlace'})
                    
        smach.StateMachine.add('PlanPick', PlanPick(move_grp), 
                               transitions={'pick_robot_arm_state':'PickRobotArm',
                               'waiting_state':'PlanPick'})

        smach.StateMachine.add('PickRobotArm', PickRobotArm(move_grp), 
                               transitions={'gripper_pick_state':'outcome4',
                               'pick_robot_arm_state':'PickRobotArm'})


    sis = smach_ros.IntrospectionServer('robot_arm_task_manager', sm, '/robot_arm_task_manager')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    print("End : {}".format(outcome))
    sis.stop()

if __name__ == '__main__':
    main()