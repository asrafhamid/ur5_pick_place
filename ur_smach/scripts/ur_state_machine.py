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
        print(data.status.status)
        self.result = data.status.status


# define state TriggerPickAndPlace
class TriggerPickAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_task','waiting_state'],output_keys=['color'])
        self.obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)
        # self.trigger = False
        self.pose=[]
        self.color = ""
        rospy.Subscriber("object_colour", String, self.trigger_pick_and_place)

    def execute(self, userdata):
        rospy.loginfo('Executing state TriggerPickAndPlace')

        if self.pose:
            userdata.color = self.color
            print(self.color)
            print(self.pose.poses.poses)
            return 'end_task'
        else:
            return 'waiting_state'
    
    def trigger_pick_and_place(self,data):
        # self.trigger = True
        self.color = data.data
        self.pose = self.obj_srv(self.color)

def main():
    rospy.init_node('ur_smach_state_machine')

    # listener = tf.TransformListener()
    move_grp = MoveGroupPythonIntefaceTutorial()
    

    zero_goal = [0, -pi/2, 0, -pi/2, 0, 0]
    observe_goal = [-0.27640452940659355, -1.5613947841166143, 0.8086120509001136, -0.8173772811698496, -1.5702185440399328, -0.2754254250487067]


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.color = ""
    # sm.userdata.mv_grp = MoveGroupPythonIntefaceTutorial()

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
                                            
        smach.StateMachine.add('TriggerPickAndPlace', TriggerPickAndPlace(), 
                               transitions={'end_task':'outcome4',
                               'waiting_state':'TriggerPickAndPlace'})


    sis = smach_ros.IntrospectionServer('robot_arm_task_manager', sm, '/robot_arm_task_manager')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    print("End : {}".format(outcome))
    sis.stop()

if __name__ == '__main__':
    main()