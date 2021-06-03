#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String
import rosservice
from obj_detection.srv import GetObject
import tf
from ur5_pick_place.ur5_pick_place import MoveGroupPythonIntefaceTutorial
 
# define state CheckRobotArm
class CheckRobotArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trigger_pick_place_task','check_robot_arm_state'])


        # here
        # start arm moveit
    def execute(self, userdata):
        rospy.loginfo('Executing state CheckRobotArm')

        # move_grp.de
        # rospy.wait_for_service('/get_obj_clr')
        if '/arm_controller/query_state' in rosservice.get_service_list():
            return 'trigger_pick_place_task'
        else:
            return 'check_robot_arm_state'


# define state TriggerPickAndPlace
class TriggerPickAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_task','waiting_state'])
        self.obj_srv = rospy.ServiceProxy('/get_obj_clr', GetObject)
        # self.trigger = False
        self.pose=[]
        self.color = ""
        rospy.Subscriber("object_colour", String, self.trigger_pick_and_place)

    def execute(self, userdata):
        rospy.loginfo('Executing state TriggerPickAndPlace')

        if self.pose:
            print(pose.poses.poses)
            return 'end_task'
        else:
            return 'waiting_state'
    
    def trigger_pick_and_place(self,data):
        # self.trigger = True
        self.color = data.data
        self.pose = self.obj_srv(self.color)

def main():
    rospy.init_node('ur_smach_state_machine')

    zero_goal = [0, -pi/2, 0, -pi/2, 0, 0]
    observe_goal = [-0.27640452940659355, -1.5613947841166143, 0.8086120509001136, -0.8173772811698496, -1.5702185440399328, -0.2754254250487067]

    move_grp = MoveGroupPythonIntefaceTutorial()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CheckRobotArm', CheckRobotArm(), 
                               transitions={'trigger_pick_place_task':'TriggerPickAndPlace', 
                                            'check_robot_arm_state':'CheckRobotArm'})
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