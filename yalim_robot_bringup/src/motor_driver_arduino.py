#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float64
from utils import topic_reader
roslib.load_manifest('yalim_robot_bringup')
import smach
import smach_ros
import moveit_commander
from geometry_msgs.msg import Pose

"""This state will recieve a goal pose (or joint angle) and move the robot (by publishing topics)"""

class MoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['pose']

    def execute(self, userdata):
        


def main():
    rospy.init_node('move_arm_arduino')
    sm = smach.StateMachine(outcomes=['succeeded', 'cancelled']

    with sm:
        smach.StateMachine.add('TOPIC_READER', topic_reader_state(), transitions={'succeeded':'MOVE_ARM_STATE', 'preempted':'TOPIC_READER', 'aborted':'TOPIC_READER'})

        smach.StateMachine.add('MOVE_ARM_STATE', MoveArm(), transitions={'succeeded':'TOPIC_READER', 'aborted':'cancelled'}

    outcome = sm.execute()


if __name__ == '__main__':
    main()

