#! /usr/bin/env python

import roslib
roslib.load_manifest('yalim_robot_bringup')
import rospy
import actionlib
from yalim_robot_bringup.msg import RobotArmAction, RobotArmResult
from geometry_msgs.msg import PoseStamped
import moveit_commander

class MoveArmServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('move_arm', RobotArmAction, self.execute, False)
        self.server.start()
        self.pub = rospy.Publisher('move_arm_goal', PoseStamped, queue_size=10)
        self.result = RobotArmResult()
        self.group = moveit_commander.MoveGroupCommander("arm")

    def execute(self, goal):
        self.group.set_pose_target(goal.goal_pose)
        plan1 = self.group.plan()
        self.group.execute(plan1)

        self.pub.publish(goal.goal_pose)
        rospy.loginfo('Goal sent!')
        rospy.sleep(5)
        self.result.is_finished = True
        self.server.set_succeeded(self.result)


if __name__ == '__main__':
    rospy.init_node('robot_arm_server')
    server = MoveArmServer()
    rospy.spin()
