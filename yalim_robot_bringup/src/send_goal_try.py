#! /usr/bin/env python

import roslib
roslib.load_manifest('yalim_robot_bringup')
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
from yalim_robot_bringup.msg import RobotArmAction, RobotArmGoal


if __name__ == '__main__':
    rospy.init_node('arm_action_client')
    client = actionlib.SimpleActionClient('move_arm', RobotArmAction)
    client.wait_for_server()

    goal = RobotArmGoal()
    goal.goal_pose.header.seq = 1
    goal.goal_pose.header.stamp = rospy.Time.now()
    goal.goal_pose.header.frame_id = '/base_link'
    goal.goal_pose.pose.position.x = -0.108
    goal.goal_pose.pose.position.y = 0.062
    goal.goal_pose.pose.position.z = 0.025
    goal.goal_pose.pose.orientation.x = 0.258
    goal.goal_pose.pose.orientation.y = 0.965
    goal.goal_pose.pose.orientation.z = 0.014
    goal.goal_pose.pose.orientation.w = -0.051

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
