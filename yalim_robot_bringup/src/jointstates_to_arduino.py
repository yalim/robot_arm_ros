#! /usr/bin/env python

import roslib
roslib.load_manifest('yalim_robot_bringup')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


def publisher(recieved):
    pub = rospy.Publisher('joint_goals', Float64MultiArray, queue_size=10)
    rospy.loginfo('Publishing the goal')
    rospy.loginfo(recieved)
    published_msg = Float64MultiArray()
    multi_dim = MultiArrayDimension()
    multi_dim.label = 'foo'
    multi_dim.size = 1
    multi_dim.stride = 1
    published_msg.layout.dim = [multi_dim]
    published_msg.layout.data_offset = 1
    published_msg.data = recieved.position
    pub.publish(published_msg)

def listener():
    rospy.Subscriber("joint_states", JointState, publisher)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joint_goal_publisher', anonymous=True)
    listener()
