#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import socket


def publisher():
    pub = rospy.Publisher('gesture_teleoperation/Imu', Imu, queue_size=10)
    rospy.init_node('imu_publisher')
    rate = rospy.Rate(50)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    sock.bind(("", 10552))
    msg = Imu()
    while not rospy.is_shutdown():
        data_raw = sock.recv(1024)
        data = [float(x.strip()) for x in data_raw.split(',')]
        msg.header.seq = data[0]
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'sensor'

        msg.linear_acceleration.x = data[1]
        msg.linear_acceleration.y = data[2]
        msg.linear_acceleration.z = data[3]

        msg.angular_velocity.x = data[4]
        msg.angular_velocity.y = data[5]
        msg.angular_velocity.z = data[6]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    publisher()
