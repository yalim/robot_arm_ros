#!/usr/bin/env python
# import roslib
import rospy
import smach
import smach_ros
# from utils.utils import lstm_gesture_classification
# from utils.utils import noise_vs_gesture
from utils.topic_reader import topic_reader_state
# import socket
# import numpy as np
# from scipy.signal import butter, lfilter
# from utils.utils import fifo_array
# from utils.utils import bcolors
from std_msgs.msg import Int32
# from geometry_msgs.msg import PoseStamped
from yalim_robot_bringup.msg import RobotArmAction, RobotArmGoal
# roslib.load_manifest('gesture_teleoperation')


# class DetectGesture(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeed', 'cancelled', 'error'], output_keys=['gesture_id'])

#     def execute(self, userdata):
#         rospy.loginfo('Get the gesture')
#         # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
#         # sock.bind(("", 10552))
#         gest_list = ['ccw_circle',
#                  'cw circle',
#                  'jerk up',
#                  'jerk down',
#                  'jerk right',
#                  'jerk left',
#                  'ccw triangle',
#                  'cw triangle',
#                  'zorro']

#         x_accel = fifo_array(100)
#         y_accel = fifo_array(100)
#         z_accel = fifo_array(100)
#         x_gyro = fifo_array(100)
#         y_gyro = fifo_array(100)
#         z_gyro = fifo_array(100)
#         gesture_prediction_list = fifo_array(100)
#         high_b, high_a = butter(3, 0.01, 'highpass')

#         # data_raw = sock.recv(1024)
#         # data = [float(x.strip()) for x in data_raw.split(',')]
#         x_accel.add_element(userdata.linear_acceleration.x)
#         y_accel.add_element(userdata.linear_acceleration.y)
#         z_accel.add_element(userdata.linear_acceleration.z)
#         x_gyro.add_element(userdata.angular_velocity.x)
#         y_gyro.add_element(userdata.angular_velocity.y)
#         z_gyro.add_element(userdata.angular_velocity.z)

#         # Filter the gravity term and convert fifos to numpy arrays
#         high_b, high_a = butter(3, 0.01, 'highpass')
#         x_accel_np = lfilter(high_b, high_a, x_accel.get_value()).reshape((1, 100))
#         y_accel_np = lfilter(high_b, high_a, y_accel.get_value()).reshape((1, 100))
#         z_accel_np = lfilter(high_b, high_a, z_accel.get_value()).reshape((1, 100))
#         x_gyro_np = x_gyro.get_value()
#         y_gyro_np = y_gyro.get_value()
#         z_gyro_np = z_gyro.get_value()
#         x = np.hstack((x_accel_np, y_accel_np, z_accel_np, x_gyro_np, y_gyro_np, z_gyro_np))
#         # x_60 = np.hstack((x_accel_np[0, 80:], y_accel_np[0, 80:], z_accel_np[0, 80:],
#         #                x_gyro_np[0, 80:], y_gyro_np[0, 80:], z_gyro_np[0, 80:]))
#         x_60 = np.hstack((x_gyro_np[0, 80:], y_gyro_np[0, 80:], z_gyro_np[0, 80:]))
#         # energy = np.power(x, 2).sum()
#         energy_final_x = np.power(x_60, 2).sum()

#         # Check if the energy is dropping
#         if energy_final_prev > energy_final_x:
#             energy_dropping = True

#         energy_final_prev = energy_final_x

#         print('Energy final: ', energy_final_x)
#         # print('Energy: ', energy)
#         prediction = np.argmax(noise_vs_gesture.predict(x), axis=1)
#         print(prediction[0])

#         if prediction[0] > 0.5:
#             gesture_detected = True
#             print(bcolors.WARNING + 'Gesture!!' + bcolors.ENDC)
#             if gesture_detected:
#                 gesture_detected = False
#                 probabilities = lstm_gesture_classification.predict([x_accel_np.reshape((1, 100, 1)),
#                                                     y_accel_np.reshape((1, 100, 1)),
#                                                     z_accel_np.reshape((1, 100, 1)),
#                                                     x_gyro_np.reshape((1, 100, 1)),
#                                                     y_gyro_np.reshape((1, 100, 1)),
#                                                     z_gyro_np.reshape((1, 100, 1))])

#                 if np.amax(probabilities, axis=1) > 0.8 and energy_dropping:  # Check sureness of the gesture detection
#                     classification = np.argmax(probabilities, axis=1)
#                     print(probabilities)
#                     print(bcolors.WARNING + 'Probability: ' +
#                           str(np.amax(probabilities, axis=1)) +
#                           'Gesture: ' + str(gest_list[classification[0]]) + bcolors.ENDC)
#         userdata.gesture_id = classification[0]
#         return 'succeed'


# class MoveRobot(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeed', 'cancelled', 'error'], input_keys=['gesture_id'])

#     def execute(self, userdata):
#         if userdata.gesture_id == 1:
#             rospy.loginfo('Moving to position 1.')
#             print('pose1')

#         if userdata.gesture_id == 2:
#             rospy.loginfo('Moving to position 2.')

#         return 'succeed'


# class GoalCreator(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'cancelled'], input_keys=['gesture_id'], output_keys=['goal_position'])
#         self.ActionGoal = RobotArmGoal()

#     def execute(self, userdata):
#         # rospy.loginfo('Detected Gesture: ', userdata.gesture_id)
#         return 'succeeded'
#         self.ActionGoal.goal_pose.header.seq = 1
#         self.ActionGoal.goal_pose.header.stamp = rospy.Time()
#         self.ActionGoal.goal_pose.header.frame_id = 'base_link'
#         if userdata.gesture_id.data == 0:
#             self.ActionGoal.goal_pose.pose.position.x = 0.0
#             self.ActionGoal.goal_pose.pose.position.y = 0.0
#             self.ActionGoal.goal_pose.pose.position.z = 0.355
#             self.ActionGoal.goal_pose.pose.orientation.x = 0.0
#             self.ActionGoal.goal_pose.pose.orientation.y = 0.0
#             self.ActionGoal.goal_pose.pose.orientation.z = 0.0
#             self.ActionGoal.goal_pose.pose.orientation.w = 1.0
#             userdata.goal_position = self.ActionGoal
#             return 'succeeded'
#         if userdata.gesture_id.data == 1:
#             self.ActionGoal.goal_pose.pose.position.x = -0.18
#             self.ActionGoal.goal_pose.pose.position.y = 0.062
#             self.ActionGoal.goal_pose.pose.position.z = 0.025
#             self.ActionGoal.goal_pose.pose.orientation.x = 0.258
#             self.ActionGoal.goal_pose.pose.orientation.y = 0.965
#             self.ActionGoal.goal_pose.pose.orientation.z = 0.014
#             self.ActionGoal.goal_pose.pose.orientation.w = -0.051
#             userdata.goal_position = self.ActionGoal
#         if userdata.gesture_id.data == 10:
#             return 'cancelled'


def main():
    """Main function."""
    rospy.init_node('gesture_teleoperation_node')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeed', 'cancelled'])
    rospy.loginfo('Hodooor')

    # Open the container
    with sm:
        # Add states to the container
        # smach.StateMachine.add('TOPIC_READER', topic_reader_state(),
        #                        transitions={'succeeded': 'DETECT_GESTURE', 'aborted': 'TOPIC_READER', 'preempted': 'TOPIC_READER'})

        # smach.StateMachine.add('DETECT_GESTURE', DetectGesture(),
        #                        transitions={'succeed': 'MOVE_ROBOT',
        #                                     'cancelled': 'cancelled',
        #                                     'error': 'DETECT_GESTURE'})

        # smach.StateMachine.add('MOVE_ROBOT', MoveRobot(),
        #                        transitions={'succeed': 'DETECT_GESTURE',
        #                                     'cancelled': 'cancelled',
        #                                     'error': 'MOVE_ROBOT'})

        def goal_callback(userdata, default_goal):
            """Create goal based on detected gesture."""
            rospy.loginfo("Detected gesture ID: " + str(userdata.gesture_id.data))
            goal = RobotArmGoal()
            goal.goal_pose.header.seq = 1
            goal.goal_pose.header.stamp = rospy.Time()
            goal.goal_pose.header.frame_id = 'base_link'
            if userdata.gesture_id.data == 0:
                goal.goal_pose.pose.position.x = -0.108
                goal.goal_pose.pose.position.y = 0.062
                goal.goal_pose.pose.position.z = 0.025
                goal.goal_pose.pose.orientation.x = 0.258
                goal.goal_pose.pose.orientation.y = 0.965
                goal.goal_pose.pose.orientation.z = 0.014
                goal.goal_pose.pose.orientation.w = -0.051
            if userdata.gesture_id.data == 1:
                goal.goal_pose.pose.position.x = 0.0
                goal.goal_pose.pose.position.y = 0.0
                goal.goal_pose.pose.position.z = 0.355
                goal.goal_pose.pose.orientation.x = 0.0
                goal.goal_pose.pose.orientation.y = 0.0
                goal.goal_pose.pose.orientation.z = 0.0
                goal.goal_pose.pose.orientation.w = 1.0
            return goal

        smach.StateMachine.add('GESTURE_READER', topic_reader_state('gesture_teleoperation/detected_gesture', Int32, 60),
                               transitions={'succeeded': 'MOVE_ARM_ACTION', 'preempted': 'GESTURE_READER', 'aborted': 'GESTURE_READER'},
                               remapping={'topic_output_msg': 'gesture_id'})

        # smach.StateMachine.add('CREATE_GOAL', GoalCreator(),
        #                       transitions={'succeeded': 'MOVE_ARM_ACTION', 'cancelled': 'GESTURE_READER'})

        smach.StateMachine.add('MOVE_ARM_ACTION',
                               smach_ros.SimpleActionState('move_arm',
                                                           RobotArmAction,
                                                           goal_cb=goal_callback,
                                                           input_keys=['gesture_id']),
                               transitions={'succeeded': 'GESTURE_READER', 'preempted': 'GESTURE_READER', 'aborted': 'GESTURE_READER'})
    # Execute SMACH plan
    outcome = sm.execute()
    print(outcome)


if __name__ == '__main__':
    main()
