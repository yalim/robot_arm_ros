#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
# import socket
import numpy as np
from scipy.signal import butter, lfilter
from keras.models import load_model
from utils.utils import fifo_array
from keras.models import Sequential
from keras.layers import Merge
from keras.layers.core import Dense
from keras.layers.recurrent import LSTM
from sensor_msgs.msg import Imu
subsample = 1
N_dense = 99  # 60 for subsample 2
N_LSTM = 44  # 40 for subsample 2
model_x_acc = Sequential()
model_x_acc.add(LSTM(N_LSTM, input_shape=(100 / subsample, 1)))
model_y_acc = Sequential()
model_y_acc.add(LSTM(N_LSTM, input_shape=(100 / subsample, 1)))
model_z_acc = Sequential()
model_z_acc.add(LSTM(N_LSTM, input_shape=(100 / subsample, 1)))

model_x_gyr = Sequential()
model_x_gyr.add(LSTM(N_LSTM, input_shape=(100 / subsample, 1)))
model_y_gyr = Sequential()
model_y_gyr.add(LSTM(N_LSTM, input_shape=(100 / subsample, 1)))
model_z_gyr = Sequential()
model_z_gyr.add(LSTM(N_LSTM, input_shape=(100 / subsample, 1)))

merged = Merge([model_x_acc,
                model_y_acc,
                model_z_acc,
                model_x_gyr,
                model_y_gyr,
                model_z_gyr],
               mode='concat')

lstm_gesture_classification = Sequential()
lstm_gesture_classification.add(merged)
lstm_gesture_classification.add(Dense(N_dense, activation='relu'))
lstm_gesture_classification.add(Dense(9, activation='softmax'))

lstm_gesture_classification.load_weights('/home/yalim/robot_arm/src/gesture_teleoperation/src/gr_keras_weights.h5')

print('Model initialized, weights loaded.')

noise_vs_gesture = load_model('/home/yalim/robot_arm/src/gesture_teleoperation/src/noise_vs_gesture_keras_model.h5')
print('Gesture vs Noise model loaded')

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
# sock.bind(("", 10552))

x_accel = fifo_array(120)
y_accel = fifo_array(120)
z_accel = fifo_array(120)
x_gyro = fifo_array(120)
y_gyro = fifo_array(120)
z_gyro = fifo_array(120)
high_b, high_a = butter(3, 0.01, 'highpass')
gesture_vote = []


def most_common(lst):
    """Find the most common element in a list."""
    return max(set(lst), key=lst.count)


def subscriber_callback(msg):
    """Subscriber callback."""
    pub = rospy.Publisher('gesture_teleoperation/detected_gesture', Int32, queue_size=10)
    gesture_prediction_list = fifo_array(100)

    global x_accel
    global y_accel
    global z_accel
    global x_gyro
    global y_gyro
    global z_gyro
    global gesture_vote

    # print(msg.linear_acceleration.x)

    x_accel.add_element(msg.linear_acceleration.x)
    y_accel.add_element(msg.linear_acceleration.y)
    z_accel.add_element(msg.linear_acceleration.z)
    x_gyro.add_element(msg.angular_velocity.x)
    y_gyro.add_element(msg.angular_velocity.y)
    z_gyro.add_element(msg.angular_velocity.z)

    x_accel_np = lfilter(high_b, high_a, x_accel.get_value())[0, 0:100:subsample].reshape((1, 100 / subsample))
    y_accel_np = lfilter(high_b, high_a, y_accel.get_value())[0, 0:100:subsample].reshape((1, 100 / subsample))
    z_accel_np = lfilter(high_b, high_a, z_accel.get_value())[0, 0:100:subsample].reshape((1, 100 / subsample))
    x_gyro_np = x_gyro.get_value()[0, 0:100:subsample].reshape((1, 100 / subsample))
    y_gyro_np = y_gyro.get_value()[0, 0:100:subsample].reshape((1, 100 / subsample))
    z_gyro_np = z_gyro.get_value()[0, 0:100:subsample].reshape((1, 100 / subsample))

    # Predict noise vs gesture then gesture classification
    # print(x_gyro_np)

    x = np.hstack((x_accel_np, y_accel_np, z_accel_np, x_gyro_np, y_gyro_np, z_gyro_np))
    x_60 = np.hstack((x_gyro.get_value()[0, 100:], x_gyro.get_value()[0, 100:], x_gyro.get_value()[0, 100:]))
    # print(x_60)
    std_dev = np.std(x_60)
    # print(std_dev)

    prediction = np.argmax(noise_vs_gesture.predict(x), axis=1)
    probability_of_gesture = noise_vs_gesture.predict(x)
    # print('Noise vs gesture prob: ' + str(probability_of_gesture))

    if probability_of_gesture[0, 1] > 0.9:  # Put a threshold to gesture probability
        gesture_detected = True
        # print(bcolors.WARNING + 'Gesture!!' + bcolors.ENDC)

        probabilities = lstm_gesture_classification.predict([x_accel_np.reshape((1, 100 / subsample, 1)),
                                            y_accel_np.reshape((1, 100 / subsample, 1)),
                                            z_accel_np.reshape((1, 100 / subsample, 1)),
                                            x_gyro_np.reshape((1, 100 / subsample, 1)),
                                            y_gyro_np.reshape((1, 100 / subsample, 1)),
                                            z_gyro_np.reshape((1, 100 / subsample, 1))])

        if np.amax(probabilities, axis=1) > 0.8 and std_dev < 0.5:  # Check sureness of the gesture detection and if gesture is finished.
            classification = np.argmax(probabilities, axis=1)
            gesture_vote.append(classification[0])
            # print(bcolors.WARNING + 'Probability: ' +
            #       str(np.amax(probabilities, axis=1)) +
            #       'Gesture: ' + str(gest_list[classification[0]]) + bcolors.ENDC)
    elif gesture_vote and std_dev < 1.0:
        # print(gesture_vote)
        selected_gesture = most_common(gesture_vote)
        pub.publish(selected_gesture)
        gesture_vote = []
    else:
        pub.publish(10)


def main():
    """Main function."""
    rospy.init_node('real_time_gesture_recognizer')
    rospy.Subscriber('gesture_teleoperation/Imu', Imu, subscriber_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
