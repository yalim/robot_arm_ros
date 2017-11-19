from keras.models import load_model
from keras.models import Sequential
from keras.layers import Merge
from keras.layers.core import Dense
from keras.layers.recurrent import LSTM
import numpy as np

#N_dense = 99
#N_LSTM = 44
#model_x_acc = Sequential()
#model_x_acc.add(LSTM(N_LSTM, input_shape=(100, 1)))
#model_y_acc = Sequential()
#model_y_acc.add(LSTM(N_LSTM, input_shape=(100, 1)))
#model_z_acc = Sequential()
#model_z_acc.add(LSTM(N_LSTM, input_shape=(100, 1)))

#model_x_gyr = Sequential()
#model_x_gyr.add(LSTM(N_LSTM, input_shape=(100, 1)))
#model_y_gyr = Sequential()
#model_y_gyr.add(LSTM(N_LSTM, input_shape=(100, 1)))
#model_z_gyr = Sequential()
#model_z_gyr.add(LSTM(N_LSTM, input_shape=(100, 1)))

#merged = Merge([model_x_acc,
 #               model_y_acc,
  #              model_z_acc,
   #             model_x_gyr,
    #            model_y_gyr,
     #           model_z_gyr],
      #         mode='concat')

#lstm_gesture_classification = Sequential()
#lstm_gesture_classification.add(merged)
#lstm_gesture_classification.add(Dense(N_dense, activation='relu'))
#lstm_gesture_classification.add(Dense(9, activation='softmax'))

#lstm_gesture_classification.load_weights('gr_keras_weights.h5')
#noise_vs_gesture = load_model('noise_vs_gesture_keras_model.h5')

class fifo_array():
    """
        A numpy ndarray that has a fixed length and works as FIFO
    """
    def __init__(self, max_length):
        self.max_len = max_length
        self.arr = np.zeros((1, self.max_len))

    def add_element(self, element):
        """
            Adds one element to the end
        """
        self.arr2 = np.append(self.arr, element)
        self.arr2 = np.delete(self.arr2, 0, 0)
        self.arr = np.reshape(self.arr2, self.arr.shape)

    def get_value(self):
        return self.arr

    def change_length(self, new_length):
        """
            This function adds zero to the end when increasing the length and
        removes from beginning when decreasing length
        """
        if self.max_len <= new_length:
            self.arr3 = np.zeros((1, new_length))
            self.arr3[0, 0:self.max_len] = self.arr
            self.arr = self.arr3
            self.max_len = new_length
        else:
            x = [y for y in range(self.max_len - new_length)]
            print(x)
            self.arr4 = np.delete(self.arr, x, 1)
            self.arr = self.arr4
            self.max_len = new_length

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
