# Copyright (c) 2019-2023 Huazhong University of Science and Technology, Dian Group
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation;
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Author: Pengyu Liu <eic_lpy@hust.edu.cn>
#         Xiaojun Guo <guoxj@hust.edu.cn>
#         Hao Yin <haoyin@uw.edu>
#         Muyuan Shen <muyuan_shen@hust.edu.cn>
# Modified by: Jingxiang Yu <yujx@mails.swust.edu.cn> (alternate: yujx.res@gmail.com)



import numpy as np
import tensorflow as tf
import keras
from keras.layers import *
import sys
import gc
import uavlink_py as py_binding
from pyuavlink import UAVLinkSimulation

import traceback
import os
from pathlib import Path

# delta for prediction
delta = 1

MAX_RBG_NUM = 10
MAX_RBG_NUM_6DOF = 16

# Get the current file's directory
current_dir = Path(__file__).parent.absolute()
# Get the ns-3 root directory
ns3_path = current_dir.parent.parent.parent.parent
# Change working directory to the ns-3 root
print("ns3_path: ", ns3_path)

os.chdir(ns3_path)

tf.random.set_seed(0)
np.random.seed(1)

input_len = 20
pred_len = 5
batch_size = 2
alpha = 0.6
not_train = False


lstm_input_vec = Input(shape=(input_len, MAX_RBG_NUM_6DOF), name="input_vec")
dense1 = TimeDistributed(Dense(30, activation='swish', kernel_regularizer='l1'))(lstm_input_vec)
lstm_mse = LSTM(20)(dense1)
predict_lstm_mse = Dense(MAX_RBG_NUM)(lstm_mse) 
lstm_model_mse = keras.Model(inputs=lstm_input_vec, outputs=predict_lstm_mse)
lstm_model_mse.compile(optimizer="adam", loss="MSE")

# Print the LSTM model structure
lstm_model_mse.summary()

def simple_MSE(y_pred, y_true):
    return (((y_pred - y_true)**2)).mean()


def weighted_MSE(y_pred, y_true):
    return (((y_pred - y_true)**2) * (1 + np.arange(len(y_pred))) /
            len(y_pred)).mean()


sinr_queue = np.empty((0, MAX_RBG_NUM_6DOF), dtype=np.float32)
train_data = np.empty((0, input_len, MAX_RBG_NUM_6DOF), dtype=np.float32)  # Assuming input_len is 10
delay_queue = np.empty((0, MAX_RBG_NUM_6DOF), dtype=np.float32)


mcs_queue = np.empty((0, MAX_RBG_NUM), dtype=np.float32)
prediction = np.empty((0, MAX_RBG_NUM), dtype=np.float32)
last = np.empty((0, MAX_RBG_NUM), dtype=np.float32)
right = np.array([], dtype=np.int32)  # Assuming right stores 0 and 1
corrected_predict = np.empty((0, MAX_RBG_NUM), dtype=np.float32)
target = np.empty((0, MAX_RBG_NUM), dtype=np.float32)
delay_queue_mcs = np.empty((0, MAX_RBG_NUM), dtype=np.float32)
is_train = True

exp = UAVLinkSimulation("uavlink-nr-v2x-uav", ".", py_binding, handle_finish=True)
msgInterface = exp.start_simulation(show_output=True)

try:
    while True:
        msgInterface.PyRecvBegin()
        if msgInterface.PyGetFinished():
            break
        gc.collect()

        # Get sinrPerRbg_6dof array from shared memory
        sinrPerRbg_6dof = msgInterface.GetCpp2PyStruct().sinrPerRbg_6dof
        selectedMcs = msgInterface.GetCpp2PyStruct().selectedMcs
        msgInterface.PyRecvEnd()

        # sinrPerRbg = np.array(sinrPerRbg) / 10.0
        sinrPerRbg_6dof= np.array(sinrPerRbg_6dof) / 10.0
        sinrPerRbg = sinrPerRbg_6dof[:-6]
        selectedMcs = np.array(selectedMcs) / 10.0

        # Print received SINR values
        print("\n My sinrPerRbg are ", sinrPerRbg)
        print("\n My sinrPerRbg_6dof are ", sinrPerRbg_6dof)
        
        # Append to delay queue
        delay_queue = np.append(delay_queue, sinrPerRbg_6dof[np.newaxis, ...], axis=0)
        delay_queue_mcs = np.append(delay_queue_mcs, selectedMcs[np.newaxis, ...], axis=0)
        

        if delay_queue.shape[0] < delta:
            current_sinr = delay_queue[-1]
            current_mcs = delay_queue_mcs[-1]
        else:
            current_sinr = delay_queue[-delta]
            current_mcs = delay_queue_mcs[-delta]
        if not_train:
            msgInterface.PySendBegin()
            msgInterface.GetPy2CppStruct().predictedMcs = current_mcs
            msgInterface.PySendEnd()
            continue

        sinr_queue = np.append(sinr_queue, current_sinr[np.newaxis, ...], axis=0)
        mcs_queue = np.append(mcs_queue, current_mcs[np.newaxis, ...], axis=0)

        if sinr_queue.shape[0] >= input_len + delta:
            target = np.append(target, current_mcs[np.newaxis, ...], axis=0)
        if sinr_queue.shape[0] >= input_len:
            one_data = np.array(sinr_queue[-input_len:])
            train_data = np.append(train_data, one_data[np.newaxis, ...], axis=0)
        else:
            msgInterface.PySendBegin()
            msgInterface.GetPy2CppStruct().predictedMcs = selectedMcs
            msgInterface.PySendEnd()
            continue


       # Execute prediction
        data_to_pred = one_data.reshape(-1, input_len, MAX_RBG_NUM_6DOF) # sinr_6dof
        _predict_mcs = lstm_model_mse.predict(data_to_pred)
        del data_to_pred


        prediction = np.append(prediction, _predict_mcs, axis=0)

        last = np.append(last, current_mcs[np.newaxis, ...], axis=0)

        corrected_predict = np.append(corrected_predict, _predict_mcs, axis=0)

        del one_data

        # Training and error calculation
        if train_data.shape[0] >= pred_len + delta:

            last_subarray = np.array(last[(-pred_len - delta):-delta])
            target_subarray = np.array(target[-pred_len:])
            prediction_subarray = np.array(prediction[(-pred_len - delta):-delta])

            # cal err_t
            err_t = weighted_MSE(last_subarray.flatten(), target_subarray.flatten())
            # cal err_p
            err_p = weighted_MSE(prediction_subarray.flatten(), target_subarray.flatten())

            # Print final error values
            print("\nWeighted MSE (err_t):", err_t)
            print("Weighted MSE (err_p):", err_p)

            if err_p <= err_t * alpha:
                if err_t < 1e-6:
                    corrected_predict[-1] = last[-1]
                print(" ")
                print("OK %d %f %f" % ((sinr_queue.shape[0]), err_t, err_p))
                right = np.append(right, [1], axis=0)
            else:
                corrected_predict[-1] = last[-1]
                if err_t <= 1e-6:
                    msgInterface.PySendBegin()
                    msgInterface.GetPy2CppStruct().predictedMcs = current_mcs
                    msgInterface.PySendEnd()
                    print("set wbsinr: ", current_sinr)
                    continue
                else:
                    print("train %d" % (sinr_queue.shape[0]))
                    right = np.append(right, [0], axis=0)
                    
                    # sinr_6dof
                    lstm_model_mse.fit(x=np.array(
                        train_data[-delta - batch_size:-delta]).reshape(
                            batch_size, input_len, MAX_RBG_NUM_6DOF),
                                        y=np.array(target[-batch_size:]),
                                        batch_size=batch_size,
                                        epochs=1,
                                        verbose=0)
        else:
            corrected_predict[-1] = last[-1]

        # Write corrected predictedMcs back to shared memory
        msgInterface.PySendBegin()
        msgInterface.GetPy2CppStruct().predictedMcs = corrected_predict[-1]
        msgInterface.PySendEnd()

except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print("Exception occurred: {}".format(e))
    print("Traceback:")
    traceback.print_tb(exc_traceback)
    exit(1)

else:
    with open("log_" + str(delta), "a+") as f:
        f.write("\n")
        if len(right):
            f.write("rate = %f %%\n" % (sum(right) / len(right)))
        f.write("MSE_T = %f %%\n" %
                (simple_MSE(np.array(target[delta:]), np.array(target[:-delta]))))
        f.write("MSE_p = %f %%\n" % (simple_MSE(
            np.array(corrected_predict[delta:]), np.array(target[:delta]))))

finally:
    print("Finally exiting...")
    del exp