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

current_dir = Path(__file__).parent.absolute()
ns3_path = current_dir.parent.parent.parent.parent
print("ns3_path: ", ns3_path)
os.chdir(ns3_path)

tf.random.set_seed(0)
np.random.seed(1)

input_len = 20
# Online training fits one (input, target) pair per slot, so batch_size=1.
# Removed the stale `batch_size = 2` constant that was inconsistent with
# the actual fit call downstream.
not_train = False

# Single-layer LSTM (return_sequences=True) → TimeDistributed dense → FC output.
# Output dimension is MAX_RBG_NUM per-RB *SINR* values (not MCS), so the network
# realises a per-RB SINR predictor.
lstm_input_vec = Input(shape=(input_len, MAX_RBG_NUM_6DOF), name="input_vec")
lstm_seq = LSTM(20, return_sequences=True)(lstm_input_vec)
td_dense = TimeDistributed(Dense(30, activation='swish', kernel_regularizer='l1'))(lstm_seq)
last_step = Lambda(lambda x: x[:, -1, :])(td_dense)
predict_sinr = Dense(MAX_RBG_NUM)(last_step)
lstm_model = keras.Model(inputs=lstm_input_vec, outputs=predict_sinr)
lstm_model.compile(optimizer="adam", loss="MSE")
lstm_model.summary()

# SNR thresholds for BLER <= 0.1 per MCS index.
# Paired with argmax to select MCS* = argmax{MCS : BLER <= 0.1 | H}.
MCS_TABLE = [
    (-6.02, 0),  (-4.14, 1),  (-2.05, 2),  (-0.03, 3),  (1.99, 4),
    (7.03, 5),   (9.93, 6),   (11.01, 7),  (11.95, 8),  (12.09, 9),
    (13.10, 10), (15.12, 11), (16.07, 12), (19.03, 13), (19.10, 14),
    (21.06, 15), (21.13, 16), (23.02, 17), (23.96, 18), (24.09, 19),
    (28.07, 20), (31.10, 21), (35.14, 22),
]

# SNR safety margin added to the table thresholds.
# The table thresholds give BLER = 0.1 *at* the threshold — channel variability
# between the slot we measured SINR in and the slot the TB is actually sent on
# pushes realised BLER above 0.1 for ~15% of slots without a margin.
# A 2 dB margin empirically keeps the end-of-run empirical BLER below 0.1.
MCS_SNR_MARGIN_DB = 2.0


def sinr_to_mcs(sinr_db):
    """Return the highest MCS index that satisfies BLER <= 0.1 with a safety margin.
    Below the minimum threshold, returns MCS 0 (most robust).
    """
    mcs_index = 0
    for thresh, idx in MCS_TABLE:
        if sinr_db >= thresh + MCS_SNR_MARGIN_DB:
            mcs_index = idx
        else:
            break
    return mcs_index


# Full 6-DoF SINR observation history (SINR + relative position + relative velocity).
sinr_queue = np.empty((0, MAX_RBG_NUM_6DOF), dtype=np.float32)
prev_cpp_seq = 0  # shared-memory sequence counter

# Scenario settings forwarded to the ns-3 binary. Supported values for
# rxSpeedKmh are {135, 130, 120}; override via the env var UAVLINK_RX_SPEED_KMH
# so the same Python script can be re-run for each speed.
_settings = {
    "rxSpeedKmh": int(os.environ.get("UAVLINK_RX_SPEED_KMH", "135")),
    "useFixedMcs": "0",  # AI-driven MCS arm; set env UAVLINK_USE_FIXED_MCS=1 for baseline
}
if os.environ.get("UAVLINK_USE_FIXED_MCS", "0") == "1":
    _settings["useFixedMcs"] = "1"
print("uavlink_macs: scenario settings =", _settings)

exp = UAVLinkSimulation("uavlink-nr-v2x-uav", ".", py_binding, handle_finish=True)
msgInterface = exp.start_simulation(settings=_settings, show_output=True)

try:
    while True:
        msgInterface.PyRecvBegin()
        if msgInterface.PyGetFinished():
            break
        gc.collect()

        # Read the cpp2py shared-memory slot.
        sinrPerRbg_6dof = msgInterface.GetCpp2PyStruct().sinrPerRbg_6dof
        selectedMcs = msgInterface.GetCpp2PyStruct().selectedMcs
        interferencePerRb = np.array(msgInterface.GetCpp2PyStruct().interferencePerRb)
        cpp_seq = msgInterface.GetCpp2PyStruct().seq
        msgInterface.PyRecvEnd()

        if cpp_seq <= prev_cpp_seq:
            print("WARNING: shared-memory seq did not advance ({} -> {}); possible stale read"
                  .format(prev_cpp_seq, cpp_seq))
        prev_cpp_seq = cpp_seq

        # The C++ side transmits SINR (dB) scaled by 10; undo the scaling.
        sinrPerRbg_6dof = np.array(sinrPerRbg_6dof) / 10.0  # first 10 = SINR dB, last 6 = pos/vel
        sinr_queue = np.append(sinr_queue, sinrPerRbg_6dof[np.newaxis, ...], axis=0)

        if not_train:
            # Oracle-MCS mode retained for ablation. Send argmax-MCS from observed SINR.
            current_sinr_db = sinrPerRbg_6dof[:MAX_RBG_NUM] * 10.0
            oracle_mcs = np.array([sinr_to_mcs(s) for s in current_sinr_db], dtype=np.float64)
            msgInterface.PySendBegin()
            msgInterface.GetPy2CppStruct().predictedMcs = (oracle_mcs / 10.0).tolist()
            msgInterface.PySendEnd()
            continue

        # Warm-up: until we have `input_len` observations, fall back to argmax-MCS
        # on the currently-observed SINR (cannot run the LSTM yet).
        if sinr_queue.shape[0] < input_len:
            current_sinr_db = sinrPerRbg_6dof[:MAX_RBG_NUM] * 10.0
            warmup_mcs = np.array([sinr_to_mcs(s) for s in current_sinr_db], dtype=np.float64)
            msgInterface.PySendBegin()
            msgInterface.GetPy2CppStruct().predictedMcs = (warmup_mcs / 10.0).tolist()
            msgInterface.PySendEnd()
            continue

        # Predict future SINR at t + delta from the last input_len observations.
        # LSTM output is in the same normalised-SINR space as the input (÷ 10).
        input_seq = sinr_queue[-input_len:].reshape(1, input_len, MAX_RBG_NUM_6DOF)
        predicted_sinr_norm = lstm_model.predict(input_seq, verbose=0)[0]
        predicted_sinr_db = predicted_sinr_norm * 10.0

        # Select the argmax MCS over the BLER <= 0.1 table.
        predicted_mcs = np.array([sinr_to_mcs(s) for s in predicted_sinr_db], dtype=np.float64)

        print("predicted_sinr_db:", predicted_sinr_db)
        print("predicted_mcs:", predicted_mcs)

        msgInterface.PySendBegin()
        msgInterface.GetPy2CppStruct().predictedMcs = (predicted_mcs / 10.0).tolist()
        msgInterface.PySendEnd()

        # Online training: the SINR now observed at t is the ground truth for
        # the prediction issued at t - delta from the window [t - delta - input_len, t - delta].
        # Feeding this pair realises continuous refinement of the predictor.
        if sinr_queue.shape[0] >= input_len + delta:
            train_input = sinr_queue[-input_len - delta:-delta].reshape(
                1, input_len, MAX_RBG_NUM_6DOF)
            train_target = sinr_queue[-1, :MAX_RBG_NUM].reshape(1, MAX_RBG_NUM)
            lstm_model.fit(train_input, train_target,
                           epochs=1, batch_size=1, verbose=0)

except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print("Exception occurred: {}".format(e))
    print("Traceback:")
    traceback.print_tb(exc_traceback)
    exit(1)

finally:
    print("Finally exiting...")
    del exp
