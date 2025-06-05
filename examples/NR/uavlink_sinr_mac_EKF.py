# Copyright (c) 2025 Southwest University of Science and Technology

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation;

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

# Authors: Jingxiang Yu <yujx@mails.swust.edu.cn> (alternate: yujx.res@gmail.com)


import numpy as np
import tensorflow as tf
import sys
import gc
import uavlink_py as py_binding
from pyuavlink import UAVLinkSimulation

import traceback
import os
from pathlib import Path

# delta for prediction
delta = 1

MAX_RBG_NUM = 10  # Number of SINR resource blocks
STATE_DIM = MAX_RBG_NUM + 6  # SINR(10) + px, py, pz, vx, vy, vz
MEASUREMENT_DIM = MAX_RBG_NUM + 6  # Observable variables

# Get the current file's directory
current_dir = Path(__file__).parent.absolute()
ns3_path = current_dir.parent.parent.parent.parent
os.chdir(ns3_path)

tf.random.set_seed(0)
np.random.seed(1)

# EKF parameters
dt = 1.0  # Time step

# === State transition matrix F ===
F = np.eye(STATE_DIM, dtype=np.float32)
F[MAX_RBG_NUM:MAX_RBG_NUM+3, MAX_RBG_NUM+3:MAX_RBG_NUM+6] = dt * np.eye(3)   # Position influenced by velocity

# === Observation matrix H ===
H = np.eye(MEASUREMENT_DIM, STATE_DIM, dtype=np.float32)  # Directly observe SINR, position, velocity

# === Process noise covariance matrix Q ===
Q_sinr = 0.01 * np.eye(MAX_RBG_NUM)  # SINR process noise
Q_pos = 0.1 * np.eye(3)  # Position process noise
Q_vel = 0.05 * np.eye(3)  # Velocity process noise
Q = np.block([
    [Q_sinr, np.zeros((MAX_RBG_NUM, 3)), np.zeros((MAX_RBG_NUM, 3))],
    [np.zeros((3, MAX_RBG_NUM)), Q_pos, np.zeros((3, 3))],
    [np.zeros((3, MAX_RBG_NUM)), np.zeros((3, 3)), Q_vel]
])

# === Measurement noise covariance matrix R ===
R_sinr = 0.1 * np.eye(MAX_RBG_NUM)  # SINR measurement error
R_pos = 0.05 * np.eye(3)  # Position measurement error
R_vel = 0.02 * np.eye(3)  # Velocity measurement error
R = np.block([
    [R_sinr, np.zeros((MAX_RBG_NUM, 3)), np.zeros((MAX_RBG_NUM, 3))],
    [np.zeros((3, MAX_RBG_NUM)), R_pos, np.zeros((3, 3))],
    [np.zeros((3, MAX_RBG_NUM)), np.zeros((3, 3)), R_vel]
])

# EKF variables
ekf_initialized = False
ekf_state = None
ekf_cov = None

# Start simulation
exp = UAVLinkSimulation("uavlink-nr-v2x-uav", ".", py_binding, handle_finish=True)
msgInterface = exp.start_simulation(show_output=True)

try:
    while True:
        msgInterface.PyRecvBegin()
        if msgInterface.PyGetFinished():
            break
        gc.collect()

 
        # Get sinrPerRbg array from shared memory
        sinrPerRbg_6dof = msgInterface.GetCpp2PyStruct().sinrPerRbg_6dof
        msgInterface.PyRecvEnd()

        sinrPerRbg_6dof= np.array(sinrPerRbg_6dof)


        sinr = sinrPerRbg_6dof[:MAX_RBG_NUM]  # Extract the first 10 SINR values
        px, py, pz = sinrPerRbg_6dof[MAX_RBG_NUM:MAX_RBG_NUM+3]  # Extract position
        vx, vy, vz = sinrPerRbg_6dof[MAX_RBG_NUM+3:MAX_RBG_NUM+6]  # Extract velocity

        # Observation vector Z
        z = np.hstack((sinr, [px, py, pz, vx, vy, vz]))

        # === EKF prediction ===
        if not ekf_initialized:
            # Initialize state
            ekf_state = np.zeros((STATE_DIM,), dtype=np.float32)
            ekf_state[:MAX_RBG_NUM] = sinr  # Initial SINR
            ekf_state[MAX_RBG_NUM:MAX_RBG_NUM+3] = [px, py, pz]  # Position
            ekf_state[MAX_RBG_NUM+3:] = [vx, vy, vz]  # Velocity

            # Initialize covariance matrix
            ekf_cov = np.eye(STATE_DIM, dtype=np.float32)
            ekf_initialized = True
        else:
            # Predict
            x_pred = F @ ekf_state
            P_pred = F @ ekf_cov @ F.T + Q

            # Compute Kalman gain
            y = z - H @ x_pred
            S = H @ P_pred @ H.T + R
            K = P_pred @ H.T @ np.linalg.inv(S)

            # Update state
            ekf_state = x_pred + K @ y
            ekf_cov = (np.eye(STATE_DIM) - K @ H) @ P_pred


        # === Predict future SINR ===
        predicted_sinr = ekf_state[:MAX_RBG_NUM]

        # Map predicted SINR to MCS
        def sinr_to_mcs(sinr_value):
            mcs_table = [
                (-6.02, 0.2344, 0), (-4.14, 0.3770, 1), (-2.05, 0.6016, 2),
                (-0.03, 0.8770, 3), (1.99, 1.1758, 4), (7.03, 1.4766, 5),
                (9.93, 1.6953, 6), (11.01, 1.9141, 7), (11.95, 2.1602, 8),
                (12.09, 2.4063, 9), (13.10, 2.5703, 10), (15.12, 2.7305, 11),
                (16.07, 3.0293, 12), (19.03, 3.3223, 13), (19.10, 3.6094, 14),
                (21.06, 3.9023, 15), (21.13, 4.2129, 16), (23.02, 4.5234, 17),
                (23.96, 4.8164, 18), (24.09, 5.1152, 19), (28.07, 5.3320, 20),
                (31.10, 5.5547, 21), (35.14, 5.8906, 22)
            ]


            mcs_index = 22
            # Find the largest MCS index whose SINR threshold is less than or equal to sinr_value
            for snr_threshold, _, index in (mcs_table):
                if sinr_value >= snr_threshold:
                    mcs_index = index

            return mcs_index

        predicted_mcs = np.array([sinr_to_mcs(s) for s in predicted_sinr])
        predicted_mcs = predicted_mcs/10.0

        # Send predicted MCS
        msgInterface.PySendBegin()
        msgInterface.GetPy2CppStruct().predictedMcs = predicted_mcs.tolist()
        msgInterface.PySendEnd()


except Exception as e:
    print("Error:", e)

finally:
    print("Exiting...")
    del exp