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

# EKF parameters.
# The update loop fires once per PSSCH receive, which is the sidelink
# reservationPeriod (RRI) = 100 ms in the active scenario. dt must match that
# cadence for the path-loss state transition to be physically consistent with
# how often we actually step the filter.
dt = 0.1


# === Nonlinear state transition: position advances by velocity (linear);
#     SINR advances by the dB change in path-loss as relative distance changes
#     (nonlinear in position/velocity). This is the nonlinearity that makes
#     this filter a genuine EKF rather than a linear KF: the SINR row of the
#     Jacobian depends on the current state and is recomputed each step. ===
def f_nonlinear(state, step):
    x = np.copy(state).astype(np.float64)
    pos = x[MAX_RBG_NUM:MAX_RBG_NUM + 3]
    vel = x[MAX_RBG_NUM + 3:MAX_RBG_NUM + 6]
    new_pos = pos + vel * step
    d_old = max(np.linalg.norm(pos), 1e-3)
    d_new = max(np.linalg.norm(new_pos), 1e-3)
    # Free-space path-loss change in dB between slots, applied uniformly per RB.
    delta_db = 20.0 * np.log10(d_old / d_new)
    x[:MAX_RBG_NUM] = x[:MAX_RBG_NUM] + delta_db
    x[MAX_RBG_NUM:MAX_RBG_NUM + 3] = new_pos
    return x.astype(np.float32)


def F_jacobian(state, step):
    """Analytical Jacobian of f_nonlinear evaluated at `state`."""
    J = np.eye(STATE_DIM, dtype=np.float32)
    pos = state[MAX_RBG_NUM:MAX_RBG_NUM + 3]
    vel = state[MAX_RBG_NUM + 3:MAX_RBG_NUM + 6]
    new_pos = pos + vel * step
    d_old = max(np.linalg.norm(pos), 1e-3)
    d_new = max(np.linalg.norm(new_pos), 1e-3)
    # Position rows: new_pos = pos + step * vel → ∂new_pos/∂vel = step*I.
    J[MAX_RBG_NUM:MAX_RBG_NUM + 3, MAX_RBG_NUM + 3:MAX_RBG_NUM + 6] = step * np.eye(3, dtype=np.float32)
    # SINR rows: new_sinr = old_sinr + 20*log10(|pos|/|new_pos|).
    # ∂Δ_dB/∂pos_i  = (20/ln10) * ( pos_i / d_old^2  -  new_pos_i / d_new^2 )
    # ∂Δ_dB/∂vel_i  = -(20/ln10) * new_pos_i / d_new^2 * step
    ln10 = np.log(10.0)
    for i in range(3):
        d_delta_d_pos = (20.0 / ln10) * (pos[i] / (d_old * d_old) - new_pos[i] / (d_new * d_new))
        d_delta_d_vel = -(20.0 / ln10) * new_pos[i] / (d_new * d_new) * step
        J[:MAX_RBG_NUM, MAX_RBG_NUM + i] = d_delta_d_pos
        J[:MAX_RBG_NUM, MAX_RBG_NUM + 3 + i] = d_delta_d_vel
    return J


# === Observation matrix H (linear: measurements directly expose SINR, pos, vel) ===
H = np.eye(MEASUREMENT_DIM, STATE_DIM, dtype=np.float32)

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

# Scenario settings forwarded to the ns-3 binary. Supported values for
# rxSpeedKmh are {135, 130, 120}; override via the env var UAVLINK_RX_SPEED_KMH
# so the same Python script can be re-run for each speed.
_settings = {
    "rxSpeedKmh": int(os.environ.get("UAVLINK_RX_SPEED_KMH", "135")),
    "useFixedMcs": "0",
}
if os.environ.get("UAVLINK_USE_FIXED_MCS", "0") == "1":
    _settings["useFixedMcs"] = "1"
print("uavlink_sinr_mac_EKF: scenario settings =", _settings)

# Start simulation
exp = UAVLinkSimulation("uavlink-nr-v2x-uav", ".", py_binding, handle_finish=True)
msgInterface = exp.start_simulation(settings=_settings, show_output=True)

try:
    while True:
        msgInterface.PyRecvBegin()
        if msgInterface.PyGetFinished():
            break
        gc.collect()

 
        # Get sinrPerRbg array from shared memory
        sinrPerRbg_6dof = msgInterface.GetCpp2PyStruct().sinrPerRbg_6dof
        # interferencePerRb is zero in the 2-UAV scenario; non-zero in multi-UAV.
        interferencePerRb = np.array(msgInterface.GetCpp2PyStruct().interferencePerRb)
        # Monotonic sequence counter; check it advances to detect stale reads.
        cpp_seq = msgInterface.GetCpp2PyStruct().seq
        msgInterface.PyRecvEnd()
        try:
            if cpp_seq <= _prev_seq:
                print("WARNING: shared-memory seq did not advance ({} -> {})"
                      .format(_prev_seq, cpp_seq))
        except NameError:
            pass
        _prev_seq = cpp_seq

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
            # EKF predict: nonlinear f with Jacobian linearisation about the current estimate.
            x_pred = f_nonlinear(ekf_state, dt)
            F_jac = F_jacobian(ekf_state, dt)
            P_pred = F_jac @ ekf_cov @ F_jac.T + Q

            # EKF update (observation is linear).
            y = z - H @ x_pred
            S = H @ P_pred @ H.T + R
            K = P_pred @ H.T @ np.linalg.inv(S)

            ekf_state = x_pred + K @ y
            ekf_cov = (np.eye(STATE_DIM) - K @ H) @ P_pred


        # === Predict future SINR for the next slot ===
        # Propagate the updated state one more step through the nonlinear f
        # so that the reported prediction corresponds to slot t+1.
        x_next = f_nonlinear(ekf_state, dt)
        predicted_sinr = x_next[:MAX_RBG_NUM]

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


            # Initialise to 0 (most robust MCS): if the predicted SINR is below
            # every threshold, fall back to the most reliable MCS, not the most
            # aggressive one. Add a 2 dB safety margin so the realised BLER stays
            # at or below 0.1 despite channel variability between prediction
            # and transmission (table thresholds sit exactly on BLER = 0.1).
            MCS_SNR_MARGIN_DB = 2.0
            mcs_index = 0
            for snr_threshold, _, index in (mcs_table):
                if sinr_value >= snr_threshold + MCS_SNR_MARGIN_DB:
                    mcs_index = index
                else:
                    break

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