#!/usr/bin/env python3
"""Fixed-MCS baseline runner.

Drives uavlink-nr-v2x-uav with useFixedMcs=1, serving only as a shared-memory
heartbeat (the NrSlUeMacSchedulerFixedMcs uses its constructor-time Mcs
attribute, so the MCS echoed back here is a no-op for scheduling). We still
need to consume each Cpp->Py message and reply, otherwise the simulator
blocks on the semaphore.

Env vars:
  UAVLINK_RX_SPEED_KMH  - UAV_B speed in km/h (default 135)
  UAVLINK_FIXED_MCS     - MCS index to pass as --mcs=... (default 2)
"""
import gc
import os
import sys
import traceback
from pathlib import Path

import numpy as np

import uavlink_py as py_binding
from pyuavlink import UAVLinkSimulation

MAX_RBG_NUM = 10

current_dir = Path(__file__).parent.absolute()
ns3_path = current_dir.parent.parent.parent.parent
os.chdir(ns3_path)

_settings = {
    "rxSpeedKmh": int(os.environ.get("UAVLINK_RX_SPEED_KMH", "135")),
    "useFixedMcs": "1",
    "mcs": int(os.environ.get("UAVLINK_FIXED_MCS", "2")),
}
print("uavlink_fixed_mcs: scenario settings =", _settings)

exp = UAVLinkSimulation("uavlink-nr-v2x-uav", ".", py_binding, handle_finish=True)
msgInterface = exp.start_simulation(settings=_settings, show_output=True)

fixed_mcs = float(_settings["mcs"])
reply = (np.ones(MAX_RBG_NUM, dtype=np.float64) * fixed_mcs / 10.0).tolist()

try:
    while True:
        msgInterface.PyRecvBegin()
        if msgInterface.PyGetFinished():
            break
        gc.collect()
        msgInterface.PyRecvEnd()

        msgInterface.PySendBegin()
        msgInterface.GetPy2CppStruct().predictedMcs = reply
        msgInterface.PySendEnd()

except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    print("Exception occurred: {}".format(e))
    traceback.print_tb(exc_traceback)
    sys.exit(1)

finally:
    print("Finally exiting...")
    del exp
