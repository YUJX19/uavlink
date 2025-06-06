# UAVLink

This repository contains the implementation of our paper:

**"ns3-uavlink: AI-Driven Dynamic MCS Scheduling for U2U Sidelink Communication"**
(Accepted at VTC2025-Fall).

---

## Quick Start

To simplify the setup process, we provide a ready-to-use Docker image.

### Run the Container

You can start the container directly. Docker will automatically pull the image if it’s not already available locally:

```bash
docker run -it rholand9/uavlink-ai:v1.1 /bin/bash
```

> Alternatively, you can create your own Dockerfile based on this image to add custom configurations.

---

Inside the container, clone the `uavlink` source code:

```bash
git clone https://github.com/YUJX19/uavlink.git
cd uavlink
```

---

## Build UAVLink Module for ns-3-v2x

Run the provided setup script to configure and build the UAVLink module:

```bash
chmod +x setup-ns3-v2x-uavlink.sh
./setup-ns3-v2x-uavlink.sh
```

This will:

* Configure the ns-3 environment
* Copy the necessary NR sidelink source files
* Compile all required modules including `contrib/uavlink`

---

## Run Demo Scripts

After compilation, you can generate experimental demo data using the following scripts:

### 1. Generate LSTM Training Data

```bash
cd /ns-3-dev
python3 contrib/uavlink/examples/NR/uavlink_macs.py
```

This script collects data for training AI-based MCS selection using LSTM.

---

### 2. Generate EKF Evaluation Data

```bash
cd /ns-3-dev
python3 contrib/uavlink/examples/NR/uavlink_sinr_mac_EKF.py
```

This script collects real-time SINR and MCS estimation data using an Extended Kalman Filter.

---

## Project Structure Overview

```
uavlink/
├── helper/                  # Helper classes for simulation setup
├── model/                   # Core UAVLink simulation logic and OpenGym integration
├── nr-sidelink/             # Customized NR Sidelink MAC/PHY modules
├── examples/NR/             # LSTM and EKF experiment scripts
├── test/                    # Unit test for ns-3 module
├── setup-ns3-v2x-uavlink.sh # One-step setup script
└── CMakeLists.txt           # Build configuration
```

---

## License

This project is licensed under the GNU General Public License v3.0.
See [LICENSE](LICENSE) for details.

---

## Acknowledgment

This project builds on top of [ns-3-dev](https://www.nsnam.org/) and NR module from CTTC.
