# CCSLAM

A ROS 2 (Jazzy) workspace for cooperative/collaborative SLAM with PX4-based UAVs.

This workspace includes:

- **PX4 / ROS 2 interface**
  - `px4_msgs`
  - `px4_ros2_cpp`
  - `px4_ros_com`
  - `offboard_control`, `px4_keyboard_offboard`, `ROS2_PX4_Offboard_Example`
- **OpenVINS-based visual-inertial modules**
  - `ov_core`
  - `ov_init`
  - `ov_msckf`
  - `ov_eval`
- **CSLAM-UAV and mapping**
  - `CSLAM-UAV`
  - `gps_denied_rtab_mapping`
  - `drone_slam_pkg`

## Requirements

- Ubuntu 24.04 (or compatible)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- PX4 toolchain (firmware, SITL, etc. as needed)
- System dependencies:
  - `libeigen3-dev`
  - `ros-jazzy-eigen3-cmake-module`
  - any other ROS 2 dependencies required by the packages

## Setup

Clone the repo:

```bash
cd ~/Desktop# CCSLAM

A ROS 2 (Jazzy) workspace for cooperative/collaborative SLAM with PX4-based UAVs.

This repository contains multiple ROS 2 packages (PX4 interface, OpenVINS, CSLAM-UAV, mapping, etc.) and is intended to be used as a **colcon workspace** or inside a workspace.

---

## Contents

Main packages included (non‑exhaustive):

- **PX4 / ROS 2 interface**
  - `px4_msgs`
  - `px4_ros2_cpp`
  - `px4_ros_com`
  - `offboard_control`
  - `px4_keyboard_offboard`
  - `ROS2_PX4_Offboard_Example`
- **OpenVINS-based visual-inertial modules**
  - `ov_core`
  - `ov_init`
  - `ov_msckf`
  - `ov_eval`
- **CSLAM-UAV and mapping**
  - `CSLAM-UAV`
  - `gps_denied_rtab_mapping`
  - `drone_slam_pkg`

---

## Requirements

- Ubuntu 24.04 (or compatible)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) installed and sourced
- PX4 toolchain (firmware, SITL, etc.) as needed
- System dependencies, for example:
  - `libeigen3-dev`
  - `ros-jazzy-eigen3-cmake-module`
  - plus any ROS 2 dependencies required by the individual packages

Eigen CMake helper (if needed):

```bash
sudo apt install libeigen3-dev ros-jazzy-eigen3-cmake-module
git clone git@github.com:joshibasu62/CCSLAM.git
cd CCSLAM

---
If building error occurs Order should be
'''bash
px4_msgs
px4_ros2_cpp
ov_core
ov_init
and at last all
'''bash

