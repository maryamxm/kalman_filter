# Kalman Filter — ROS2 Port (B31YS Assignment)

A ROS2 recreation of my B31YS (Robotic Systems) coursework at Heriot-Watt University, originally developed and run on The Construct's ROS1 cloud platform. This repo ports the assignment to run locally on ROS2, with an added feature for selecting different robot motion patterns at launch.

## What this is

Three ROS2 nodes simulating a mobile robot and estimating its position with a Kalman filter:

- **`student_trajectory`** — drives a simulated robot along a chosen motion pattern (circle, square, figure-8, curvy line, or spiral) and publishes ground-truth odometry on `/odom`.
- **`fake_gps`** — takes the true position and publishes a noisy GPS-style estimate on `/gps`, simulating real-world sensor noise.
- **`kalman_filter`** — fuses the motion model and noisy GPS readings to produce a smoothed position estimate on `/kalman_estimate`.

RViz2 visualizes all three: ground truth, raw noisy GPS, and the filtered estimate.

## Requirements

- Ubuntu with ROS2 installed (developed against ROS2 "Lyrical Luth" on Ubuntu 26.04; should work on any recent ROS2 distro with minor adjustments)
- `colcon` build tools (`sudo apt install python3-colcon-common-extensions`)

## Setup

Clone into a ROS2 workspace's `src/` folder:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/maryamxm/Kalman_Filter.git kalman_demo
```

Build:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Running it

```bash
ros2 launch kalman_demo kalman_launch.py pattern:=circle
```

Available `pattern` values: `circle`, `square`, `figure8`, `curvy`, `spiral`. Defaults to `circle` if omitted.

RViz2 opens automatically and shows:
- **Blue** — ground-truth odometry (`/odom`)
- **Gold** — Kalman filter estimate (`/kalman_estimate`)

## Project structure

```
kalman_demo/
├── kalman_demo/            # node source files
│   ├── student_trajectory.py
│   ├── fake_gps.py
│   └── kalman_filter.py
├── launch/
│   └── kalman_launch.py
├── rviz/
│   └── kalman_demo.rviz
├── package.xml
├── setup.py
└── setup.cfg
```

## Background

The Kalman filter logic (prediction/correction steps, noise handling) is my own implementation from the original assignment. The assignment brief and any supplied template code are Heriot-Watt University coursework material and are not included in this repo. The ROS2 port, launch tooling, pattern-selection feature, and RViz configuration were built by me to run the assignment outside of the original cloud platform.

## License

MIT — see `LICENSE`. Covers my own code in this repo only; it does not grant any rights to the original assignment brief or university-supplied material.
