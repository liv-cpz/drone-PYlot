# drone-PYlot - Flight Control and Autonomous Landing on a Moving Target  
*DJI Tello EDU – MTRX5700 Major Project*

> Vision-based tracking • PID descent • ROS 2 Gazebo simulation  
> University of Sydney – AMME – Semester 1 2025

<!-- ——————————————————————————————————— -->
## 1 Project Overview  
This repository contains all software developed for MTRX5700 major project:  
*“Flight Control and Autonomous Landing on a Moving Target using a DJI Tello Drone.”*  
The system lets a **DJI Tello EDU** detect a mobile landing platform, align itself, and land autonomously by combining:

1. **Colour-based tracking** for long-range guidance.  
2. **ArUco marker pose-estimation** for fine alignment and touchdown.  
3. **PID-based closed-loop control** tuned through simulation and live tests.  
4. A **ROS 2 (Gazebo 11) world** that reproduces the scenario for rapid iteration before flight.

<!-- ——————————————————————————————————— -->
## 2 Repository Layout  
├── src/
│ └── main.py # Appendix A – online CV + flight controller
│ └── drone_terminal_simulator.py # Appendix B – log-polynomial PID tuner
│ └── aruco_trajectory_estimator.py # Appendix C – target path predictor
├── simulation/ # ROS 2 Humble packages for simulation testing
│ ├── aruco_det/ # Aruco detection node
│ ├── tello_ros/ # tello DJI used from https://github.com/ACFR-RPG/tello-driver-ros?tab=readme-ov-file
│ └── calibrationdata.tar.gz # Tello drone camera calibration file
└── README.md

<!-- ——————————————————————————————————— -->
## 3 Demostration Videos
1. Drone landing: https://youtu.be/6jwKVQCQYvo
2. Simulation: https://www.youtube.com/watch?v=y8ItKe9LRSs