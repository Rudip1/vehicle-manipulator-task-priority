# Hands-On Intervention: Kinematic Control of a Vehicle–Manipulator System

This repository contains the complete source code, simulation files, project report, and demonstration videos for:

**"Kinematic Control of a Vehicle–Manipulator System Using Task-Priority Redundancy Resolution"**

### Members
1. [Pravin Oli](mailto:pravin.oli.08@gmail.com)  
2. [Gebrecherkos G.](mailto:chereg2016@gmail.com)  

*Master’s Program in Intelligent Field Robotic Systems (IFRoS)*  
*University of Girona*

---

## Project Overview

This project presents the design and implementation of a task-priority-based kinematic control framework for a mobile manipulator performing **pick–transport–place** tasks. The system combines **resolved-rate motion control** with **task-priority redundancy resolution**, uses **behavior trees** for task sequencing, and supports both **dead reckoning** and **ArUco marker-based localization** for closed-loop control.

---

## System Components

### Hardware
- **Kobuki TurtleBot 2** – Differential-drive mobile base  
- **uFactory uArm Swift Pro** – 4-DOF robotic manipulator with vacuum gripper  
- **Intel RealSense D435i** – RGB-D camera for object detection  
- **RPLidar A2** – 2D LiDAR for environment sensing  
- **Raspberry Pi 4B** – Onboard processor for real deployment

### Software
- **Ubuntu 20.04 LTS** with **ROS Noetic (ROS 1)**  
- Behavior trees implemented using [`py_trees`](https://github.com/splintered-reality/py_trees)  
- Custom Python/C++ scripts for kinematics and motion control  
- Simulation and testing using:
  - [Stonefish Simulator](https://github.com/patrykcieslak/stonefish)  
  - [Stonefish ROS Bridge](https://github.com/patrykcieslak/stonefish_ros)
  
## Demo Video

See inside media folder or click the link below

Drive link  
🔗 [Project Demonstration Video](https://drive.google.com/drive/folders/1aWwxXO2Fg-kpDT9R32qac7A4vbF0WJ9x?usp=sharing)

---

## Installation and Execution

Ensure your system is running **Ubuntu 20.04** with **ROS Noetic** installed and sourced.

To install dependencies, clone, build, and launch the project:
```bash
# Step 1: Navigate to your catkin workspace
cd ~/catkin_ws/src

# Step 2: Clone simulation dependencies
git clone https://github.com/patrykcieslak/stonefish.git
git clone https://github.com/patrykcieslak/stonefish_ros.git

# Step 3: Clone this project
git clone https://github.com/your-username/hands_on_intervention.git

# Step 4: Build and source the workspace
cd ~/catkin_ws
catkin build
source devel/setup.bash

# Step 5: Run the system

# Launch manipulator arm only (no base)
roslaunch hands_on_intervention manipulator_hoi.launch

# Run end-effector position task (individual task testing)
roslaunch hands_on_intervention ee_pose.launch

# Run full pick-and-place task using dead reckoning
roslaunch hands_on_intervention deadreckoning_hoi.launch

# Run full pick-and-place task using ArUco marker detection
roslaunch hands_on_intervention aruco_hoi.launch





