# BumperBot ROS 2 Navigation Project

A complete ROS 2 (Humble) mobile robot simulation project focused on **mapping, localization, and navigation** using **Nav2**, **Gazebo**, and **RViz**. This project demonstrates end-to-end robot navigation starting from environment setup to autonomous motion in a simulated world.

---

## 🚀 Features

* Differential-drive mobile robot (BumperBot)
* Simulation using **Gazebo**
* Visualization with **RViz2**
* Keyboard-based teleoperation
* Modular ROS 2 package structure
* Support for **SLAM (slam_toolbox)** and **localization (AMCL / particle filter)**

---

## ⚙️ Requirements

* WSL Ubuntu 22.04
* ROS 2 Humble
* Gazebo Sim (Fortress)
* RViz2
* `key_teleop` package

---

## 🔧 Build Instructions

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ Running the Simulation

The project supports **two operating modes** using a **single unified launch file**, controlled via launch arguments.

### 🔹 Navigation Mode (Pre-built Map + AMCL)

This mode uses a **pre-existing occupancy grid map** and performs **particle filter–based localization (AMCL)** with the **Nav2** stack enabled.

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house
```

**Use this mode when:**

* A map of the environment already exists
* You want to test localization and autonomous navigation
* Global and local planners from Nav2 are required

---

### 🔹 SLAM Mode (Online Mapping with slam_toolbox)

This mode enables **online SLAM** to build the occupancy grid map in an **unknown environment** using **slam_toolbox**. Localization and mapping run simultaneously.

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house
```

**What this does internally:**

* Launches the robot in Gazebo
* Starts **slam_toolbox** in online mode
* Continuously updates the occupancy grid
* Publishes the map for visualization in RViz

**Use this mode when:**

* No prior map is available
* You want to generate a map from scratch
* Exploring new or modified environments

---

## 🎮 Teleoperation (Keyboard Control)

To manually drive the robot using the keyboard:

```bash
ros2 run key_teleop key_teleop
```

Ensure the simulation is running before starting teleoperation.

---

## 🧠 Concepts Demonstrated

* Robot description using URDF
* ROS 2 launch system with runtime arguments
* Gazebo–ROS 2 integration
* Controller setup using `ros2_control`
* Online SLAM using **slam_toolbox**
* Particle filter–based localization (AMCL)
* Autonomous navigation using the **Nav2** stack

---

## 🧪 Future Work

* Multi-floor and dynamic environment support
* Comparison of different SLAM backends
* Advanced local planner and costmap tuning
* Integration with **Isaac Sim** for high-fidelity simulation
* Research on custom global and local planners

---

## 📸 Demo
https://github.com/user-attachments/assets/abd3e289-31df-41c1-b88b-9eee5f46737d

