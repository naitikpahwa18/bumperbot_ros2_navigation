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

### 1️⃣ Launch Robot with Navigation & Localization (Nav2 + AMCL)

Use this launch file to start the robot with **Nav2** and **AMCL (particle filter–based localization)**:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

This mode assumes a **pre-built map** and enables navigation using the Nav2 stack.

---

### 2️⃣ Launch Robot for Mapping (SLAM Toolbox)

Use the following command to perform **online SLAM** and build the occupancy grid map using **slam_toolbox**:

```bash
ros2 launch bumperbot_mapping slam.launch.py
```

This mode is intended for **map creation** and exploration in unknown environments.

---

## 🎮 Teleoperation (Keyboard Control)

To manually drive the robot using the keyboard:

```bash
ros2 run key_teleop key_teleop
```

Make sure the simulation is already running before starting teleoperation.

---

## 🧠 Concepts Demonstrated

* Robot description using URDF
* ROS 2 launch system
* Gazebo–ROS 2 integration
* Controller setup with `ros2_control`
* SLAM using **slam_toolbox**
* Particle filter–based localization (AMCL)
* Navigation-ready architecture using Nav2

---

## 🧪 Future Work

* Multi-floor and dynamic environment support
* Comparison of different SLAM backends
* Advanced local planners and costmap tuning
* Integration with **Isaac Sim** for high-fidelity simulation
* Custom global / local planner research

---

## 📸 Demo



---
