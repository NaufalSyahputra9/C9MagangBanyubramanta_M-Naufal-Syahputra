# C9MagangBanyubramanta_M-Naufal-Syahputra
# 🎮 ROS 2 Joystick Controller Project (Humble)

This repository contains the setup for a ROS 2 workspace that connects a joystick input (`joy` package) to a custom controller node via a custom message interface.  
It demonstrates how to **subscribe to joystick data** and **publish to controller node using cmd_vel**

---

## 📑 Table of Contents
1. [Install Dependencies](#-install-dependencies)
2. [Set Up Workspace](#-set-up-workspace)
3. [Set Up Interfaces](#-set-up-interfaces)
4. [Add Topic and Controller Node](#-add-topic-and-controller-node)
5. [Build and Run](#-build-and-run)
6. [Notes](#-notes)

---

# 1. 🧩 Install Dependencies

Start by installing the `joy` package:

```bash
sudo apt-get install ros-humble-joy

```
---
cd ~/ros2_ws/src
ros2 pkg create controller --build-type ament_cmake --dependencies rclcpp geometry_msgs sensor_msgs --license Apache-2.0

# 2. Set Up Workspace
This is my workspace set up:
```bash
~/Magang_banyu/ros2ws/
├── src/
│   ├── interfaces/          # contains custom message definitions
│   ├── controller/          # contains controller node source code and cmd_vel topic
│   └── ...
└── install/
└── build/
└── log/

```

# 3. Set Up Interfaces
## Create a Custom Message
Create the file:
```bash
interfaces/msg/Controller.msg
```

Content of Controller.msg :
```bash
float64 x
float64 y
float64 depth
float64 yaw
```

# 2. Update CMakeList.txt in interfaces
Add message generation dependencies:
```
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Controller.msg"
    DEPENDENCIES std_msgs
)
```

# 4. Add topic
In folder

Create controller node 

## Build and Run

run this in new terminal : 
ros2 run joy joy_node

run this also in new terminal :
ros2 run controller controller node

in new terminal cd ~Magang_banyu/ros2ws/
colcon build

if there is no error
. install
ros2 echo topic /cmd_vel
