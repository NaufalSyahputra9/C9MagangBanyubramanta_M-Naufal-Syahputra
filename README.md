# Tugas Magang C9Banyubramanta 2025

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green)](https://opencv.org/)
[![OpenVINO](https://img.shields.io/badge/OpenVINO-YOLOv5-purple)](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/overview.html)

---

## 1. ROS 2 Joystick Controller Project (Humble)

**Overview:**  
Integrates a joystick with ROS 2 to control a custom controller node. Subscribes to joystick inputs and publishes velocity and orientation commands to the robot.

**Dependencies:**
- ROS 2 Humble
- `joy` package
- Custom message `interfaces/msg/Controller`

```bash
sudo apt install ros-humble-joy
```

**Note:**  
Subscribes to `/joy` and publishes to `/cmd_vel`. Depth and yaw are incrementally adjusted based on joystick axes.

---

## 2. OpenCV Masking Image

**Overview:**  
Captures video frames and creates a mask based on HSV color ranges. Publishes raw and masked images to ROS2 topics.

**Dependencies:**
- OpenCV
- ROS 2 Humble
- `cv_bridge`, `image_transport`

```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

**Note:**  
Reads video frames, converts to HSV, applies threshold masking, and publishes results to `camera` and `mask` topics.

---

## 3. OpenCV Masking Video with OpenVINO & ROS2

**Overview:**  
Uses OpenVINO YOLOv5 for real-time object detection in videos via ROS2. Publishes detected objects info and annotated video frames.

**Dependencies:**
- OpenVINO Toolkit
- OpenCV
- ROS 2 Humble
- `cv_bridge`

```bash
pip install openvino-dev[onnx]
```

**Note:**  
Detects two classes: Baskom & Flare. Publishes bounding boxes, confidence, and annotated frames to `objects` and `objects_box` topics.

---

## 4. Serial Communication (STM32) to ROS2

**Overview:**  
ROS2 node sends controller data to STM32 via serial communication. Listens to `/cmd_vel` and writes x, y, yaw, depth values to the serial port.

**Dependencies:**
- ASIO library
- ROS 2 Humble
- `interfaces/msg/Controller`

```bash
sudo apt install libasio-dev
```

**Note:**  
Enables hardware-in-the-loop testing by transmitting ROS2 control messages to STM32 microcontroller.

---

## 5. 6 DoF Gazebo Simulation

**Overview:**  
Simulates a 6 DOF ROV in Gazebo with ROS2 integration for movement and camera. Publishes odometry and camera frames while subscribing to velocity commands.

**Dependencies:**
- Gazebo
- ROS 2 Humble
- `gazebo_ros_pkgs`

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

**Note:**  
Includes odometry publishing, velocity command handling, and camera streaming for testing robot behavior in a virtual environment.

---

## 6. OpenCV Masking with Trackbar

**Overview:**  
Interactive HSV masking with trackbars to tune color detection in real-time. Displays original and masked video streams.

**Dependencies:**
- OpenCV

```bash
sudo apt install libopencv-dev
```

**Note:**  
Useful for selecting HSV ranges for object detection or tracking, allowing live parameter adjustments.

---
