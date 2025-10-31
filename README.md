# C9MagangBanyubramanta_M-Naufal-Syahputra
# 🎮 ROS 2 Joystick Controller Project (Humble)

This repository contains the setup for a ROS 2 workspace that connects a joystick input (`joy` package) to a custom controller node via a custom message interface.  
It demonstrates how to **subscribe to joystick data** and **publish to controller node using cmd_vel**

---

## 📑 Table of Contents
1. [Install Dependencies](#1-install-dependencies)
2. [Set Up Workspace](#2-set-up-workspace)
3. [Set Up Interfaces](#3-set-up-interfaces)
4. [Add Topic](#4-add-topic)
5. [Build and Run](#5-build-and-run)
6. [Notes](#6-notes)

---

# 1. Install Dependencies

Start by installing the `joy` package:

```bash
sudo apt-get install ros-humble-joy

```
---


# 2. Set Up Workspace
In folder ros2ws/src create package for our controller node :
```
ros2 pkg create controller --build-type ament_cmake --dependencies rclcpp geometry_msgs sensor_msgs --license Apache-2.0
```

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

## Update CMakeList.txt in interfaces
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

Inside ```package.xml``` add :
```
<depend>std_msgs</depend>
<depend>rosidl_default_generators</depend>
  
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

# 4. Add Topic
Now, in ```controller/src``` add new file for our code :
```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/controller.hpp"

using std::placeholders::_1;

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node"), yaw_(0.0), depth_(0.0), prev_depth_axis(0.0), prev_yaw_axis(0.0)
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ControllerNode::joyCallback, this, _1));

    cmd_pub_ = this->create_publisher<interfaces::msg::Controller>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Controller Node started with incremental depth control.");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto cmd = interfaces::msg::Controller();

    cmd.x   = -msg->axes[1] * 255.0;   // depan-belakang
    cmd.y   = -msg->axes[0] * 255.0;   // kiri-kanan

    // kedalaman (depth)
    float depth_axis = msg->axes[4];
    const float threshold = 0.5;     

    if (depth_axis < -threshold && prev_depth_axis >= -threshold)
    {
      depth_ += 1.0;
    }
    else if (depth_axis > threshold && prev_depth_axis <= threshold)
    {
      depth_ -= 1.0;
    }

    prev_depth_axis = depth_axis;
    depth_ = std::clamp(depth_, 0.0f, 10.0f);


    // rotasi (yaw)
    float yaw_axis = msg->axes[3]; 
    const float yaw_step = 30.0;    

    if (yaw_axis > threshold && prev_yaw_axis <= threshold)
      yaw_ += yaw_step;
    else if (yaw_axis < -threshold && prev_yaw_axis >= -threshold)
      yaw_ -= yaw_step;
    
    if (yaw_ > 180.0)
      yaw_ = 0.0;
    else if (yaw_ < -180.0)
      yaw_ = 0.0;

    prev_yaw_axis = yaw_axis;
    prev_depth_axis = depth_axis;

    cmd.depth = depth_;
    cmd.yaw = yaw_;

    cmd_pub_->publish(cmd);
  }
  float yaw_;
  float depth_;           
  float prev_depth_axis;  
  float prev_yaw_axis; 

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<interfaces::msg::Controller>::SharedPtr cmd_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
```

Add executable inside CMakeLists.txt. (controller)
```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
  "rclcpp"
  "sensor_msgs"
  "interfaces"
)

install(
  TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)
```
Inside ```package.xml``` add : (controller)
```
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>interfaces</depend>
```

# 5. Build and Run
After that, in cd ~Magang_banyu/ros2ws build :
```bash
colcon build
```

And do sourcing :
```
. install/setup.bash
```

If there are no errors, open another 2 terminals 
## 🖥️ Terminal 1 – Run Joystick Node
```bash
ros2 run joy joy_node
```

## 🖥️ Terminal 2 – Run Controller Node
```bash
ros2 run controller controller_node
```

## 🖥️ Terminal 3 – Echo Output 
In the same terminal from when we are sourcing 
```bash
ros2 topic echo /cmd_vel
```

# 6. Notes

