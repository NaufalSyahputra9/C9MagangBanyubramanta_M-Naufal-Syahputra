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
