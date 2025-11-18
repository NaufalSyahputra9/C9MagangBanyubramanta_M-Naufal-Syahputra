#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/controller.hpp"
#include <asio.hpp>
#include <iostream>
#include <sstream>


class SendPortNode : public rclcpp::Node
{ public: SendPortNode() : Node("serial_com_stm32"), io_(), port_(io_)
  {

    const std::string PORT = "/tmp/ttyv0";
    port_.open(PORT);

    port_.set_option(asio::serial_port_base::baud_rate(115200));
    port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::software));
    port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    port_.set_option(asio::serial_port_base::character_size(8));

    sub_ = this->create_subscription<interfaces::msg::Controller>("cmd_vel", 10, std::bind(&SendPortNode::callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "SendPort node started, listening to /cmd_vel...");
  }

private:
    asio::io_context io_;
    asio::serial_port port_;
    rclcpp::Subscription<interfaces::msg::Controller>::SharedPtr sub_;

  void callback(const interfaces::msg::Controller::SharedPtr msg)
  {
    std::ostringstream out;
    out << msg->x << msg->y << msg->yaw << msg->depth;

    try {
      asio::write(port_, asio::buffer(out.str()));
      RCLCPP_INFO(this->get_logger(), "Send data to STM32 : %s", out.str().c_str());
    }
    catch(std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "Failed to send data to STM: %s", e.what());
      }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendPortNode>());
  rclcpp::shutdown();
  return 0;
}
