#ifndef ROV_CONTROLLER_HPP
#define ROV_CONTROLLER_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "pid.hpp"

namespace gazebo
{
class ROVController : public ModelPlugin
{
public:
    ROVController() = default;

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;
    void OnUpdate();
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void OnNewFrame(const unsigned char *image, unsigned int width,
                    unsigned int height, unsigned int depth,
                    const std::string &format);

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Gazebo
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_connection_;
    common::Time prev_time_;

    // ROS
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // PID controllers
    PID pid_lin_x_{1.0, 0.0, 0.0};
    PID pid_lin_y_{1.0, 0.0, 0.0};
    PID pid_lin_z_{1.0, 0.0, 0.0};
    PID pid_ang_roll_{1.0, 0.0, 0.0};
    PID pid_ang_pitch_{1.0, 0.0, 0.0};
    PID pid_ang_yaw_{1.0, 0.0, 0.0};

    // Target velocity
    ignition::math::Vector3d target_lin_vel_;
    ignition::math::Vector3d target_ang_vel_;

    // Camera
    gazebo::sensors::CameraSensorPtr camera_sensor_;
    gazebo::rendering::CameraPtr camera_;
    event::ConnectionPtr camera_connection_;
};
} // namespace gazebo

#endif
