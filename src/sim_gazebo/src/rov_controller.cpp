#include "rov_controller.hpp"
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{

void ROVController::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    model_ = model;
    world_ = model_->GetWorld();
    prev_time_ = world_->SimTime();

    // Initialize ROS node
    node_ = gazebo_ros::Node::Get(sdf);

    sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&ROVController::OnCmdVel, this, std::placeholders::_1));
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  
    pub_camera_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera", 10);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ROVController::OnUpdate, this));

    auto sensor = gazebo::sensors::SensorManager::Instance()->GetSensor("camera_sensor");
    if (sensor)
    {
        camera_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sensor);
        camera_ = camera_sensor_->Camera();
        camera_connection_ = camera_->ConnectNewImageFrame(
            std::bind(&ROVController::OnNewFrame, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4,
                      std::placeholders::_5));
    }

    RCLCPP_INFO(node_->get_logger(), "ROVController plugin loaded successfully");
}

void ROVController::OnUpdate()
{
    common::Time currTime = world_->SimTime();
    prev_time_ = currTime;

    ignition::math::Pose3d pose = model_->WorldPose();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = node_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);
    auto linear_vel = model_->WorldLinearVel();
    odom.twist.twist.linear.x = linear_vel.X();
    odom.twist.twist.linear.y = linear_vel.Y();
    odom.twist.twist.linear.z = linear_vel.Z();

    auto angular_vel = model_->WorldAngularVel();
    odom.twist.twist.angular.x = angular_vel.X();
    odom.twist.twist.angular.y = angular_vel.Y();
    odom.twist.twist.angular.z = angular_vel.Z();

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = node_->now();
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(pose);
    tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_INFO(node_->get_logger(), "Publishing odometry...");
}


void ROVController::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    ignition::math::Pose3d pose = model_->WorldPose();
    model_->SetLinearVel(ignition::math::Vector3d(
        msg->linear.x * cosf(pose.Rot().Yaw()) - msg->linear.y * sinf(pose.Rot().Yaw()),
        msg->linear.y * cosf(pose.Rot().Yaw()) + msg->linear.x * sinf(pose.Rot().Yaw()),
        msg->linear.z));
    model_->SetAngularVel(ignition::math::Vector3d(msg->angular.x, msg->angular.y, msg->angular.z));
}

void ROVController::OnNewFrame(const unsigned char *image,
                               unsigned int width, unsigned int height,
                               unsigned int depth, const std::string &format)
{
    sensor_msgs::msg::Image msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "camera_frame";
    msg.height = height;
    msg.width = width;
    msg.encoding = "rgb8";
    msg.is_bigendian = 0;
    msg.step = width * 3;
    msg.data.resize(height * msg.step);

    memcpy(msg.data.data(), image, height * msg.step);
    pub_camera_->publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(ROVController)
} 
