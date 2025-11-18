#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

class ImageNode : public rclcpp::Node {
public:
    ImageNode() : Node("image_node")
    {
        raw_pub_= this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
        mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("mask", 10);;

        std::string video_path = "src/opencv_img/include/fourth.mp4";
        cap_.open(video_path);
        if(!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Camera tidak bisa dibuka!");
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ImageNode::process_frame, this)
        );
    }
 
public:
    void process_frame()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        cv::Mat hsv, mask;

        // convert BGR ke HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        int h_max = 10, h_min = 0;
        int s_min = 120, s_max = 255;
        int v_min = 70, v_max = 255;
        cv::Scalar minHSV(h_min, s_min, v_min);  
        cv::Scalar maxHSV(h_max, s_max, v_max);   

        // mask tunggal
        cv::inRange(hsv, minHSV, maxHSV, mask);

        cv::Mat imgThresh;
        cv::inRange(hsv, minHSV, maxHSV, imgThresh);
            
        cv::erode(imgThresh, imgThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate( imgThresh, imgThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

        cv::dilate( imgThresh, imgThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(imgThresh, imgThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
       

        // publish raw image
        sensor_msgs::msg::Image::SharedPtr raw_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        raw_pub_->publish(*raw_msg);

        // publish mask image
        sensor_msgs::msg::Image::SharedPtr mask_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", imgThresh).toImageMsg();
        mask_pub_->publish(*mask_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageNode>());
    rclcpp::shutdown();
    return 0;
}
