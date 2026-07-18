#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

// --- Isolated Camera Module ---
class Camera {
public:
    Camera(int device_id = 0) {
        cap_.open(device_id);
    }

    bool is_ready() const {
        return cap_.isOpened();
    }

    bool capture(cv::Mat& frame) {
        if (!cap_.isOpened()) return false;
        return cap_.read(frame);
    }

private:
    cv::VideoCapture cap_;
};

// --- Main ROS 2 Node ---
class VideoProcessor : public rclcpp::Node {
public:
    VideoProcessor() : Node("video_processor_node"), 
                       current_model_("No Model"), 
                       camera_(0) {
        
        // --- Critical Hardware Check ---
        if (!camera_.is_ready()) {
            RCLCPP_FATAL(this->get_logger(), "Could not open video device! Shutting down.");
            rclcpp::shutdown();
            return;
        }

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliable();
        qos_profile.transient_local();

        model_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/selected_model", qos_profile, 
            std::bind(&VideoProcessor::model_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(33ms, std::bind(&VideoProcessor::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Video Processor Node initialized successfully.");
        RCLCPP_INFO(this->get_logger(), "Current model: [ %s ]", current_model_.c_str());
    }

private:
    void model_callback(const std_msgs::msg::String::SharedPtr msg) {
        current_model_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Model selection updated to: %s", current_model_.c_str());
    }

    void timer_callback() {
        cv::Mat frame;
        if (camera_.capture(frame)) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            image_publisher_->publish(*msg);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_subscriber_;
    
    std::string current_model_;
    Camera camera_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoProcessor>();
    
    // Check if the node is still running after constructor (in case of failed init)
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    
    rclcpp::shutdown();
    return 0;
}