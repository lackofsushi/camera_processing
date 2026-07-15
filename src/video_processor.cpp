#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class VideoProcessor : public rclcpp::Node {
public:
    VideoProcessor() : Node("video_processor_node"), current_model_("None") {
        // Create the image publisher (standard volatile QoS is fine here)
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        
        // Define explicit matching Reliable + Transient Local QoS profile
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliable();
        qos_profile.transient_local();

        // Subscribe using the custom matching QoS profile
        model_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/selected_model", 
            qos_profile, 
            std::bind(&VideoProcessor::model_callback, this, std::placeholders::_1)
        );

        // Open the default system camera (0)
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video device!");
            return;
        }

        // Timer for grabbing video frames (~30 FPS)
        timer_ = this->create_wall_timer(33ms, std::bind(&VideoProcessor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Video Processor Node has started. Current model: [ %s ]", current_model_.c_str());
    }

private:
    // Callback that reacts to and logs the published model name
    void model_callback(const std_msgs::msg::String::SharedPtr msg) {
        current_model_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Model updated! Video Processor is now reacting to: [ %s ]", current_model_.c_str());
    }

    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;

        if (!frame.empty()) {
            // Convert OpenCV Mat to ROS Image Message and publish
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            image_publisher_->publish(*msg);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_subscriber_;
    cv::VideoCapture cap_;
    
    // Tracks the current model state safely; defaults to "None" if dashboard isn't active
    std::string current_model_; 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoProcessor>());
    rclcpp::shutdown();
    return 0;
}
