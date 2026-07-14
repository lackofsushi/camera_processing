#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class VideoProcessor : public rclcpp::Node {
public:
    VideoProcessor() : Node("video_processor_node") {
        // Create the image publisher
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        
        // Open the default system camera (0)
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video device!");
            return;
        }

        // Timer for grabbing video frames (~30 FPS)
        timer_ = this->create_wall_timer(33ms, std::bind(&VideoProcessor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Video Processor Node has started.");
    }

private:
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
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoProcessor>());
    rclcpp::shutdown();
    return 0;
}
