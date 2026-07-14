#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

class Dashboard : public rclcpp::Node {
public:
    Dashboard() : Node("dashboard_node") {
        // Subscribe to the incoming video stream
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&Dashboard::image_callback, this, std::placeholders::_1));
        
        cv::namedWindow("Dashboard UI", cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "Dashboard UI initialized.");
    }

    ~Dashboard() {
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        try {
            // Convert ROS Image Message back to OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            
            // Render frame
            cv::imshow("Dashboard UI", cv_ptr->image);
            cv::waitKey(1); 
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dashboard>());
    rclcpp::shutdown();
    return 0;
}
