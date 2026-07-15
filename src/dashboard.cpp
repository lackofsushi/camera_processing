#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

// ============================================================================
// 1. DYNAMIC IMAGE GUI
// ============================================================================
class ImageGUI {
public:
  ImageGUI(const std::string& window_name) : window_name_(window_name) {
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
  }

  ~ImageGUI() {
    cv::destroyWindow(window_name_);
  }

  // Returns 'false' if the user closed the window, allowing clean node shutdown
  bool render(const cv::Mat& frame) {
    // 1. Check if the window was closed BEFORE we attempt to draw on it
    if (!is_first_run_) {
      try {
        double prop = cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE);
        if (prop < 1.0) { // If closed or destroyed
          return false;
        }
      } catch (const cv::Exception& e) {
        // Safe fallback in case the window property query throws an exception on closure
        return false; 
      }
    }

    // 2. Prepare the frame canvas
    cv::Mat canvas;
    if (frame.empty()) {
      // Sensible fallback size if no camera data is available yet
      canvas = cv::Mat::zeros(480, 640, CV_8UC3);
      draw_centered_text(canvas, "Connecting to camera stream...", 0.6, cv::Scalar(0, 0, 255));
    } else {
      canvas = frame.clone();
    }

    // 3. Render and process events
    cv::imshow(window_name_, canvas);
    cv::waitKey(1); // Keeps GUI responsive

    is_first_run_ = false;
    return true;
  }

private:
  // Dynamically scales and centers text based on the size of the canvas passed in
  void draw_centered_text(cv::Mat& img, const std::string& text, double font_scale, cv::Scalar color) {
    int face = cv::FONT_HERSHEY_SIMPLEX;
    int thickness = 2;
    int baseline = 0;

    // Get text box dimensions in pixels
    cv::Size text_size = cv::getTextSize(text, face, font_scale, thickness, &baseline);

    // Calculate exact center coordinates
    int x = (img.cols - text_size.width) / 2;
    int y = (img.rows + text_size.height) / 2;

    cv::putText(img, text, cv::Point(x, y), face, font_scale, color, thickness);
  }

  std::string window_name_;
  bool is_first_run_ = true;
};


// ============================================================================
// 2. DASHBOARD NODE (Camera subscriber and safe shutdown logic)
// ============================================================================
class DashboardNode : public rclcpp::Node {
public:
  DashboardNode() : Node("dashboard") {
    const std::string window_name = "Dashboard Stream";
    
    // Initialize our decoupled, adaptive visualizer
    image_gui_ = std::make_unique<ImageGUI>(window_name);

    // Subscribe to the real camera stream
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10, std::bind(&DashboardNode::image_callback, this, std::placeholders::_1));

    // Spin our rendering pipeline at 30Hz
    gui_timer_ = this->create_wall_timer(33ms, std::bind(&DashboardNode::update_gui_loop, this));

    RCLCPP_INFO(this->get_logger(), "Dynamic ImageGUI Initialized. Close-detection fix active.");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void update_gui_loop() {
    cv::Mat frame_to_render;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!latest_frame_.empty()) {
        frame_to_render = latest_frame_.clone();
      }
    }

    // Render, and check if the user triggered a window close
    if (!image_gui_->render(frame_to_render)) {
      RCLCPP_INFO(this->get_logger(), "Window closed. Shutting down node cleanly.");
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr gui_timer_;
  
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  
  std::unique_ptr<ImageGUI> image_gui_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DashboardNode>());
  rclcpp::shutdown();
  return 0;
}
