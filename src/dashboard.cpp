#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <filesystem>
#include <sstream>  // For stringstream handling
#include <iomanip>  // For fixed precision float printing

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <ament_index_cpp/get_package_share_path.hpp> 
#include <opencv2/opencv.hpp>
#include "camera_processing/msg/detection_result.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

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

  bool render(const cv::Mat& frame) {
    if (!is_first_run_) {
      try {
        double prop = cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE);
        if (prop < 1.0) {
          return false;
        }
      } catch (const cv::Exception&) {
        return false; 
      }
    }

    cv::Mat canvas;
    if (frame.empty()) {
      canvas = cv::Mat::zeros(480, 640, CV_8UC3);
      draw_centered_text(canvas, "Connecting to camera stream...", 0.6, cv::Scalar(0, 0, 255));
    } else {
      canvas = frame.clone();
    }

    cv::imshow(window_name_, canvas);
    cv::waitKey(1);

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
// 2. MODEL SELECTOR GUI
// ============================================================================
class ModelSelectorGUI {
public:
  ModelSelectorGUI(const std::string& window_name, 
                   int* active_model_ptr, 
                   const std::vector<std::string>& models) 
    : window_name_(window_name) 
  {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 850, 180);

    const int max_value = std::max(1, static_cast<int>(models.size()) - 1);
    cv::createTrackbar(
      "Model Select",
      window_name_,
      active_model_ptr,
      max_value,
      nullptr
    );

    // Create a matching dark gray canvas background
    bg_canvas_ = cv::Mat::zeros(180, 850, CV_8UC3);
  }

  ~ModelSelectorGUI() {
    cv::destroyWindow(window_name_);
  }

  // Accepts active index and model list to render the text feed below the slider
  bool render(int active_idx, const std::vector<std::string>& models, const std::string& latency_str) {
    try {
      double prop = cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE);
      if (prop < 1.0) {
        return false;
      }
    } catch (const cv::Exception&) {
      return false;
    }

    // 1. Clear the canvas with a solid dark gray panel
    bg_canvas_ = cv::Scalar(45, 45, 45);

    // 2. Get the name of the active model
    int safe_idx = std::max(0, std::min(active_idx, static_cast<int>(models.size()) - 1));
    std::string model_name = models[safe_idx];
    
    std::string display_label = model_name + " (" + latency_str + ")";

    // 3. Draw "Active Model:" subtitle
    cv::putText(bg_canvas_, "Active Model Telemetry:", cv::Point(30, 50), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

    // 4. Draw the actual file name in vibrant green
    cv::putText(bg_canvas_, display_label, cv::Point(30, 115), 
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    cv::imshow(window_name_, bg_canvas_);
    return true;
  }

private:
  std::string window_name_;
  cv::Mat bg_canvas_;
};

// ============================================================================
// 3. DASHBOARD NODE
// ============================================================================
class DashboardNode : public rclcpp::Node {
public:
  DashboardNode() : Node("dashboard"), active_model_index_(0), last_model_index_(-1) {
    const std::string window_name = "Dashboard Stream";
    const std::string selector_window_name = "Model Controller";
    
    std::string package_share_directory;
    try {
      package_share_directory = ament_index_cpp::get_package_share_path("camera_processing").string();
      RCLCPP_INFO(this->get_logger(), "Found share path: %s", package_share_directory.c_str());
    } catch (const std::exception& e) {
      RCLCPP_FATAL(this->get_logger(), "Could not find package share directory: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    fs::path model_dir = fs::path(package_share_directory) / "models";
    RCLCPP_INFO(this->get_logger(), "Path used to scan models: %s", model_dir.c_str());
    
    scan_model_directory(model_dir.string());

    image_gui_ = std::make_unique<ImageGUI>(window_name);
    model_selector_gui_ = std::make_unique<ModelSelectorGUI>(selector_window_name, &active_model_index_, discovered_models_);

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.reliable();
    qos_profile.transient_local();
    model_pub = this->create_publisher<std_msgs::msg::String>("/selected_model", qos_profile);

    subscription_ = this->create_subscription<camera_processing::msg::DetectionResult>(
      "camera/image", 10, std::bind(&DashboardNode::result_callback, this, std::placeholders::_1));

    gui_timer_ = this->create_wall_timer(33ms, std::bind(&DashboardNode::update_gui_loop, this));
 
    one_shot_timer_ = this->create_wall_timer(0ms, [this]() {
      auto initial_msg = std_msgs::msg::String();
      initial_msg.data = get_selected_model_name();
      model_pub->publish(initial_msg);
      last_model_index_ = active_model_index_;
      one_shot_timer_->cancel();
    });

    RCLCPP_INFO(this->get_logger(), "Dynamic ImageGUI Initialized.");
  }

private:
  void scan_model_directory(const std::string& path) {
    discovered_models_.clear();
    discovered_models_.push_back("No Model");

    if (!fs::exists(path) || !fs::is_directory(path)) {
      RCLCPP_INFO(this->get_logger(), "Model directory '%s' missing.", path.c_str());
      return;
    }

    for (const auto& entry : fs::directory_iterator(path)) {
      if (entry.is_regular_file()) {
        std::string ext = entry.path().extension().string();
        if (ext == ".onnx" || ext == ".pb" || ext == ".xml") {
          discovered_models_.push_back(entry.path().filename().string());
        }
      }
    }
  }

  void result_callback(const camera_processing::msg::DetectionResult::SharedPtr msg) {
    try {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = cv_bridge::toCvCopy(msg->image, "bgr8")->image;
      
      // Build string displaying all metrics with 1 decimal space precision for the average
      std::stringstream ss;
      ss << "Cur: " << msg->inference_time << "ms | "
         << "Min: " << msg->min_inference_time << "ms | "
         << "Max: " << msg->max_inference_time << "ms | "
         << "Avg: " << std::fixed << std::setprecision(1) << msg->avg_inference_time << "ms";
      current_latency_str_ = ss.str();
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  std::string get_selected_model_name() const {
    if (active_model_index_ < 0 || active_model_index_ >= static_cast<int>(discovered_models_.size())) {
      return "No Model";
    }
    return discovered_models_[active_model_index_];
  }

  void update_gui_loop() {
    std::string lat_str;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      lat_str = current_latency_str_;
    }

    if (!model_selector_gui_->render(active_model_index_, discovered_models_, lat_str)) {
      RCLCPP_INFO(this->get_logger(), "Control window closed. Shutting down cleanly.");
      rclcpp::shutdown();
      return;
    }

    if (active_model_index_ != last_model_index_) {
      auto msg = std_msgs::msg::String();
      msg.data = get_selected_model_name();
      model_pub->publish(msg);
      last_model_index_ = active_model_index_;
    }

    cv::Mat frame_to_render;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!latest_frame_.empty()) {
        frame_to_render = latest_frame_.clone();
      }
    }

    if (!image_gui_->render(frame_to_render)) {
      RCLCPP_INFO(this->get_logger(), "Window closed. Shutting down node cleanly.");
      rclcpp::shutdown();
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr model_pub;
  rclcpp::Subscription<camera_processing::msg::DetectionResult>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr gui_timer_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_;
  
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  std::string current_latency_str_ = "Cur: 0ms | Min: 0ms | Max: 0ms | Avg: 0.0ms";
  
  std::unique_ptr<ImageGUI> image_gui_;
  std::unique_ptr<ModelSelectorGUI> model_selector_gui_;

  std::vector<std::string> discovered_models_;
  int active_model_index_;
  int last_model_index_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DashboardNode>());
  rclcpp::shutdown();
  return 0;
}