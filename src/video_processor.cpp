#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "nlohmann/json.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "camera_processing/msg/detection_result.hpp"

using namespace std::chrono_literals;

class GenericDetector {
public:
    // Silenced the unused parameter warning by commenting out the variable name
    GenericDetector(const std::string& /*model_name*/, const std::string& model_path, const std::string& json_path) {
        std::ifstream f(json_path);
        nlohmann::json cfg = nlohmann::json::parse(f);
        model_type_ = cfg["model_type"];
        input_size_ = cv::Size(cfg["input_size"][0], cfg["input_size"][1]);
        scale_ = cfg["scale"];
        
        if (cfg.contains("anchors")) anchors_ = cfg["anchors"].get<std::vector<float>>();

        // DYNAMIC PREPROCESSING: Load parameters from JSON with safe fallbacks
        if (cfg.contains("mean") && cfg["mean"].is_array() && cfg["mean"].size() == 3) {
            mean_ = cv::Scalar(cfg["mean"][0], cfg["mean"][1], cfg["mean"][2]);
        } else {
            mean_ = cv::Scalar(0.0, 0.0, 0.0);
        }
        swap_rb_ = cfg.value("swap_rb", true);
        crop_ = cfg.value("crop", false);

        // Labels
        std::string class_path = json_path.substr(0, json_path.find_last_of("/\\") + 1) + (std::string)cfg["classes"];
        std::ifstream class_file(class_path);
        std::string line;
        while (std::getline(class_file, line)) if (!line.empty()) labels_.push_back(line);

        // Load the model
        net_ = cv::dnn::readNet(model_path);
    }

    int64_t detect(cv::Mat& frame) {
        // Use dynamic parameters for blob creation
        cv::Mat blob = cv::dnn::blobFromImage(frame, scale_, input_size_, mean_, swap_rb_, crop_);
        net_.setInput(blob);
        std::vector<cv::Mat> outputs;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        std::vector<int> ids; std::vector<float> confs; std::vector<cv::Rect> boxes;

        if (model_type_ == "ssd") {
            parse_ssd(outputs[0], frame.size(), boxes, confs, ids);
        } else if (model_type_ == "yolov2") {
            parse_yolov2(outputs[0], frame.size(), boxes, confs, ids);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("GenericDetector"), "No parser for model type: %s", model_type_.c_str());
            throw std::runtime_error("No parser for selected model");
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confs, 0.4f, 0.4f, indices);
        for (int idx : indices) visualize(frame, boxes[idx], confs[idx], ids[idx]);

        return duration;
    }

private:
    std::string model_type_;
    std::vector<std::string> labels_;
    std::vector<float> anchors_;
    cv::dnn::Net net_;
    cv::Size input_size_;
    double scale_;
    
    // Dynamic preprocessing state variables
    cv::Scalar mean_;
    bool swap_rb_;
    bool crop_;

    void visualize(cv::Mat& frame, const cv::Rect& box, float conf, int id) {
        std::string label = (id >= 0 && id < (int)labels_.size()) ? labels_[id] : "ID:" + std::to_string(id);
        cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, label + " " + std::to_string((int)(conf*100))+"%", cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    void parse_ssd(const cv::Mat& output, cv::Size fs, std::vector<cv::Rect>& b, std::vector<float>& c, std::vector<int>& i) {
        cv::Mat detections(output.size[2], output.size[3], CV_32F, output.data);
        for (int row = 0; row < detections.rows; ++row) {
            float conf = detections.at<float>(row, 2);
            if (conf > 0.5f) {
                b.emplace_back((int)(detections.at<float>(row, 3) * fs.width), (int)(detections.at<float>(row, 4) * fs.height),
                               (int)((detections.at<float>(row, 5) - detections.at<float>(row, 3)) * fs.width),
                               (int)((detections.at<float>(row, 6) - detections.at<float>(row, 4)) * fs.height));
                c.push_back(conf); i.push_back((int)detections.at<float>(row, 1) - 1);
            }
        }
    }

    void parse_yolov2(const cv::Mat& output, cv::Size fs, std::vector<cv::Rect>& b, std::vector<float>& c, std::vector<int>& i) {
        static constexpr float OBJECTNESS_THRESHOLD = 0.3f;
        static constexpr float CONFIDENCE_THRESHOLD = 0.3f;

        // Calculate total anchor boxes (each anchor has 2 values: width and height)
        int num_anchors = anchors_.size() / 2;
        
        // Extract the grid dimension (e.g., 13 for a 13x13 feature map grid)
        int grid = output.size[2];
        
        // Lambda for Sigmoid activation to squash raw logits into 0.0 - 1.0 ranges
        auto sig = [](float x) { return 1.0f / (1.0f + std::exp(-x)); };
        
        // MODERN C++ CAST: Explicitly reinterpret the raw byte buffer pointer (uchar*) as a float pointer
        const float* d = reinterpret_cast<const float*>(output.data);
        
        // Stride is the number of channels per anchor box (5 attributes + number of classes)
        int stride = output.size[1] / num_anchors;

        // Loop through every anchor box configuration
        for (int a = 0; a < num_anchors; ++a) {
            // Loop through every row of the grid map
            for (int r = 0; r < grid; ++r) {
                // Loop through every column of the grid map
                for (int col = 0; col < grid; ++col) {
                    
                    // Compute the base pointer offset for the current anchor at grid cell (col, r)
                    int offset = a * stride * grid * grid + (r * grid + col);
                    
                    // Index 4 corresponds to the raw objectness confidence score channel
                    float obj = sig(d[offset + 4 * grid * grid]);
                    
                    // Fast rejection: Skip calculation early if no object is likely to exist here
                    if (obj < OBJECTNESS_THRESHOLD) continue;
                    
                    // Loop through all potential object class labels
                    for (int cls = 0; cls < (int)labels_.size(); ++cls) {
                        
                        // Class logits start at channel index 5 onwards
                        float prob = sig(d[offset + (5 + cls) * grid * grid]);
                        
                        // Calculate final combined probability (Objectness * Class Probability)
                        if (obj * prob > CONFIDENCE_THRESHOLD) {
                            
                            // Decode bounding box center coordinates relative to the grid cell
                            // Channel 0 = tx (X offset), Channel 1 = ty (Y offset)
                            float x = (col + sig(d[offset])) / grid;
                            float y = (r + sig(d[offset + grid * grid])) / grid;
                            
                            // Decode bounding box dimensions scaled against the specific anchor dimensions
                            // Channel 2 = tw (Width log-scaling), Channel 3 = th (Height log-scaling)
                            float w = std::exp(d[offset + 2 * grid * grid]) * anchors_[a * 2] / grid;
                            float h = std::exp(d[offset + 3 * grid * grid]) * anchors_[a * 2 + 1] / grid;
                            
                            // Convert normalized center-based coordinates (x, y, w, h) into absolute 
                            // pixel-based top-left corner coordinates required by cv::Rect
                            b.emplace_back(
                                (int)((x - w/2) * fs.width), 
                                (int)((y - h/2) * fs.height), 
                                (int)(w * fs.width), 
                                (int)(h * fs.height)
                            );
                            
                            // Record the valid combined confidence score and the matching class ID
                            c.push_back(obj * prob); 
                            i.push_back(cls);
                        }
                    }
                }
            }
        }
    }
};

class DetectorFactory {
public:
    static std::shared_ptr<GenericDetector> getDetector(const std::string& msg_data) {
        static std::map<std::string, std::shared_ptr<GenericDetector>> cache;
        if (msg_data == "No Model") return nullptr;
        if (cache.find(msg_data) == cache.end()) {
            std::string pkg_path = ament_index_cpp::get_package_share_path("camera_processing").string();
            std::string base = msg_data.substr(0, msg_data.find_last_of('.'));
            cache[msg_data] = std::make_shared<GenericDetector>(msg_data, pkg_path + "/models/" + msg_data, pkg_path + "/models/" + base + ".json");
        }
        return cache[msg_data];
    }
};

class VideoProcessor : public rclcpp::Node {
public:
    VideoProcessor() : Node("video_processor_node") {
        result_pub_ = this->create_publisher<camera_processing::msg::DetectionResult>("camera/image", 10);
        
        rclcpp::QoS qos(1); qos.transient_local();
        model_sub_ = this->create_subscription<std_msgs::msg::String>("/selected_model", qos, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                current_detector_ = DetectorFactory::getDetector(msg->data);
                current_model_name_ = msg->data;
                min_time_ = 0; max_time_ = 0; total_time_ = 0; frame_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "Loaded: %s", msg->data.c_str());
            });
        cap_.open(0);
        timer_ = this->create_wall_timer(33ms, std::bind(&VideoProcessor::timer_callback, this));
    }
private:
    void timer_callback() {
        cv::Mat frame;
        if (cap_.read(frame)) {
            int64_t duration = 0;
            if (current_detector_) {
                duration = current_detector_->detect(frame);
                int32_t current_ms = static_cast<int32_t>(duration);
                if (frame_count_ == 0) { min_time_ = current_ms; max_time_ = current_ms; }
                else { if (current_ms < min_time_) min_time_ = current_ms; if (current_ms > max_time_) max_time_ = current_ms; }
                total_time_ += current_ms; frame_count_++;
            }
            
            camera_processing::msg::DetectionResult msg;
            msg.image = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            msg.inference_time = static_cast<int32_t>(duration);
            msg.model_name = current_model_name_;
            msg.min_inference_time = min_time_;
            msg.max_inference_time = max_time_;
            msg.avg_inference_time = frame_count_ > 0 ? static_cast<float>(total_time_) / frame_count_ : 0.0f;
            result_pub_->publish(msg);
        }
    }
    cv::VideoCapture cap_;
    std::string current_model_name_ = "No Model";
    int32_t min_time_ = 0, max_time_ = 0;
    int64_t total_time_ = 0, frame_count_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<camera_processing::msg::DetectionResult>::SharedPtr result_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_sub_;
    std::shared_ptr<GenericDetector> current_detector_ = nullptr;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoProcessor>());
    rclcpp::shutdown();
    return 0;
}