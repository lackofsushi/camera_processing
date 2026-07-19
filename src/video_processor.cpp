#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <algorithm>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "nlohmann/json.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"

using namespace std::chrono_literals;

class GenericDetector {
public:
    GenericDetector(const std::string& model_name, const std::string& model_path, const std::string& json_path) 
    : model_name_(model_name) {
        std::ifstream f(json_path);
        if (!f.is_open()) throw std::runtime_error("Could not open JSON: " + json_path);
        
        nlohmann::json cfg = nlohmann::json::parse(f);
        input_size_ = cv::Size(cfg["input_size"][0], cfg["input_size"][1]);
        mean_ = cv::Scalar(cfg["mean"][0], cfg["mean"][1], cfg["mean"][2]);
        scale_ = cfg["scale"];
        swapRB_ = cfg["swapRB"];
        crop_ = cfg["crop"];
        anchors_ = cfg["anchors"].get<std::vector<float>>();

        // Load labels
        if (cfg.contains("classes")) {
            std::string class_filename = cfg["classes"];
            size_t last_slash = json_path.find_last_of("/\\");
            std::string class_file_path = json_path.substr(0, last_slash + 1) + class_filename;
            std::ifstream class_file(class_file_path);
            std::string line;
            while (std::getline(class_file, line)) if (!line.empty()) labels_.push_back(line);
        }

        net_ = cv::dnn::readNetFromONNX(model_path);
    }

    void detect(cv::Mat& frame) {
        cv::Mat blob = cv::dnn::blobFromImage(frame, scale_, input_size_, mean_, swapRB_, crop_);
        net_.setInput(blob);
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        parse_darknet_raw(outputs[0], frame.size(), boxes, confidences, class_ids);

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, 0.5f, 0.4f, indices);

        for (int idx : indices) {
            visualize(frame, boxes[idx], confidences[idx], class_ids[idx]);
        }
    }

private:
    std::string model_name_;
    std::vector<std::string> labels_;
    std::vector<float> anchors_;
    cv::dnn::Net net_;
    cv::Size input_size_;
    cv::Scalar mean_;
    double scale_;
    bool swapRB_;
    bool crop_;

    void visualize(cv::Mat& frame, const cv::Rect& box, float confidence, int class_id) {
        std::string label = (class_id >= 0 && class_id < (int)labels_.size()) ? labels_[class_id] : "ID:" + std::to_string(class_id);
        label += " " + std::to_string((int)(confidence * 100)) + "%";

        cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::rectangle(frame, cv::Point(box.x, box.y - labelSize.height - 5), cv::Point(box.x + labelSize.width, box.y), cv::Scalar(0, 255, 0), cv::FILLED);
        cv::putText(frame, label, cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }

    void parse_darknet_raw(const cv::Mat& output, cv::Size frame_size, std::vector<cv::Rect>& boxes, std::vector<float>& confs, std::vector<int>& ids) {
        int num_classes = labels_.empty() ? 80 : (int)labels_.size();
        int num_anchors = anchors_.size() / 2;
        int grid_size = output.size[2];
        int grid_area = grid_size * grid_size;
        int stride = output.size[1] / num_anchors;

        auto sigmoid = [](float x) { return 1.0f / (1.0f + std::exp(-x)); };
        const float* data = (const float*)output.data;

        for (int a = 0; a < num_anchors; ++a) {
            for (int r = 0; r < grid_size; ++r) {
                for (int c = 0; c < grid_size; ++c) {
                    int offset = a * stride * grid_area + (r * grid_size + c);
                    float obj_conf = sigmoid(data[offset + 4 * grid_area]);
                    if (obj_conf < 0.5f) continue;

                    int best_class = 0; float max_prob = 0.0f;
                    for (int i = 0; i < num_classes; ++i) {
                        float prob = sigmoid(data[offset + (5 + i) * grid_area]);
                        if (prob > max_prob) { max_prob = prob; best_class = i; }
                    }

                    if (obj_conf * max_prob > 0.5f) {
                        float cx = (c + sigmoid(data[offset])) / grid_size;
                        float cy = (r + sigmoid(data[offset + grid_area])) / grid_size;
                        float w = std::exp(data[offset + 2 * grid_area]) * anchors_[a * 2] / grid_size;
                        float h = std::exp(data[offset + 3 * grid_area]) * anchors_[a * 2 + 1] / grid_size;
                        boxes.emplace_back((int)((cx - w/2) * frame_size.width), (int)((cy - h/2) * frame_size.height), (int)(w * frame_size.width), (int)(h * frame_size.height));
                        confs.push_back(obj_conf * max_prob); ids.push_back(best_class);
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
        if (cache.find(msg_data) == cache.end()) {
            std::string pkg_path = ament_index_cpp::get_package_share_path("camera_processing").string();
            std::string base_name = msg_data;
            size_t dot_pos = base_name.find_last_of('.');
            if (dot_pos != std::string::npos) base_name = base_name.substr(0, dot_pos);
            cache[msg_data] = std::make_shared<GenericDetector>(msg_data, pkg_path + "/models/" + msg_data, pkg_path + "/models/" + base_name + ".json");
        }
        return cache[msg_data];
    }
};

class VideoProcessor : public rclcpp::Node {
public:
    VideoProcessor() : Node("video_processor_node") {
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        
        // 1. Create a QoS profile with a history depth of 1
        // rclcpp::QoS qos_profile(1);
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.reliable();
        // 2. Set the durability to Transient Local
        qos_profile.transient_local();

        // 3. Apply the QoS profile to the subscription
        model_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/selected_model", 
            qos_profile, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "No Model") {
                    current_detector_ = nullptr;
                    RCLCPP_INFO(this->get_logger(), "Detection disabled.");
                } else {
                    try {
                        current_detector_ = DetectorFactory::getDetector(msg->data);
                        RCLCPP_INFO(this->get_logger(), "Loaded: %s", msg->data.c_str());
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
                    }
                }
            });

        cap_.open(0);
        timer_ = this->create_wall_timer(33ms, std::bind(&VideoProcessor::timer_callback, this));
    }
private:
    void timer_callback() {
        cv::Mat frame;
        if (cap_.read(frame)) {
            if (current_detector_) current_detector_->detect(frame);
            image_publisher_->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());
        }
    }
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr model_subscriber_;
    std::shared_ptr<GenericDetector> current_detector_ = nullptr;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoProcessor>());
    rclcpp::shutdown();
    return 0;
}
