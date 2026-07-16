# camera_processing

ROS 2 package featuring a modular video streaming, real-time visualization, and dynamic model-selection pipeline.

The package compiles into two isolated runtime executables that communicate over ROS 2 topics.

---

## System Architecture

The package contains two independent C++ nodes:

1. `video_processor`:
   * Accesses the local video capture device via OpenCV.
   * Grabs camera frames and publishes them to `/camera/image`.
   * Subscribes to `/selected_model` using a Reliable + Transient Local QoS profile.
   * Intent: Dynamically update the internal model state and apply the selected machine learning model to the incoming video frames in real-time.

2. `dashboard`:
   * Scans the `models/` directory for .onnx, .pb, or .xml files.
   * Spawns an OpenCV GUI (`Model Controller`) for dynamic model switching.
   * Publishes the selected model filename to `/selected_model` using a Transient Local QoS profile.
   * Renders the video stream received from the `/camera/image` topic.

---

## Model Management & Custom Models

### 1. Model Downloader
To populate the `models/` directory with standard detection models, use the provided script:

```bash
cd ~/ros2_ws/src/camera_processing
./download_models.sh
```

## Model Configuration

The `video_processor` requires a JSON configuration file for each model to handle preprocessing and output parsing. Each config file must share the same base name as its corresponding `.onnx` model file (e.g., `yolo_2026.onnx` requires `yolo_2026.json`).

### Configuration Format

| Key | Type | Description |
| :--- | :--- | :--- |
| `parser_type` | string | Defines the parsing logic (`yolo_grid` or `yolo_multi_head`). |
| `input_size` | [int, int] | The network input resolution as `[width, height]`. |
| `mean` | [float, float, float] | BGR mean values for normalization. |
| `scale` | float | Scaling factor for input pixels (e.g., `1/255` ≈ `0.00392`). |
| `swapRB` | bool | Whether to swap Red and Blue channels (set `true` for OpenCV). |
| `crop` | bool | Whether to center-crop the image before inference. |
| `layout` | string | Tensor layout (typically `NCHW`). |

#### Parser Types
The `parser_type` determines how the node interprets the model's output tensors:

*   **`yolo_grid`**: Used for older, monolithic grid-based architectures (e.g., YOLOv2). Performs a single-tensor reshape followed by NMS.
*   **`yolo_multi_head`**: Used for modern architectures (e.g., YOLOv8, YOLO11, YOLO2026). Handles outputs across multiple feature map scales, concatenates them, and performs global NMS.

### Example Configuration

```json
{
  "parser_type": "yolo_multi_head",
  "input_size": [640, 640],
  "mean": [0.0, 0.0, 0.0],
  "scale": 0.00392157,
  "swapRB": true,
  "crop": false,
  "layout": "NCHW"
}
```

> Note on Startup Sequence: ROS 2 DDS discovery requires initialization time. If `video_processor` is started before the `dashboard` has established its publisher and history buffer, the initial model selection message **may** be lost. Starting the `dashboard` node first ensures the Transient Local buffer is populated and available for late-joining nodes.

---

## System Specifications & Dependencies

* Operating System: Ubuntu 26.04 LTS
* ROS 2 Distribution: Lyrical
* C++ Compiler: GCC 15.2.0 (C++20)
* Libraries: 
  * OpenCV 4.10.0
  * cv_bridge
  * rclcpp
  * sensor_msgs
  * std_msgs

---

## Installation and Setup

### 1. Structure the Workspace

```text
~/ros2_ws/
└── src/
    └── camera_processing/
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── models/                <-- Place custom model files here
        └── src/
            ├── video_processor.cpp
            └── dashboard.cpp
```

### 2. Compile the Package
```bash
cd ~/ros2_ws
colcon build --packages-select camera_processing
```

### 3. Source the Environment
```bash
source install/setup.bash
```

---

## Running the Nodes

* Terminal 1: Start the Video Processor
```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run camera_processing video_processor
```

* Terminal 2: Launch the Dashboard GUI
```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run camera_processing dashboard
```
