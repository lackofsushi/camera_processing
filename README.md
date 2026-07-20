# camera_processing

ROS 2 package featuring a modular video streaming and processing pipeline with real-time visualization, and dynamic model-selection for the processing (object detection). It also contains minimal performance data for inferences done in the video processing pipeline.

The package compiles into two isolated runtime executables that communicate over ROS 2 topics.

---

## System Architecture

The package contains two independent C++ nodes:

1. `video_processor`:
   * Accesses the local video capture device via OpenCV.
   * Grabs camera frames and publishes them to `/camera/image`.
   * Subscribes to `/selected_model` using a Reliable + Transient Local QoS profile.
   * Dynamically update the internal model state and apply the selected machine learning model to the incoming video frames in real-time.

2. `dashboard`:
   * Scans the `models/` directory for .onnx, .pb, or .xml files.
   * Spawns an OpenCV GUI (`Model Controller`) for dynamic model switching.
   * Publishes the selected model filename to `/selected_model` using a Transient Local QoS profile.
   * Renders the video stream received from the `/camera/image` topic.
   * Displays min, max, avg time for every object detection inference.

---

## Model Management & Custom Models

### 1. Model Downloader
To populate the `models/` directory with standard detection models, use the provided script:

```bash
cd ~/ros2_ws/src/camera_processing
./download_models.sh
```

## Model Configuration

The `video_processor` requires a JSON configuration file for each model to handle preprocessing and output parsing. Each config file must share the same base name as its corresponding `.onnx` model file (e.g., `yolo_v2.onnx` requires `yolo_v2.json`).

### Configuration Format

| Key | Type | Description |
| :--- | :--- | :--- |
| `model_type` | string | Defines the parsing logic. Note: The parser for the model type needs to be implemented in the video processor node |
| `input_size` | [int, int] | The network input resolution as `[width, height]`. |
| `mean` | [float, float, float] | BGR mean values for normalization. |
| `scale` | float | Scaling factor for input pixels (e.g., `1/255` ≈ `0.00392`). |
| `swapRB` | bool | Whether to swap Red and Blue channels (set `true` for OpenCV). |
| `crop` | bool | Whether to center-crop the image before inference. |
| `layout` | string | Tensor layout (typically `NCHW`). |

#### Parser Types
The `model_type` determines how the node interprets the model's output tensors. Currently only one type is implemented:

*   **`yolo_v2`**: Used for older, monolithic grid-based architectures (e.g., YOLOv2). Performs a single-tensor reshape followed by NMS.

### Example Configuration

```json
{
  "model_type": "yolov2",
  "input_size": [416, 416],
  "mean": [0, 0, 0],
  "scale": 0.00392157,
  "swapRB": true,
  "crop": false,
  "classes": "coco.names",
  "anchors": [0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 5.47434, 7.88282, 3.52778, 9.77052, 9.16828]
}
```

> Note: Anchors is optional since not all models use it.

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
### 4. Future work
* Flag to unload old model when new model is selected.
* Throughput and latency execution mode for video processing
* Support for other models (needs OpenCV 5.X or other frameworks) 
