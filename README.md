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

* Storage: Model files are indexed from the `models/` directory in the package root.
* Custom Models: Users may add their own .onnx, .pb, or .xml files to the `models/` folder. The dashboard detects and lists these automatically after a rebuild.

> Note on Startup Sequence: ROS 2 DDS discovery requires initialization time. If `video_processor` is started before the `dashboard` has established its publisher and history buffer, the initial model selection message may be lost. Starting the `dashboard` node first ensures the Transient Local buffer is populated and available for late-joining nodes.

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

### 2. Compile the Package
cd ~/ros2_ws
colcon build --packages-select camera_processing

### 3. Source the Environment
source install/setup.bash

---

## Running the Nodes

* Terminal 1: Start the Video Processor
  source ~/ros2_ws/install/setup.bash
  ros2 run camera_processing video_processor

* Terminal 2: Launch the Dashboard GUI
  source ~/ros2_ws/install/setup.bash
  ros2 run camera_processing dashboard
