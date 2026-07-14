# camera_processing

ROS 2 package featuring a modular video streaming and visualization pipeline.

The package compiles into two isolated runtime executables that communicate over ROS 2 topics.

---

## System Architecture

The package contains two independent C++ nodes:

1. **`video_processor`**:
   * Accesses the default local video capture device (`/dev/video0`) via OpenCV.
   * Grabs camera frames at ~30 FPS.
   * Converts raw OpenCV frames (`cv::Mat`) to ROS 2 standard `sensor_msgs/msg/Image` payloads using `cv_bridge`.
   * Publishes the converted frames over the `/camera/image` topic.

2. **`dashboard`**:
   * Subscribes to the `/camera/image` topic.
   * Deserializes incoming messages back into OpenCV format using `cv_bridge`.
   * Spawns a dedicated GUI thread to render the real-time video stream using OpenCV's native high-level GUI windowing capabilities.

---

## System Specifications & Dependencies

This codebase is configured, tested on:
* **Operating System:** Ubuntu 26.04 LTS
* **ROS 2 Distribution:** Lyrical
* **C++ Compiler:** GCC 15.2.0 (leveraging C++20 features)
* **Libraries:** * OpenCV `4.10.0` (native development headers)
  * `cv_bridge` (ROS 2 bridging suite)
  * `rclcpp` (ROS 2 C++ client library)
  * `sensor_msgs` (Standard sensor interface specifications)

---

## Installation and Setup

### 1. Structure the Workspace
Ensure your workspace directories match the standard ROS 2 structural layout:

```text
~/ros2_ws/
└── src/
    └── camera_processing/         <-- Git Repository Root
        ├── .gitignore
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        └── src/
            ├── video_processor.cpp
            └── dashboard.cpp
```

### 2. Compile the Package
Navigate to your workspace root directory and build the package utilizing `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select camera_processing
```

### 3. Source the Environment
After compilation succeeds, register your workspace overlay paths in your active terminal:

```bash
source install/setup.bash
```

---

## Running the Nodes

### Local Execution (Single Machine)

Start the publisher and visualizer as two isolated operating system processes:

* **Terminal 1: Start the Camera Pipeline**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run camera_processing video_processor
  ```

* **Terminal 2: Launch the GUI Visualizer**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run camera_processing dashboard
  ```
