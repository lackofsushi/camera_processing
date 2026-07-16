#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TARGET_DIR="$SCRIPT_DIR/models"

mkdir -p "$TARGET_DIR"
touch "$TARGET_DIR/.gitkeep"

# Helper function to generate modular JSON config
create_config() {
    local filename=$1
    local width=$2
    local height=$3
    local mean_val=$4
    local scale=$5
    local swap=$6
    local parser=$7
    
    local json_file="$TARGET_DIR/${filename%.onnx}.json"
    
    cat <<EOF > "$json_file"
{
  "parser_type": "$parser",
  "input_size": [$width, $height],
  "mean": [$mean_val, $mean_val, $mean_val],
  "scale": $scale,
  "swapRB": $swap,
  "crop": false,
  "layout": "NCHW"
}
EOF
    echo "Generated config: $json_file"
}

echo "Downloading object detection models and generating configs..."

# 1. YOLOv2 (Grid-based architecture)
if [ ! -f "$TARGET_DIR/yolov2.onnx" ]; then
    curl -L -o "$TARGET_DIR/yolov2.onnx" "https://github.com/onnx/models/raw/main/validated/vision/object_detection_segmentation/yolov2-coco/model/yolov2-coco-9.onnx"
    create_config "yolov2" 416 416 0.0 0.00392157 true "yolo_grid"
fi

# 2. YOLO 2026 (YOLO11n - Multi-head architecture)
if [ ! -f "$TARGET_DIR/yolo_2026.onnx" ]; then
    curl -L -o "$TARGET_DIR/yolo_2026.onnx" "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx"
    create_config "yolo_2026" 640 640 0.0 0.00392157 true "yolo_multi_head"
fi

# Note: ResNet-50 excluded as it is a classification model, not detection.

echo "Setup complete!"