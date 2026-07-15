#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TARGET_DIR="$SCRIPT_DIR/models"

echo "Creating models directory at: $TARGET_DIR"
mkdir -p "$TARGET_DIR"

echo "Downloading models..."

# 1. YOLOv2 (Classic, lightweight)
if [ ! -f "$TARGET_DIR/yolov2.onnx" ]; then
    echo "Downloading YOLOv2..."
    curl -L -o "$TARGET_DIR/yolov2.onnx" \
        "https://github.com/onnx/models/raw/main/validated/vision/object_detection_segmentation/yolov2-coco/model/yolov2-coco-9.onnx"
else
    echo "YOLOv2 already exists, skipping."
fi

# 2. YOLO 2026 (Using the ultra-modern YOLOv11 nano model as our 2026 representation!)
if [ ! -f "$TARGET_DIR/yolo_2026.onnx" ]; then
    echo "Downloading YOLO 2026..."
    curl -L -o "$TARGET_DIR/yolo_2026.onnx" \
        "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx"
else
    echo "YOLO 2026 already exists, skipping."
fi

# 3. ResNet-50 (Standard image classification)
if [ ! -f "$TARGET_DIR/resnet50.onnx" ]; then
    echo "Downloading ResNet-50..."
    curl -L -o "$TARGET_DIR/resnet50.onnx" \
        "https://huggingface.co/Qdrant/resnet50-onnx/resolve/main/model.onnx"
else
    echo "ResNet-50 already exists, skipping."
fi

echo "All models downloaded successfully inside $TARGET_DIR!"
