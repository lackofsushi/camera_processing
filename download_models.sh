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
    local classes=$7
    local anchors=$8
    
    local json_file="$TARGET_DIR/${filename%.onnx}.json"
    
    cat <<EOF > "$json_file"
{
  "input_size": [$width, $height],
  "mean": [$mean_val, $mean_val, $mean_val],
  "scale": $scale,
  "swapRB": $swap,
  "crop": false,
  "classes": "$classes",
  "anchors": $anchors
}
EOF
    echo "Generated config: $json_file"
}

# 1. Download labels
if [ ! -f "$TARGET_DIR/coco.names" ]; then
    curl -L -o "$TARGET_DIR/coco.names" "https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names"
fi

echo "Downloading Single-Head YOLOv2 models..."

# YOLOv2 (Standard)
if [ ! -f "$TARGET_DIR/yolov2.onnx" ]; then
    curl -L -o "$TARGET_DIR/yolov2.onnx" "https://github.com/onnx/models/raw/main/validated/vision/object_detection_segmentation/yolov2-coco/model/yolov2-coco-9.onnx"
    # COCO Anchors
    ANCHORS="[0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 5.47434, 7.88282, 3.52778, 9.77052, 9.16828]"
    create_config "yolov2" 416 416 0.0 0.00392157 true "coco.names" "$ANCHORS"
fi

# YOLOv2-Tiny
if [ ! -f "$TARGET_DIR/yolov2-tiny.onnx" ]; then
    # Standard Tiny YOLOv2 COCO anchors
    # These are specific to Tiny's configuration
    curl -L -o "$TARGET_DIR/yolov2-tiny.onnx" "https://github.com/onnx/models/raw/main/validated/vision/object_detection_segmentation/tiny-yolov2/model/tiny-yolov2-7.onnx"
    
    # Tiny YOLOv2 Anchors
    ANCHORS="[1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52]"
    create_config "yolov2-tiny" 416 416 0.0 0.00392157 true "coco.names" "$ANCHORS"
fi

echo "Setup complete!"
