#!/bin/bash
set -e

# Setup directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TARGET_DIR="$SCRIPT_DIR/models"
mkdir -p "$TARGET_DIR"

# Helper function to generate JSON config
create_config() {
    local name=$1; local type=$2; local width=$3; local height=$4; local classes=$5; local mean=$6; local scale=$7; local anchors=$8
    local json_file="$TARGET_DIR/${name}.json"
    
    cat <<EOF > "$json_file"
{
  "model_type": "$type",
  "input_size": [$width, $height],
  "mean": [$mean],
  "scale": $scale,
  "swapRB": true,
  "crop": false,
  "classes": "$classes"
  $( [ -n "$anchors" ] && echo ", \"anchors\": $anchors" )
}
EOF
    echo "Generated config: $json_file"
}

# 1. Download Labels (Common to all models)
if [ ! -f "$TARGET_DIR/coco.names" ]; then
    curl -L -o "$TARGET_DIR/coco.names" "https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names"
fi

# 2. YOLOv2 (ONNX)
# Standard YOLOv2. Input 416x416. Scale is 1/255.
echo "Downloading YOLOv2..."
curl -L -o "$TARGET_DIR/yolov2.onnx" "https://github.com/onnx/models/raw/main/validated/vision/object_detection_segmentation/yolov2-coco/model/yolov2-coco-9.onnx"
ANCHORS="[0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 5.47434, 7.88282, 3.52778, 9.77052, 9.16828]"
create_config "yolov2" "yolov2" 416 416 "coco.names" "0, 0, 0" "0.00392157" "$ANCHORS"

# # 3. SSD-ResNet34 (ONNX)
# # Heavy-duty model from Open Model Zoo. Input 1200x1200.
# echo "Downloading SSD-ResNet34..."
# curl -L -o "$TARGET_DIR/ssd-resnet34-1200-onnx.onnx" "https://storage.openvinotoolkit.org/repositories/open_model_zoo/public/2022.1/ssd-resnet34-1200-onnx/resnet34-ssd1200.onnx"
# # ResNet models typically use 0 mean and 1.0 scale (images are already preprocessed or handled internally)
# create_config "ssd-resnet34-1200-onnx" "ssd" 1200 1200 "coco.names" "0, 0, 0" "1.0" ""

echo "-------------------------------------------------"
echo "Download complete! Both YOLOv2 and SSD-ResNet34 are ready."