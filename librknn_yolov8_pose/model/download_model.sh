#!/bin/bash
set -e

if [ ! -f yolov8n-pose.onnx ]; then
    wget -O yolov8n-pose.onnx https://ftrg.zbox.filez.com/v2/delivery/data/95f00b0fc900458ba134f8b180b3f7a1/examples/yolov8_pose/yolov8n-pose.onnx
fi
# Convert the model for rk3588
# python convert.py yolov8n-pose.onnx rk3588
