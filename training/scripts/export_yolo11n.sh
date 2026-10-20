#!/usr/bin/env bash
set -euo pipefail

MODEL="${1:-runs/detect/treasure_yolo11n/weights/best.pt}"
IMGSZ="${IMGSZ:-512}"

yolo export model="${MODEL}" format=onnx imgsz="${IMGSZ}" simplify=True
yolo export model="${MODEL}" format=openvino imgsz="${IMGSZ}"

