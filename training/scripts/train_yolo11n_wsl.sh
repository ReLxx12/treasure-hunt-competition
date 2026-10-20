#!/usr/bin/env bash
set -euo pipefail

DATA_YAML="${1:-training/configs/dataset_template/data.yaml}"
EPOCHS="${EPOCHS:-100}"
IMGSZ="${IMGSZ:-512}"
BATCH="${BATCH:-16}"
DEVICE="${DEVICE:-0}"
RUN_NAME="${RUN_NAME:-treasure_yolo11n}"

yolo detect train \
  model=yolo11n.pt \
  data="${DATA_YAML}" \
  epochs="${EPOCHS}" \
  imgsz="${IMGSZ}" \
  batch="${BATCH}" \
  device="${DEVICE}" \
  name="${RUN_NAME}"


