# YOLO11n 训练流程

本流程整理自机器人使用的 YOLO11n 训练资料。仓库包含训练/导出脚本、六类配置、相机采集脚本和采集清单，不包含原始图片、标签、基础预训练权重或训练运行缓存。

## 类别顺序

训练标签、数据 YAML 和部署模型必须使用同一顺序：

    0 cake
    1 banana
    2 apple
    3 clock
    4 broccoli
    5 teddybear

类别表见 configs/dataset_template/classes.txt，示例配置见 configs/dataset_template/data.yaml。旧指南中的四类/七类说明已过时，不要用于当前六类模型。

## 1. 准备数据

在独立数据目录建立以下结构：

    dataset/
    ├─ data.yaml
    ├─ images/
    │  ├─ train/
    │  └─ val/
    └─ labels/
       ├─ train/
       └─ val/

每张图片对应同名 YOLO 标签文本。每行格式为 class_id x_center y_center width height，坐标归一化到 0–1。训练集和验证集应按采集场景或视频段分开，避免相邻帧同时进入两组造成验证结果偏高。

用机器人相机的实际视角采集数据；标注可抓取物体时框住物体可见外轮廓，确保遮挡、漏标和类别编号正确。scripts/capture_ros_images.py 可辅助从 ROS 图像话题采集图片，采集清单在 docs/capture_checklist.csv。

编辑 data.yaml，让 train 和 val 指向数据集内图片目录，并设置 nc: 6 及上述 names 顺序。目录样例来自当前六类配置；实际训练路径不要使用其他电脑上的固定路径。

## 2. 安装训练环境

在支持 GPU 的 Linux/WSL 环境安装匹配的 Python、PyTorch 与 Ultralytics。训练入口使用 yolo 命令。训练软件版本、GPU 型号和驱动版本应记录在每次训练记录中；仓库没有锁定这些版本。

## 3. 开始训练

    bash training/scripts/train_yolo11n_wsl.sh /path/to/dataset/data.yaml

脚本默认使用 yolo11n.pt、100 epochs、512 输入尺寸、batch 16 和 GPU 0。可通过环境变量调整：

    EPOCHS=100 IMGSZ=512 BATCH=16 DEVICE=0 RUN_NAME=treasure_yolo11n bash training/scripts/train_yolo11n_wsl.sh /path/to/dataset/data.yaml

Ultralytics 通常把训练结果写到 runs/detect/<RUN_NAME>/。实际输出目录以训练日志为准。

## 4. 验证并选择权重

使用验证集查看 precision、recall、mAP、混淆矩阵和漏检/误检样例；还要在机器人摄像头画面上检查六类识别、遮挡和比赛距离下的表现。保存训练配置、数据版本、日志和验证结果。当前仓库中的部署权重是实机程序正在引用的 treasure6_yolo11n.pt；本仓库未提供可核验的训练日志，因此不对其精度指标作承诺。

## 5. 导出与部署

导出脚本示例：

    bash training/scripts/export_yolo11n.sh /path/to/best.pt

脚本导出 ONNX 和 OpenVINO 格式。机器人 launch 默认使用 PyTorch .pt 权重；除非目标硬件的推理节点明确支持其他格式，不要仅凭文件导出成功就替换部署模型。

经机器人端验证后，将权重放入 auto_match/models/，并通过 yolo_model_path 指向它。核对类别顺序、置信度阈值、输入尺寸、设备参数、相机话题和实际帧率。

## 6. 训练记录建议

每次训练记录：日期、数据版本/来源、每类图片和实例数、训练/验证划分、类别顺序、基础模型、Ultralytics/PyTorch 版本、epochs、imgsz、batch、GPU、验证指标、实机结果、权重 SHA-256。未经确认，不公开含个人信息或第三方授权不明的图片。
