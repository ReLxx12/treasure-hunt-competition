# 安装与运行

## 环境要求

比赛程序面向 Spark ROS Noetic 机器人工作区，需安装对应硬件驱动和 ROS 包，包括相机、雷达、底盘、导航/AMCL、机械臂及 Spark 项目依赖。具体系统镜像和硬件版本应以机器人现场环境为准。

YOLO 检测节点还需要与机器人 Python 环境兼容的 PyTorch、Ultralytics、OpenCV、cv_bridge 和 NumPy。实际安装版本应在机器人上记录后补充；当前代码包未锁定完整依赖版本。

## 选择代码版本

将一个自动赛版本和所需配套包复制或链接到 catkin 工作区 src：

- 2026：robot/auto_2026/auto_match、spark_move、spark_alignment_mark
- 2024：robot/auto_2024/auto_match、common、spark_move、spark_alignment_mark

半自动赛使用 robot/semi_auto_2024/semi_auto_match_24。还需配置对应 Spark 驱动和底盘依赖。

在 ROS Noetic 环境中编译工作区后 source devel/setup.bash。本仓库未包含 Spark 工作空间其余厂商驱动，因此单独克隆本仓库不能组成完整机器人镜像。

## 地图和标记点

本仓库不包含比赛场地图和 mark_map.pkl。根据现场地图生成地图 YAML/图像，并在机器人端按程序要求记录或生成标记点文件。auto_match/config/test_map.yaml.example 是配置格式示例；准备好地图后复制为 test_map.yaml，将图像路径指向实际地图，并在启动时传入地图参数，例如：

    roslaunch auto_match auto_match.launch map_file:=/absolute/path/to/map.yaml

运行前需检查地图坐标、AMCL 初始位姿、导航点和标记点。不要把场地专用文件当成通用默认数据。

## 启动 2026 自动赛

在已配置 Spark ROS 依赖和场地图的机器人上：

    roslaunch auto_match auto_match.launch detector_type:=yolo

常用启动参数可在 launch 文件中检查，包括 yolo_model_path、yolo_conf、yolo_imgsz、yolo_device、camera_type_tel 和 map_file。赛前确认实际相机话题、模型路径、模型类别顺序和机械臂/导航参数。

2026 配置中的 debug_disable_time_limit: true 会禁用比赛计时限制。正式比赛前应根据比赛规则关闭该调试设置并验证计时行为。

## 启动 2024 自动赛或半自动赛

使用各自目录内的 launch 文件，并阅读对应 package.xml、脚本及配置。2024 自动赛和 2026 自动赛都使用 ROS 包名 auto_match，不能同时存在于同一个 catkin src。
