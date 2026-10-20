# 机器人底层核心源码

这里补充机器人工作区中比赛程序使用的底层 ROS 包。原工作区来源为 NXROBO spark_noetic，基准 Git 提交是 917b649dfa24e3da05d78f61960ac15951d7c4d8；复制的是提供的机器人工作区快照，不能将其视为该提交的干净检出。

维护者已确认原机器人程序运行过。当前默认应用配置使用 D435、YDLIDAR G6 和 uArm SwiftPro；本次文件整理没有重新连接机器人。

## 源码清单

| ROS 包 | 用途 |
|---|---|
| spark_bringup | 组合启动底盘、模型和相机 |
| spark_description | Spark 的 URDF、模型网格、TF 相关配置 |
| spark_teleop | 厂商遥控脚本和显示配置 |
| spark_base | 底盘通信、速度下发、里程计、IMU 与 TF |
| cereal_port、serial | 底盘和机械臂串口支持 |
| swiftpro | uArm 驱动、消息定义、URDF 与网格 |
| camera_driver_transfer | 相机选型启动入口 |
| realsense2_camera、realsense2_description | 提供的 RealSense ROS 驱动快照及模型 |
| lidar_driver_transfer、ydlidar_ros_driver | 雷达选择和 G6 ROS 驱动 |
| ydlidar_g2 | 原厂 G6 的 xacro 分支仍引用该包的雷达 URDF，故同时保留该包源码与模型 |
| spark_slam、spark_navigation | 建图、定位/导航配套启动文件和参数 |
| common | move_straight、turn_body 与距离服务的接口及通用代码 |
| vision_msgs | 与现有检测代码 id/score 字段匹配的视觉消息定义 |
| ar_track_alvar、ar_track_alvar_msgs | spark_alignment_mark 依赖的 AR 标记识别及消息 |

common 和 ar_track_alvar 取自原工作区 spark_hm_task_match 下的公共依赖目录，仅收录相应包。其他比赛业务程序没有一起加入。设备规则原件放在 udev/，未在本次整理中安装或执行。

完整原位置、目标位置、大小及 SHA-256 在 [source-manifest.csv](source-manifest.csv)；包信息在 [packages.json](packages.json)。复制保留了源码字节内容；仓库另设置 Linux 脚本的可执行权限。生成目录、Git/IDE 元数据、缓存、地图和本机 pkl 数据未纳入。

## 配合比赛应用使用

在 catkin 的 src 中加入这里的 src 下各包，再加入 robot/auto_2026 下的包组。不要同时加入多个自动赛版本。

2024 自动赛目录自带 common，选择该历史版本时不要再加入这里的 common。原机器人已有同名包时也应选择一份，避免重复 ROS 包名。

这里只收录核心源码与资源。ROS Noetic 的导航、AMCL、图像转换、TF 等系统包，librealsense2、YDLidar-SDK、PyTorch/Ultralytics 仍需与原机器人环境匹配。camera_driver_transfer 保留原厂可选启动文件；切换到 Astra、其他雷达、其他机械臂或可选 SLAM 后，可能还要安装这些模式专用的包。不能仅凭此快照宣称所有硬件组合均可编译运行。

详细部署步骤见 [安装与运行](../../docs/install-and-run.md)，控制实现见 [核心逻辑](../../docs/core-logic.md)。

## 来源与许可

上游仓库：[nxrobo/spark_noetic](https://github.com/nxrobo/spark_noetic)。其顶层 BSD-3-Clause 许可原文保留在 licenses/NXROBO-BSD-3-Clause.txt。各包自带的 LICENSE、版权声明及 package.xml 许可字段保留原样，各自适用于相应来源。这个来源记录不为本仓库其他比赛改动或 YOLO 权重赋予统一许可证。
