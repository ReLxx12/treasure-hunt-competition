# 安装与运行

## 环境要求

比赛程序面向 Spark ROS Noetic 机器人工作区，需安装对应硬件驱动和 ROS 包，包括相机、雷达、底盘、导航/AMCL、机械臂及 Spark 项目依赖。具体系统镜像和硬件版本应以机器人现场环境为准。

维护者已确认原工作区的程序在机器人上运行过。本仓库补入了 19 个底层 ROS 包；本次整理没有在机器人上重跑。已有可运行机器人环境时，应先复用原环境与场地标定，再迁移整理后的代码。

YOLO 检测节点还需要与机器人 Python 环境兼容的 PyTorch、Ultralytics、OpenCV、cv_bridge 和 NumPy。底层相机与雷达驱动分别需要 librealsense2、YDLidar-SDK，ROS 侧还需 move_base、amcl、map_server、图像传输、TF 和 actionlib 等依赖。实际安装版本应在机器人上记录后补充；当前快照未锁定完整系统环境。仓库内的 RealSense ROS 驱动 package.xml 版本为 2.2.16，迁移时要与原机 SDK 配套。

## 选择代码版本

将 robot/platform/src 下的底层包和一个自动赛版本的配套包复制或链接到 catkin 工作区 src：

- 2026：robot/auto_2026/auto_match、spark_move、spark_alignment_mark
- 2024：robot/auto_2024/auto_match、common、spark_move、spark_alignment_mark

选择 2024 自动赛时，使用其自带 common，不再加入 platform/src/common。已有 Spark 工作区中也可能已存在底盘、相机等同名包；同一工作空间只保留一份。

半自动赛使用 robot/semi_auto_2024/semi_auto_match_24。还需配置对应 Spark 驱动和底盘依赖。

在新的 Linux/ROS Noetic 工作区中，2026 自动赛的源码装配示例为（修改 repo_dir 为克隆位置）：

```bash
source /opt/ros/noetic/setup.bash
repo_dir=/absolute/path/to/treasure-hunt-competition
ws_dir="$HOME/treasure_hunt_ws"
mkdir -p "$ws_dir/src"
for pkg in "$repo_dir"/robot/platform/src/* "$repo_dir"/robot/auto_2026/*; do
    ln -s "$pkg" "$ws_dir/src/$(basename "$pkg")"
done
cd "$ws_dir"
catkin_make
source devel/setup.bash
```

这是空工作区的装配示例；不是本次已经执行成功的编译记录。系统 ROS 包与两个厂商 SDK 需要预先就绪。保留的可选硬件/建图 launch 还可能依赖其他软件，默认 D435/G6 配置与替代配置应分别检查。不要把整个 robot/ 直接链接进 src，那里包含重复包名的历史版本。

## 串口与机械臂标定

- 底盘 launch 默认 /dev/sparkBase；机械臂驱动源码固定 /dev/ttyACM0；G6 launch 默认 /dev/ydlidar。设备规则原件在 robot/platform/udev/，此次整理没有安装这些规则。
- 当前 runtask.py 和半自动赛 grasp.py 读取 $HOME/thefile.txt。需要从实际机器人迁移对应标定系数，文件是按空白分隔的四个数：kx bx ky by。抓取映射为 x=kx*v+bx、y=ky*u+by，再叠加类别偏置。
- 仓库保留的 cali_node.py 写往 spark_hm_task_match/config/cali_config.yaml；这与上述文本接口不同，且 spark_hm_task_match 业务包不属于本仓库。不能直接将该 YAML 当作运行标定文件，详情见核心逻辑文档。

## 地图和标记点

本仓库不包含比赛场地图和 mark_map.pkl。根据现场地图生成地图 YAML/图像，并在机器人端按程序要求记录或生成标记点文件。auto_match/config/test_map.yaml.example 是配置格式示例；准备好地图后复制为 test_map.yaml，将图像路径指向实际地图，并在启动时传入地图参数，例如：

    roslaunch auto_match auto_match.launch map_file:=/absolute/path/to/map.yaml

运行前需检查地图坐标、AMCL 初始位姿、导航点和标记点。不要把场地专用文件当成通用默认数据。

## 启动 2026 自动赛

在已配置 Spark ROS 依赖和场地图的机器人上：

    roslaunch auto_match auto_match.launch detector_type:=yolo

常用启动参数可在 launch 文件中检查，包括 yolo_model_path、yolo_conf、yolo_imgsz、yolo_device、camera_type_tel 和 map_file。赛前确认实际相机话题、模型路径、模型类别顺序和机械臂/导航参数。

默认 YOLO 参数为 conf=0.45、imgsz=640、device=cpu。启动后 auto_initial_pose 节点可能执行旋转预热和前进动作，需为其留出运动空间。待定位初始化完成、导航点和标定文件就绪后发送任务指令：

```bash
# 开始自动赛
rostopic pub -1 /task_start_flag std_msgs/String "data: '1'"
# 请求停止
rostopic pub -1 /task_start_flag std_msgs/String "data: '0'"
```

当前主状态机没有强制等待 /auto_initial_pose_done，因此由操作者确认初始化完成后再开始任务。

2026 配置中的 debug_disable_time_limit: true 会禁用比赛计时限制。正式比赛前应根据比赛规则关闭该调试设置并验证计时行为。

## 启动 2024 自动赛或半自动赛

使用各自目录内的 launch 文件，并阅读对应 package.xml、脚本及配置。2024 自动赛和 2026 自动赛都使用 ROS 包名 auto_match，不能同时存在于同一个 catkin src。

半自动赛入口：roslaunch semi_auto_match_24 semi_auto.launch。w/a/s/d 控制运动，g 抓取、h 放置。半自动赛抓取使用 HSV 蓝色识别，不会自动使用 2026 的 YOLO；/grasp 的 0/1 含义与自动赛手动接口相反，参见 [接口表](core-logic.md#10-关键接口与运行状态)。
