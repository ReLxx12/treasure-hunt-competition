# 夺宝奇兵比赛代码与训练资料

本仓库整理了夺宝奇兵自动赛、半自动赛代码，以及 2026 YOLO 目标检测模型的训练与部署资料。各年份代码分开放置；同名 ROS 包不要同时放进一个 catkin 工作空间。

**实机情况：维护者已确认原机器人工作区中的程序在机器人上运行过。** 仓库保存实际机器人程序及其使用的 YOLO 权重。本次重新整理后做了源码完整性和文档检查，没有连接机器人重新部署；历史代码与未启用功能不据此标为全部实测通过。

想了解程序如何工作，先读 [核心代码与运行逻辑](docs/core-logic.md)，其中包括状态机图、分类板 B/C/D 映射、抓取坐标换算、导航、堆叠放置与半自动赛控制。

## 目录

- robot/auto_2026/：当前完整自动赛代码，来自实际机器人工作区，含运行时使用的 treasure6_yolo11n.pt。
- [robot/platform/](robot/platform/README.md)：从机器人工作区补齐的 19 个底层 ROS 包，包含底盘、机械臂、相机、雷达、导航配套、公共消息和串口支持，以及原厂模型资源和许可声明。
- robot/auto_2024/：2024 自动赛代码。
- robot/semi_auto_2024/：2024 半自动赛代码，包含键盘遥控和抓取节点。
- robot/references/auto_2026/：2026-07-01 修改的 runtask_2026_reference.py 与 task_2026_reference.yaml。它们是参考文件，旧版启动文件没有调用它们。
- training/：数据准备、采集、标注、训练和导出流程。
- docs/：版本划分、程序结构、运行说明及授权/资产说明。

## 版本说明

当前可确认的完整 2026 自动赛程序位于 robot/auto_2026/。其源目录名称是 auto_match_25，核心程序在 2026 年 7 月修改。旧自动赛和半自动赛压缩包虽然在 2026-07-01 修改，压缩包内核心源码时间戳仍为 2024 年；因此分别按 2024 代码归档。详细说明见 docs/versions-and-modes.md。

## YOLO 模型

自动赛启动文件默认加载：

    robot/auto_2026/auto_match/models/treasure6_yolo11n.pt

模型对应六类：cake、banana、apple、clock、broccoli、teddybear。训练步骤和部署参数见 training/README.md 与 docs/install-and-run.md。

## 重要运行条件

- 程序面向 Spark ROS Noetic 机器人环境，依赖底盘、相机、雷达、导航、定位和机械臂等 ROS 包；普通电脑不能独立完成比赛运行。
- auto_2024 和 auto_2026 中都含有 ROS 包 auto_match，每次只把一个版本放入运行用的 catkin src。
- 场地地图和 mark_map.pkl 属于机器人/赛场数据，本仓库不包含。首次运行前按 docs/install-and-run.md 准备地图与标记点。
- 2026 配置中 debug_disable_time_limit: true 是调试豁免。正式比赛前需按规则确认并关闭。
- 厂商底层源码保留上游和各包的许可原文；比赛改动与模型不声明统一许可证，见 [来源与资产说明](docs/licensing-and-assets.md)。

## 快速导航

- [核心代码与运行逻辑](docs/core-logic.md)
- [机器人底层源码清单](robot/platform/README.md)
- [版本与比赛模式](docs/versions-and-modes.md)
- [程序结构与比赛流程](docs/competition-flow.md)
- [安装和运行](docs/install-and-run.md)
- [YOLO 训练流程](training/README.md)
- [源码、模型和地图说明](docs/licensing-and-assets.md)
