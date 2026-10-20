# 程序结构与比赛流程

逐个函数的流程图、坐标换算、状态转换、底层驱动与半自动赛区别见 [核心代码与运行逻辑](core-logic.md)。机器人底层源码位于 [robot/platform](../robot/platform/README.md)。

## 2026 自动赛

入口是 robot/auto_2026/auto_match/launch/auto_match.launch。该启动文件负责启动 Spark 底盘和相机驱动、YOLO 检测节点、标记点记录工具、比赛流程节点、AMCL/导航以及比赛需要的辅助节点。默认检测器为 YOLO，模型默认位于 auto_match/models/treasure6_yolo11n.pt。

核心流程节点是 auto_match/nodes/runtask.py，负责比赛状态调度以及导航、识别、机械臂抓取等步骤。具体参数位于 auto_match/config/。运行前检查目标类别、导航点、地图与标记点是否适配当前场地。

auto_match/nodes/stack_vision.py 作为独立模块保留，但当前主流程 select_scoring_drop_target 使用固定 main 放置点，没有调用该模块。仅修改 enabled 参数不能据此认定主流程已接入深度校正。

## 2024 自动赛

入口是 robot/auto_2024/auto_match/launch/auto_match.launch。该版本保留独立的 auto_match、common、spark_move 和 spark_alignment_mark 包组。运行逻辑以本目录内实际启动配置为准，不与 2026 代码混用。

## 2024 半自动赛

入口是 robot/semi_auto_2024/semi_auto_match_24/launch/semi_auto.launch。主要节点包括键盘遥控和视觉抓取脚本。操作者负责遥控流程，视觉识别/抓取功能按该版本节点实现工作。

## 识别类别

当前部署 YOLO 模型使用六类：

| ID | 类别 |
|---:|---|
| 0 | cake |
| 1 | banana |
| 2 | apple |
| 3 | clock |
| 4 | broccoli |
| 5 | teddybear |

不要沿用旧训练手册中的四类 toy 或七类方案；类别顺序必须与训练标签、数据 YAML 和模型输出一致。
