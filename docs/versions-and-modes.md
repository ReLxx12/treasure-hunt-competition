# 版本与比赛模式

| 目录 | 模式/年份 | 当前判断 | 主要入口 |
|---|---|---|---|
| robot/auto_2026/ | 自动赛，2026 | 当前完整自动赛程序；核心运行文件在 2026-07 修改 | auto_match/launch/auto_match.launch |
| robot/auto_2024/ | 自动赛，2024 | 来自 auto_match_24；压缩包内核心源码时间为 2024 | auto_match/launch/auto_match.launch |
| robot/semi_auto_2024/ | 半自动赛，2024 | 当前查到的遥控、抓取和启动核心文件为 2024 | semi_auto_match_24/launch/semi_auto.launch |
| robot/references/auto_2026/ | 自动赛参考资料，2026 | 两个单独的参考文件，不等同于完整可启动版本 | runtask_2026_reference.py、task_2026_reference.yaml |

自动赛与半自动赛是不同运行模式：自动赛由比赛流程节点调度导航、识别和抓取；半自动赛由操作者遥控，视觉抓取节点提供辅助。请以各自的启动文件和节点实现为准。

## 日期判定

- auto_match_24.rar 修改于 2026-07-01 12:57，包内 runtask.py 和 auto_match.launch 的成员时间为 2024-02。
- semi_auto_match_24.rar 修改于 2026-07-01 12:57，包内 teleop.py、grasp.py 和 semi_auto.launch 的成员时间为 2024-02。
- 旧自动赛解压目录另有两个 2026-07-01 文件：runtask_2026_reference.py、task_2026_reference.yaml。旧 auto_match.launch 仍启动 runtask.py，不会自动使用这两个参考文件。
- 当前完整自动赛工作区目录名是 auto_match_25，但其核心代码修改到 2026-07-12；按代码日期归档在 auto_2026。

## catkin 包名冲突

auto_2024/auto_match 和 auto_2026/auto_match 的包名相同。不要将两份自动赛目录同时加入一个 catkin 工作空间的 src。按需选择一个版本，再加入配套的 spark_move、spark_alignment_mark 等包。
