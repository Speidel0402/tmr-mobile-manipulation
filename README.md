# TMR Mobile Manipulation

TMR 全向底盘、双雷达、Franka FR3、Robotiq 夹爪与腕部 D405 相机的比赛任务代码。仓库保存了当前已在实机上验证的两段流程：

1. 底盘从固定起点前进 `0.85 m`，顺时针转 `90°`，使用本轮实时地图与前后雷达识别门框，横移至门洞中线，在门前 `0.50 m` 对齐后向前 `1.20 m`；
2. 左臂恢复已验证的抓取顶层姿态，以杯子右边缘做自适应视觉伺服，按地面坐标下降约 `33 cm`，下降完成后闭合 Robotiq，再抬回顶层高度。

仓库刻意不包含密码、SSH 私钥、模型权重、ROS bag、运行日志和重复的临时输出。

## 目录

- [`base/`](base/)：底盘启动、建图、门框检测、连续路线、速度租约、雷达/地图碰撞保护与离线测试。
- [`grasp/`](grasp/)：杯沿感知、三类物体检测、视觉伺服、MoveIt 求解服务、夹爪策略、抓取流程、相机与双臂启动配置。
- [`mission/`](mission/)：把底盘路线和左臂抓取合成一个带检查点、可只恢复抓取阶段的连续状态机。
- [`tools/camera_mjpeg_viewer.py`](tools/camera_mjpeg_viewer.py)：RGB 实时画面及快照服务；深度图是可选项，不再是抓取前置条件。
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)：三主机、ROS 版本、网络和数据流。
- [`docs/WORKFLOW.md`](docs/WORKFLOW.md)：当前完整任务流程、关键参数和运行方式。
- [`docs/FAILURE_FIXES.md`](docs/FAILURE_FIXES.md)：实机中遇到的主要故障、根因及当前修复。

## 快速检查（不运动机器人）

```bash
python -m pip install pytest numpy pyyaml scipy opencv-python-headless pillow
python -m pip install -e grasp
python -m pytest base/tests grasp/tests grasp/scripts/test_pick_cycle_policy.py mission/tests
python -m py_compile base/scripts/*.py grasp/scripts/*.py mission/scripts/*.py tools/camera_mjpeg_viewer.py
```

底盘正式控制环必须在底盘主机本地运行；抓取流程建议部署到机器人计算机本地运行。Windows/Codex 端用于部署、启动、停止和查看状态，不承担比赛时的实时控制环。

## 实机入口

Windows 上启动整机 ROS 组件：

```powershell
powershell -ExecutionPolicy Bypass -File .\grasp\scripts\start_tmr_system.ps1
```

底盘主机上运行连续起点到抓取位流程：

```bash
python3 ~/tmr_cycle/scripts/07_start_to_pickup.py \
  --config ~/tmr_cycle/config/start_to_pickup.yaml --execute
```

机器人计算机上的左臂抓取入口是：

```bash
python3 grasp/scripts/run_streamed_live_pick_cycle.py
```

两段流程的比赛综合入口（在机械臂主机运行）是：

```bash
python3 mission/scripts/run_long_range_pick.py --execute
```

该入口依赖 ROS 2 环境、左臂/夹爪节点、左 D405、相机快照服务和 `left_ik` 求解服务。完整启动顺序见 [`docs/WORKFLOW.md`](docs/WORKFLOW.md)。

> 实机执行前必须确认现场任务坐标、整机外廓、门宽、桌面高度和急停状态与配置一致。地图图片只用于复盘；任务不会把某一张 PNG 的像素当成固定导航坐标。
