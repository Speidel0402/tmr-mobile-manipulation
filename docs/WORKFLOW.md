# 当前任务流程

## 1. 启动

Windows 执行 `grasp/scripts/start_tmr_system.ps1`，按顺序启动底盘、ZED、Spine、双 Robotiq、双 FR3 和双 D405。遥操作仅在显式使用 `-EnableTeleop` 时启动。

比赛时底盘任务和抓取任务应分别部署在 `.50` 与 `.100`，Windows 只负责发起和观察。相机画面服务默认监听 `18080`；`/snapshot.npz` 只强制要求 RGB，深度与相机内参均可缺省。

## 2. 底盘到抓取位

入口：`base/scripts/07_start_to_pickup.py`。

状态顺序：

1. `INITIAL_FORWARD`：从固定起点沿机器人正前方前进 `0.85 m`。
2. `TURN_CW90`：里程计闭环顺时针旋转 `90°`；不会把上一个阶段已经发生的转角重复计算。
3. `ACQUIRE_DOOR`：从本轮 `/map` 提取墙体缺口候选，再由前后雷达多帧确认门框两侧和可穿越性。
4. `ALIGN_TO_MIDPOINT`：保持朝向，横移到门洞中垂线。
5. `CROSS_DOOR`：先停在门平面前 `0.50 m`，再沿冻结的门洞中线前进 `1.20 m`。
6. `FINAL_STOP`：锁存零速，等待抓取阶段。

路线使用本次启动时冻结的 `mission_start` 和本轮识别出的门框，不依赖历史地图原点或示例图片像素。实时双雷达是硬保护，OccupancyGrid 是二级扫掠证据；只有适配器拥有控制器速度发布权。

从确认的门后停止点退回门前 `1.0 m` 使用 `base/scripts/08_reverse_to_predoor.py`。正常入口在门后约 `0.70 m`，因此默认里程计闭环后退 `1.70 m`；脚本通过独占任务速度通道运行，并持续检查双雷达车尾制动区域。2026-09-01 实机结果为后退 `1.6767 m`、名义门前距离 `0.9767 m`、航向误差约 `0.009°`。

## 3. 左臂杯子抓取

入口：`grasp/scripts/run_streamed_live_pick_cycle.py`。

1. 恢复左臂硬件、状态发布器与阻抗保持控制器。
2. 夹爪保持初始张开；任何闭合都只能发生在下降完成之后。
3. 恢复实机成功记录的顶层 7 关节姿态，并以实测关节误差和 FK 验证高度/姿态。
4. 从左 D405 RGB 图像识别杯子右边缘。
5. 锁定末端高度和方向，通过两个小探测动作估计局部 `base XY -> image UV` 雅可比；随后采用缩步、Broyden 更新和滞回确认，使右边缘接近标定像素。
6. 沿地面垂直方向下降到 `z = 0.266413 m`，约为顶层下方 `33 cm`。每个动作有超时，最后用实测终点判断；必要时只做一次有限精度收敛。
7. 到达低位后闭合 Robotiq。`ABORTED + stalled + 有足够闭合行程` 是物体接触证据；空夹完全闭合不算抓取成功。
8. 抬回本轮视觉校准顶层高度，等待 PTP 控制器退出后恢复左臂状态发布与阻抗保持。

当前实机成功样本中，夹爪因杯子阻挡停在约 `0.178–0.196`，动作状态为 aborted/stalled；这是有效物体接触，不应重新张开。

## 4. 关键配置

- 底盘路线与安全参数：`base/config/start_to_pickup.yaml`。
- 建图/桌腿旧循环参数：`base/config/route.yaml`。
- 三主机、相机、夹爪和初始关节：`grasp/config/system_startup.psd1`。
- 左相机、工作面和抓取参数：`grasp/config/left.yaml`。
- 手眼标定：`grasp/config/left_hand_eye_calibrated.yaml` 与 `grasp/calibration/`。

调整场地参数时，优先重新测量固定起点、整机外廓、门宽和桌面高度；不要通过无限放大阈值来掩盖错误坐标或错误相机。
