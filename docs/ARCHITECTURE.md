# 系统与环境架构

## 计算节点

| 节点 | 地址 | 主要职责 | 运行约束 |
|---|---|---|---|
| 底盘主机 | `172.16.0.50` | Swerve 底盘、前后雷达、SLAM、门框检测、20 Hz 路线控制 | ROS 2 Humble；自主控制图使用 `ROS_DOMAIN_ID=97` 与 `ROS_LOCALHOST_ONLY=1` |
| 机器人主机 | `172.16.0.100` | Spine、左右 FR3、左右 Robotiq、左右 D405、视觉与抓取 | ROS 2 Jazzy；执行前加载 `~/tmr_env.sh` |
| 遥操作主机 | `172.16.0.101` | GELLO 与脚踏板，可选 | 自主比赛流程默认不启动，避免速度/机械臂命令抢占 |
| Windows 操作端 | 现场电脑 | 部署、启动、停止、查看状态和画面 | 不运行底盘实时控制环 |

机器人内部设备网：Spine 为 `172.16.16.10`，右 FR3 为 `172.16.16.11`，左 FR3 为 `172.16.16.12`。

## 传感器与执行器

- 底盘：四轮 Swerve 控制器，里程计 `/swerve_drive_controller/odom`。
- 雷达：前后 `LaserScan`，当前连续任务使用 `/lidar_front/scan` 与 `/lidar_rear/scan`。
- 地图：本轮在线 SLAM 的 `/map` OccupancyGrid；仅作为门框候选和二级碰撞证据。
- 头部 ZED Mini：序列号 `17064700`；固定运行在底盘本机视觉 Domain 1，由单实例管理器输出原子 JPEG/HTTP，不与 Domain 97 的底盘控制图混流。
- 左腕 D405：序列号 `409122272639`；右腕 D405：`409122274492`。
- 左 Robotiq：`DAANVRU5`；右 Robotiq：`DAANTK6Q`。
- 左 FR3 ROS 命名空间 `/left`；当前抓取使用左臂和左夹爪。

## 控制数据流

```text
实时 /map + 前后 LaserScan + odom
                 │
                 ▼
      07_start_to_pickup.py
                 │ /tmr_cycle/mission_cmd_vel
                 ▼
       cmd_vel_adapter.py  ──租约/零速锁存──► /swerve_drive_controller/cmd_vel

左 D405 RGB ─► camera_mjpeg_viewer.py ─► /snapshot.npz
                                            │
                                            ▼
杯子右边缘检测 ─► 自适应 XY 视觉伺服 ─► MoveIt FK/Cartesian 求解
                                            │
                                            ▼
                    左 FR3 PTP ─► 地面坐标下降 ─► Robotiq 闭合 ─► 抬升
```

头部 ZED 的高带宽 DDS 数据不跨主机。`zed_frame_export.py` 在底盘本机写入原子 JPEG，字母识别和三相机网页通过该文件/HTTP 读取。

## 隔离原则

底盘 Humble 图与机器人 Jazzy 图不应在 domain 0 中无选择混合。当前比赛实现把底盘闭环完全放在 `.50` 本机；机械臂抓取放在 `.100` 本机。跨主机任务交接应使用一个明确、低频、可确认完成的接口，而不是把远程 SSH 或 Codex 会话放进实时闭环。

## 运行依赖

- 底盘：ROS 2 Humble、`franka_bringup` 的 TMR 底盘启动包、SLAM Toolbox、PyYAML、NumPy。
- 机器人：ROS 2 Jazzy、`franka_ros2`/FR3 控制包、MoveIt 2、Robotiq 控制包、RealSense ROS、NumPy、OpenCV、SciPy。
- 可选感知：PyTorch、SAM 2、Transformers；当前杯沿快速检测和抓取不要求在线下载模型。
- Windows：PowerShell 7 或 Windows PowerShell、OpenSSH 客户端；建议使用 SSH key/agent，仓库不保存密码。
