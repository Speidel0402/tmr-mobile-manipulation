# TMR Mobile Manipulation

TMR 全向底盘、双雷达、双 Franka FR3、Robotiq 夹爪、腕部 D405 与顶部 ZED 的比赛任务代码。

## 最终实机版本

2026-09-02 已从场地标记起点完整验证以下连续流程（运行编号 `36a78660aa63`）：

1. 初始化双夹爪、双臂标准姿态和 `0.70 m` 升降柱高度；
2. 前进 `0.85 m`、顺时针转 `90°`，根据本轮双雷达数据识别门框并对准中线；
3. 到门前 `0.50 m`，沿门中线前进 `1.20 m` 到物品桌；
4. 抓杯子并放到 `B`；
5. 返回物品桌，抓食物盆并放到 `A`；
6. 返回物品桌，抓盘子并放到 `D`；
7. 停在 `D`，恢复手柄控制。

三种物体使用浅绿色外观和实物直径联合区分：杯子 `76 mm`、食物盆 `120 mm`、盘子 `185 mm`。抓取顺序固定为 `张开 → 视觉识别与校准 → 下降 → 闭合 → 抬升`，不会在下降前闭合或在夹取后再次松开。

最终参数：

| 物体 | 默认区域 | 抓取/放置下降 | 放置前伸 |
|---|---:|---:|---:|
| 杯子 | B | `0.340 m` | `0.100 m`，受末端 `x ≤ 1.0 m` 限制 |
| 食物盆 | A | `0.360 m` | `0.070 m`，受末端 `x ≤ 1.0 m` 限制 |
| 盘子 | D | `0.375 m` | `0.100 m`，受末端 `x ≤ 1.0 m` 限制 |

杯子和食物盆识别到目标字母后继续向右 `0.08 m` 再放置。盘子面对 `D` 时使用专用模式：真实白色卡片上的 `D` 连续两帧确认后立即停止横移并放置，不再附加 `0.08 m`。

## 一键运行

比赛时在机械臂计算机 `aup@172.16.0.100` 本地运行；实时底盘控制会通过 SSH 在底盘计算机 `tmr-user@172.16.0.50` 本地执行。Windows/Codex 只负责部署、启动和观察，不承载实时控制环。

```bash
cd /home/aup/tmr-mobile-manipulation
bash mission/scripts/run_complete_from_start.sh \
  --cup-letter B --bowl-letter A --plate-letter D
```

该入口强制选择“从标记起点完整运行”，并拒绝混入中途恢复参数。直接调用等价入口为：

```bash
python3 mission/scripts/run_three_object_delivery.py \
  --execute --fresh-start-confirmed \
  --cup-letter B --bowl-letter A --plate-letter D
```

运行前只需满足以下条件：

- 机器人确实放在已标记起点，场地和物品摆放与标定一致；
- 双 FR3、双 Robotiq、升降柱、左右腕部 RGB 和顶部 ZED 服务已启动；
- `.100` 可免密 SSH 到 `.50`；
- 不要同时启动第二个任务脚本或另一个速度发布器。

任务入口会自动关闭手柄速度控制、取得任务租约、检查底盘里程计并在需要时重启底盘运行栈；正常完成、异常退出或 `Ctrl+C` 后都会尝试恢复手柄控制。相机应在整场比赛中常驻，不要在视觉校准中途重启。

## 完整动作逻辑

每件物体都先由左腕相机在标准高度重新识别，再以小步自适应视觉伺服对准；算法只使用新鲜 RGB 帧，不依赖深度图。夹持后底盘执行：后退 `1.70 m` → 逆时针 `180°` → 再后退 `0.25 m` → 向右搜索字母（最多 `2.40 m`）。

杯子和食物盆放置后，底盘按本轮实际右移距离向左返回并额外左移 `0.20 m`，再顺时针旋转 `180°`，重新识别门框，执行“门中线 → 门前 `0.50 m` → 前进 `1.20 m`”返回物品桌。盘子是最后一件，放到 D 后不再返回。

字母只在亮白、近矩形卡片内部识别，并按孔洞拓扑区分 `A/B/D`；场内分类字母表包含 `ABCDE`，避免把可见的 `E` 强制误认成 `A`。盘子、餐具和灰色地面不满足白卡约束，不能授权放置。

## 常见问题与恢复

### 1. 右臂硬件激活失败

典型信息为 `right runtime controller recovery failed` 或 `right hardware activation failed`。这种情况发生在运动开始前；不要反复发送关节目标。重启双臂控制进程：

```bash
screen -S tmr_fr3_arms -X quit
screen -L -Logfile /tmp/tmr_fr3_arms.log -dmS tmr_fr3_arms bash -lc \
  'source /home/aup/tmr_env.sh; exec ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:=tmr_duo_config.yaml'
```

确认两侧 `joint_impedance_controller` 均为 `active`：

```bash
source /home/aup/tmr_env.sh
ros2 control list_controllers -c /right/controller_manager
ros2 control list_controllers -c /left/controller_manager
```

随后把机器人放回标记起点，再运行一键入口。

### 2. 底盘报告 `no odometry progress`、`No configuration` 或不响应

任务会立即锁存零速并退出，不会无限发送速度。底盘运行栈由以下脚本恢复；它会验证当前里程计、控制器和唯一速度适配器，失效时只重启本项目管理的运行栈：

```bash
ssh tmr-user@172.16.0.50 '~/tmr_cycle/scripts/19_ensure_navigation_stack.sh'
```

恢复后将机器人重新放到标记起点，再运行完整入口。不要并行手工启动第二个 `tmrv0_2.launch.py` 或 `cmd_vel_adapter.py`。

### 3. 相机网页在线但识别使用旧画面

左腕抓取会检查图像序号、时间戳和相机会话编号；顶部字母搜索也只接受不断更新的新 JPEG。检查左腕服务：

```bash
curl -fsS http://127.0.0.1:18080/healthz
```

若序号不增长，先终止当前任务，再恢复 D405/快照服务；不要在机械臂已经开始视觉伺服后切换相机进程。深度图掉线不影响本版本抓取。

### 4. 中途恢复但不重复已完成物体

只有人工确认机器人已经回到物品桌标准位置时才能使用恢复入口：

```bash
# 已在物品桌，重新从杯子开始（跳过起点到底盘路线）
python3 mission/scripts/run_three_object_delivery.py --execute \
  --resume-at-pickup-confirmed --cup-letter B --bowl-letter A --plate-letter D

# 杯子已完成，从食物盆开始
python3 mission/scripts/run_three_object_delivery.py --execute \
  --resume-object-at-pickup-confirmed bowl \
  --cup-letter B --bowl-letter A --plate-letter D

# 杯子和食物盆已完成，从盘子开始
python3 mission/scripts/run_three_object_delivery.py --execute \
  --resume-object-at-pickup-confirmed plate \
  --cup-letter B --bowl-letter A --plate-letter D
```

如果杯子已经确认夹住并位于标准抬升高度，可使用 `--resume-after-cup-held-confirmed`；否则不要使用该参数。任何不确定状态都应先人工复位到标记起点，再使用一键完整入口。

### 5. 手柄没有恢复

任务最后会自动恢复；若远程连接同时断开，可在底盘计算机执行：

```bash
~/tmr_cycle/scripts/17_control_mode.sh teleop
```

## 目录与验证

- [`base/`](base/)：底盘启动、门框识别、里程计闭环、控制租约和字母搜索。
- [`grasp/`](grasp/)：三类物体检测、视觉伺服、双臂/夹爪/升降柱及放置流程。
- [`mission/`](mission/)：完整任务状态机、参数和恢复入口。
- [`tools/`](tools/)：三相机网页、快照和跨 ROS 域图像桥接工具。
- [`docs/FAILURE_FIXES.md`](docs/FAILURE_FIXES.md)：历史故障与设计原因。

不连接机器人时可运行：

```bash
python -m pip install pytest numpy pyyaml scipy opencv-python-headless pillow
python -m pip install -e grasp
python -m pytest base/tests grasp/tests grasp/scripts/test_pick_cycle_policy.py mission/tests
python -m py_compile base/scripts/*.py grasp/scripts/*.py mission/scripts/*.py tools/*.py
```

运行日志和检查点默认保存在机械臂计算机的 `~/.tmr_three_object_delivery/`。仓库不提交密码、SSH 私钥、现场调试图像、ROS bag 或运行日志。
