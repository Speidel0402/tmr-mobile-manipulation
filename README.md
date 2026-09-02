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

### 什么时候可以直接运行

如果机器人已经在标记起点，并且双臂、双夹爪、升降柱、D405 和 ZED 的底层服务保持运行，那么下面这一条就是最终比赛入口，可以直接执行。2026-09-02 的完整成功运行使用的就是这条路径：

```bash
cd /home/aup/tmr-mobile-manipulation
bash mission/scripts/run_complete_from_start.sh \
  --cup-letter B --bowl-letter A --plate-letter D
```

这不是“机器人刚通电后的全套驱动启动命令”。如果刚开机、FCI 刚重启或底层服务全部关闭，应先在 Windows 仓库根目录运行启动器，等待各服务就绪和双臂初始化结束：

```powershell
powershell -ExecutionPolicy Bypass -File .\grasp\scripts\start_tmr_system.ps1
```

然后登录 `.100` 执行上面的最终比赛入口。比赛自动流程前不要使用 `-EnableTeleop`；任务入口会自行关闭手柄控制，任务结束后再自动恢复。冷启动器与完整任务是两个步骤：前者负责底层服务和初始姿态，后者负责比赛动作。

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

开始后不要在当前位置再次运行同一条“从起点完整运行”命令，否则会重复前进和转向。只有机器人被人工放回标记起点后才重新完整运行；状态明确的中途恢复使用后文的专用参数。

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

### 6. 初始双臂姿态或夹爪状态不正确

完整任务要求左臂处于已验证的夹取顶层姿态、右臂内收并在后续保持、左夹爪在下降前张开。初始化并不是厂家零位。若初始状态明显不对，应终止任务，不要让抓取阶段用大容差继续；重新运行 Windows 启动器完成双臂、夹爪和 `0.70 m` 升降柱初始化，再从标记起点开始。

### 7. 识别到了错误物体，或杯子位置偏得很远

历史原因主要是相机网页残留帧、相机会话切换、只看颜色而忽略尺寸，以及画面抖动时轮廓在多个候选间跳变。当前版本要求同一相机会话中的新鲜连续 RGB 帧，并联合浅绿色、稳定轮廓和直径 `76/120/185 mm` 区分杯子、食物盆和盘子。识别失败会停止本件物体，不会沿用上一件物体的坐标盲抓。恢复时先确认 `/healthz` 的帧序号持续增长，再从确定的物品桌检查点恢复；不要仅因为浏览器仍显示旧图就继续执行。

### 8. 把餐具、盘子或倒置字母误认为 A/B/D

当前字母授权只接受亮白近矩形卡片内部的字符，并结合字母孔洞拓扑、连续新帧和底盘实际位移。杯子、食物盆必须先对准目标字母再额外右移 `0.08 m`；盘子面对 D 时采用经过实机验证的专用直接放置。若画面不是真实白卡，流程不应授权放置；不要通过放宽 OCR 阈值绕过它。

### 9. 动作卡住、下降后直接上升或夹爪顺序异常

历史上零散远程命令发生过乱序，PTP 也可能在极小关节误差处长期保持运行。当前每次抓取均在单一状态机内执行 `张开 → 对准 → 下降 → 闭合 → 抬升`，动作有硬超时，接近目标后最多进行一次有限修正；闭合完成前不会进入抬升。若仍异常退出，保持现场状态并查看 `~/.tmr_three_object_delivery/` 的本轮日志，按明确检查点恢复，不要连续重发单步夹爪命令。

### 10. FK/IK 服务或 ROS 域不匹配

底盘固定在 `.50` 的 ROS 2 Humble/Domain 97 本机图中运行；机械臂、夹爪和几何服务在 `.100` 的机器人环境中运行；ZED 使用独立视觉域并通过新鲜 JPEG 交给任务。不要把这些域手工统一，也不要跨主机桥接实时速度或关节流。Windows 启动器会启动左臂 FK/Cartesian 几何服务；若 `/left_ik/compute_fk` 或 `/left_ik/compute_cartesian_path` 缺失，重新运行启动器恢复服务后再启动任务。

### 11. 如何判断能否重跑

- 机器人已回到标记起点、手中无物体：使用最终一键入口完整重跑。
- 已在物品桌且手中无物体：使用对应的 `--resume-...-confirmed` 恢复参数。
- 已确认夹住杯子并处于标准抬升高度：只使用 `--resume-after-cup-held-confirmed`。
- 位置、所持物体或当前阶段不确定：不要猜检查点；先人工复位到标记起点。

更完整的历史原因、代码层修复和设计约束见 [`docs/FAILURE_FIXES.md`](docs/FAILURE_FIXES.md)。旧的单步调试脚本仍保留用于诊断，但比赛时只使用本 README 的最终入口，避免混入旧参数或旧动作顺序。

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
