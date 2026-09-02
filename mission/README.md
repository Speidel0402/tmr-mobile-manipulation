# 综合长程抓取流程

## 最终三物体比赛入口（2026-09-02 实机通过）

从标记起点完整运行杯子→B、食物盆→A、盘子→D：

```bash
cd /home/aup/tmr-mobile-manipulation
bash mission/scripts/run_complete_from_start.sh \
  --cup-letter B --bowl-letter A --plate-letter D
```

该入口固定调用 `run_three_object_delivery.py --execute --fresh-start-confirmed`，并禁止误传中途恢复参数。本次完整实机运行编号为 `36a78660aa63`，三次抓取、三次字母授权、三次释放均完成，最后自动恢复手柄控制。杯子、食物盆和盘子的下降量分别为 `0.340/0.360/0.375 m`；盘子面对 D 时连续两帧确认即直接放置，不再附加横移。

仓库根目录 [`README.md`](../README.md) 记录最终参数、一键运行要求、控制器/里程计/相机故障恢复和分物体续跑命令。下文保留单段入口，供诊断与受控恢复使用。

入口：`scripts/run_long_range_pick.py`。

该协调器运行在机械臂主机 `172.16.0.100`，只负责阶段切换：底盘的闭环速度控制始终在 `172.16.0.50` 本地执行，视觉、机械臂和夹爪控制始终在 `172.16.0.100` 本地执行。SSH 不承载任何实时控制流。

固定时序：

1. 底盘前进 `0.85 m`；
2. 顺时针旋转 `90°`；
3. 使用本轮实时地图与双雷达识别门框并横移到中线；
4. 到门前 `0.50 m`，沿中垂线前进 `1.20 m`，锁存零速；
   底盘需在最长 `4 s` 内通过静止窗口、零命令和独占租约确认，才允许进入机械臂阶段；
5. 左夹爪确认张开；
6. 左臂恢复实机成功姿态，进行杯子右边缘视觉校准；
7. 下降后闭合夹爪，确认真实夹持，再提升；
8. 等待 PTP 清理完成，稳定恢复阻抗保持后才报告完成。

当前比赛综合入口按现场要求向底盘任务显式传入 `--disable-collision-guard`：不使用雷达/地图扫掠、旋转净空或门侧净空来中断运动。里程计闭环、传感器新鲜度、速度发布者独占、阶段超时和无进展停止仍保留，避免控制进程卡住或持续发送命令。

只检查策略，不连接机器人：

```bash
python3 mission/scripts/run_long_range_pick.py
```

在机械臂主机正式执行：

```bash
python3 mission/scripts/run_long_range_pick.py --execute
```

如果底盘已经成功到位、但抓取阶段失败，并且检查点明确记录机械臂已安全恢复到顶部保持状态，可只重跑抓取，避免再次前进或重复旋转：

```bash
python3 mission/scripts/run_long_range_pick.py --execute --resume-grasp
```

如果失败发生在闭合、已确认夹取、提升或人工停止阶段，`--resume-grasp` 会拒绝自动张开夹爪，必须先人工确认/恢复机械臂状态。

检查点默认保存在 `~/.tmr_long_range_pick/state.json`。存在旧检查点时，只有机器人已经人工返回地面标记起点，才可用 `--fresh-start-confirmed` 开始新的完整路线。综合脚本不会自动重复底盘阶段，也不会在抓取失败后释放底盘的零速租约。任务全程有本机文件锁，底盘端另有内核文件锁，重复启动会在运动前失败。

运行前需保证：底盘本地控制器、双雷达、SLAM、速度适配器已常驻；两台主机间已配置 SSH 密钥；机械臂主机的左臂、夹爪、D405、RGB 快照服务和 `left_ik` 服务已启动。RGB 快照必须由本版本 `tools/camera_mjpeg_viewer.py` 提供；抓取脚本会在张开夹爪前验证左腕相机、640×480 图像和本次相机会话，并在视觉校准期间拒绝相机进程切换。

ROS 环境脚本必须先完成加载，随后才能启用 shell 的 `nounset` 严格检查；本版协调器已固定该顺序，避免 `AMENT_TRACE_SETUP_FILES: unbound variable` 在运动前中断。机械臂主机首次部署时允许创建 `/home/aup/tmr-mobile-manipulation`，并需保证该主机可用 SSH key 无交互连接 `tmr-user@172.16.0.50`。

## 实机验证的自适应字母投放版本

入口：`scripts/run_letter_delivery_competition.py`，参数集中在
`config/letter_delivery.json`。可用 `--target-letter A|B|E` 临时覆盖目标，
也可用 `--row near|far|auto` 指定或自动判断远近排。

后段顺序固定为：已验证的退后/180°/后退前缀 → 最多向右 2.40 m 的
ZED 白卡字母搜索 → 连续帧居中 → near 直接下降 0.36 m 放置，或 far
前伸 0.16 m、下降 0.36 m 放置并回缩 → 按本轮实测右移距离等距向左 → 逆时针 180° →
实时门中线 → 门前 0.50 m → 前进 1.20 m → 零速保持。

只输出策略，不连接机器人：

```bash
python3 mission/scripts/run_letter_delivery_competition.py --target-letter B
```

视觉只使用 ZED RGB，不使用深度。ZED 默认运行在独立视觉域 1，最新压缩帧
通过原子 JPEG 文件交给控制域 0 的搜索器，避免大图像 DDS 流量污染底盘速度
控制。底盘主机需在部署阶段一次性运行
`base/scripts/14_prepare_letter_vision.sh`；比赛运行期间不会安装依赖。

实机 B 流程已验证：B 近排居中、下降 0.36 m 放置、按实测横移距离返回、
逆时针 180°、重新识别门中线并完成 `0.50 m + 1.20 m` 回程。门框子流程
失败但横移和旋转已完成时，可用 `--resume-door` 从 `DOOR_RETURN_FAILED`
检查点续跑，不会重复前两段运动。
