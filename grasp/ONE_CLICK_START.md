# TMR 一键启动与初始化

本地入口：

```powershell
powershell -ExecutionPolicy Bypass -File .\rim_grasp_perception\scripts\start_tmr_system.ps1
```

脚本依次连接三台主机并启动：

1. `172.16.0.50`：TMR 底盘、ZED-M 头部相机；
2. `172.16.0.100`：Spine、Robotiq 双夹爪、FR3 双臂、D405 双腕相机；
3. 初始化：底盘零速度、左臂和右臂顺序低速恢复“夹取初始状态”、双夹爪张开至 `0.8`，Spine 上电并移动至 `0.7 m`；
4. `172.16.0.101`：仅在指定 `-EnableTeleop` 时启动 GELLO 与脚踏板。

## 使用方式

建议先给三台主机配置 OpenSSH 密钥或 `ssh-agent`，这样才是真正无需重复输入密码的一键启动。脚本和配置均不保存密码；未配置密钥时，各 SSH 窗口会自行提示输入密码。

完整启动但不启用遥操作：

```powershell
.\rim_grasp_perception\scripts\start_tmr_system.ps1
```

完成初始化后再启用 GELLO 和脚踏板：

```powershell
.\rim_grasp_perception\scripts\start_tmr_system.ps1 -EnableTeleop
```

只启动节点，不恢复双臂或初始化夹爪：

```powershell
.\rim_grasp_perception\scripts\start_tmr_system.ps1 `
  -SkipArmRestore -SkipGripperInitialize
```

只打印计划执行的远程命令，不连接机器人：

```powershell
.\rim_grasp_perception\scripts\start_tmr_system.ps1 -DryRun
```

## 配置

所有可调项位于 `config/system_startup.psd1`：

- 三台主机地址和用户名；
- 当前 D405 左右序列号；
- 当前 Robotiq `by-id` 映射；
- 双臂“夹取初始状态”的 7 关节角；
- 最大恢复速度和容差；
- 各节点启动等待时间；
- 夹爪初始化开度；
- Spine 夹取初始绝对高度、速度、加速度和减速度。

当前实际映射为：

- 左 D405：`409122272639`；
- 右 D405：`409122274492`；
- 左夹爪：`DAANVRU5`；
- 右夹爪：`DAANTK6Q`；
- 左 FR3：`/left`，`172.16.16.12`；
- 右 FR3：`/right`，`172.16.16.11`。

## “零位”的语义

- 双臂恢复到此前人工摆放并保存的“夹取初始状态”，不是厂家机械零点。
- 夹爪初始化为张开 `0.8`。
- 底盘只发送零速度以保持静止。脚本不会自动驶回里程计原点，因为那不是经过验证的安全物理路径。
- Spine 的夹取初始高度已由用户现场确认并配置为绝对位置 `0.7 m`。初始化流程会先加载 `~/tmr_env.sh`，调用 `/franka_spine_node/switch_on`，再向 `/franka_spine_node/move_absolute` 发送目标并等待反馈。
- 相机、ZED 和其他传感器的“初始化”是启动驱动并开始发布，不包含机械校准或清零标定。

## 注意事项

- 运行前确保机器人周围无人、无障碍物，并可随时触发急停。
- 不要在系统已部分运行且状态不明确时反复执行脚本。
- 双臂恢复采用顺序动作：左臂完成并恢复阻抗控制后，右臂才开始。
- `-EnableTeleop` 会在自动初始化结束后启动，但仍可能立即接收 GELLO/脚踏板输入；无人操作时不要启用。
- 该版本按要求只完成代码实现，尚未执行完整冷启动验证。
