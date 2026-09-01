# 左腕 D405 桌面器皿口沿感知

项目只做感知：读取左腕 RealSense D405，寻找桌面上的 cup、bowl、plate，验证真实口沿并选择一个夹取候选；不发送机械臂、控制器或夹爪命令。

## 当前运行状态

- 左腕 RGB 和深度正在约 30 Hz 发布。
- RGB：`/wrist_camera_left/color/image_raw`，848×480，`rgb8`。
- 深度：`/depth/image_rect_raw`，848×480，`16UC1`。
- 深度没有对齐 RGB，在线节点会用内参与 depth→color 外参重投影，不做大范围空洞插值。
- 当前只有相机内部 TF，缺少机械臂根部/末端到左腕相机的手眼链。因此目前可以输出相机 optical-frame 坐标，但不能标成机器人基座/TCP 坐标。

## 多目标选择逻辑

每帧先对所有目标独立执行：检测掩码、桌面拟合、内部边缘、椭圆残差、口沿深度支持、三维高度、夹指体积和下降路径检查。invalid 目标永不入选。

多个 valid 目标按以下分数选择一个：

```text
0.55 × geometry_score
+ 0.30 × detection_score
+ 0.10 × rim_depth_support
+ 0.05 × edge_support
```

JSON 保留全部物体，并以 `diagnostics.selected=true` 标记被选目标；`contact_pose` 和 RViz Marker 只发布该目标。该分数是确定性排序质量，不是成功概率。

## 真实左腕图像 Demo

现有本地查看器为 `http://127.0.0.1:18080`。运行：

```powershell
cd C:\Users\ckck9\Desktop\frankaCV\rim_grasp_perception
python scripts\run_camera_image_demo.py
```

也可指定保存的真实图像：

```powershell
python scripts\run_camera_image_demo.py --input outputs\camera_left_capture\rgb.jpg
```

Demo 执行桌面区域提取、圆形口沿候选、边缘覆盖率、器皿内外颜色差和多目标选择，结果保存到：

```text
outputs/camera_image_demo_left/
  rgb.jpg
  table_mask.png
  overlay.jpg
  result.json
```

当前查看器的 `/snapshot.npz` 提供 RGB、对齐到彩色的原始深度、CameraInfo 内参和时间戳。Demo 使用原始深度反投影口沿点；不会用伪彩 JPEG 反推米制深度。输出坐标仍属于相机光学 frame，不是机械臂基座或 TCP 坐标。

## 模型与 ROS 依赖

```bash
cd /path/to/frankaCV/rim_grasp_perception
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install -r requirements-dev.txt
python -m pip install -r requirements-ml.txt
python scripts/download_weights.py --cache models

sudo apt install ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-tf2-ros ros-jazzy-visualization-msgs \
  ros-jazzy-realsense2-camera-msgs
```

当前 Windows 的 PyPI/Hugging Face 连接被内部代理阻断，SAM 2、Open3D 和模型权重尚未安装完成；真实 RGB Demo 使用 OpenCV 候选验证。生产在线节点仍按固定路线加载本地 Grounding DINO + SAM 2，并使用 Open3D（不可用时仅测试用 NumPy RANSAC 回退）。

构建：

```bash
cd /path/to/frankaCV
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rim_grasp_perception
source install/setup.bash
```

## 启动左腕在线节点

本地 ROS 2 Jazzy 主机必须能直接接收机器人 DDS 数据：

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///absolute/path/to/config/cyclonedds_local.xml

ros2 launch rim_grasp_perception left_wrist.launch.py \
  config:=/absolute/path/to/rim_grasp_perception/config/left.yaml
```

订阅：

```bash
ros2 topic echo /rim_grasp/left/result_json std_msgs/msg/String
ros2 topic echo /rim_grasp/left/contact_pose geometry_msgs/msg/PoseStamped
ros2 run rqt_image_view rqt_image_view /rim_grasp/left/overlay
```

`result_json` 是权威状态。没有有效口沿、深度不足、下降路径未知或目标超时都会发布 invalid；旧目标不会继续有效。

## 坐标语义

- `rim_position`：真实观测口沿表面位置。
- `contact_pose`：内外夹指计划接触中心；+X 为向内闭合，-Z 为接近方向。
- `tcp_pose`：只有配置 `contact_to_tcp` 标定后才提供，否则为 null。
- 当前默认 frame 是 `wrist_camera_left_color_optical_frame`。

透明/反光、浅盘沿、堆叠遮挡和大面积缺深度会被保守拒绝。局部 RGB-D 检查不代表整臂可达性或完整避碰验证。

## 测试

```bash
python -m pytest -q
python -m compileall rim_grasp_perception scripts launch
```

## 左臂 MoveIt IK-only

项目提供一个仅计算、不执行运动的左臂 IK 适配层。它复用机器人已有的
`franka_mobile_fr3_duo_moveit_config`、`left_arm` 规划组和 KDL 插件，使用
`/left/franka_robot_state_broadcaster/measured_joint_states` 作为7关节 seed。
launch 显式关闭 MoveIt 轨迹执行和控制器管理，不会生成、激活或切换
`full_body_controller`，也不会向 Franka 控制器或 PTP action 发送目标。

实机的关节状态发布 QoS 是 `BEST_EFFORT`，而 MoveIt 状态监视器要求
`RELIABLE`。`left_ik_client` 会把它只读转发到 `/left_ik/joint_states`；这不是
控制命令话题。

该功能必须在 ROS 2 Jazzy/MoveIt 2 的 Ubuntu 环境运行；原生 Windows Python
不能运行。代码可继续保存在 Windows 工作区，并从装有相同机器人描述包的
WSL2 Ubuntu 使用：

```bash
cd /mnt/c/Users/ckck9/Desktop/frankaCV
source /opt/ros/jazzy/setup.bash
# 还需要 source 包含 franka_mobile_fr3_duo_moveit_config 的工作区
colcon build --packages-select rim_grasp_perception --symlink-install
source install/setup.bash
ros2 launch rim_grasp_perception left_ik_only.launch.py
```

接口：

```text
输入  /rim_grasp/left/contact_pose      geometry_msgs/msg/PoseStamped
输出  /rim_grasp/left/ik_joint_target   sensor_msgs/msg/JointState
诊断  /rim_grasp/left/ik_result_json    std_msgs/msg/String
服务  /left_ik/compute_ik               moveit_msgs/srv/GetPositionIK
```

查看结果：

```bash
ros2 topic echo /rim_grasp/left/ik_result_json
ros2 topic echo /rim_grasp/left/ik_joint_target
```

输入必须是完整6DoF `PoseStamped`，并使用 MoveIt TF 树中可转换的 frame。
垂直向下抓取时，固定接近方向为公共参考系 `-Z`，闭合方向取桌面内口沿径向；
第7自由度不作为“第7维坐标”输入，而由 KDL 从当前7关节 seed 附近求解。
在相机到左臂基座的手眼 TF 缺失时，相机坐标目标会返回
`FRAME_TRANSFORM_FAILURE`，不得把它作为可执行关节目标。

当前 `avoid_collisions=false`，因为完整规划场景和双臂状态尚未验证；输出只用于
IK自检，不能视为整臂避碰、可达性或可安全执行的证明。

2026-08-31 在 `172.16.0.100` 上做过一次临时启动验证：
`/left_ik/compute_ik` 服务成功建立，`/left_ik` 下没有动作服务器，测试期间机械臂
未运动。测试随后用 Ctrl-C 停止并确认无残留进程。该主机的 MoveIt 2.12.4 在
退出析构阶段报告过一次段错误；不影响上述服务启动验证，但正式常驻前应继续
验证其退出稳定性。
