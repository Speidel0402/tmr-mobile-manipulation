# 左腕真实图像 Demo 运行报告（2026-08-30）

## 输入

- 实时流：`http://127.0.0.1:18080/rgb.mjpg`
- 来源：`/wrist_camera_left/color/image_raw`
- 分辨率：848×480
- 场景：画面上方白色桌面，桌面上有一个绿色碗。

深度预览有信号，但网页输出是 0–2 m 伪彩 JPEG，不是原始 `16UC1`。本次 Demo 未伪造三维坐标。

## 实际执行

```powershell
python scripts\run_camera_image_demo.py
```

结果：

```text
camera_id: left
frame_id: wrist_camera_left_color_optical_frame
valid: true
category: bowl
center_px: approximately [436, 55]
rim_radius_px: approximately 49
edge_support: approximately 0.98
selection_score: approximately 0.87
selected: true
rim_position_3d: null
contact_pose: null
```

输出位于 `outputs/camera_image_demo_left/`。叠加图标出了桌面、碗口沿、选择状态和二维径向闭合方向。

## 多目标行为

若发现多个器皿：

1. invalid 目标全部剔除；
2. 按几何质量、检测质量、深度支持、边缘支持计算选择分数；
3. 只将最高分目标标记为 `selected=true`；
4. 在线 ROS 节点只发布该目标的 contact pose 和 Marker；
5. JSON 保留其余目标及拒绝原因，便于调试。

## 验证

- 真实左腕 RGB 抓帧成功。
- 真实桌面区域与碗口沿识别成功。
- 多目标选择单元测试已加入。
- 核心深度、变换、口沿、杯底拒绝和失效测试继续通过。
- 没有发送任何机械臂或夹爪命令。

恢复本地 ROS 2 Jazzy、模型权重和原始深度订阅后，可用同一在线节点输出左相机 optical frame 下的三维接触目标；加入手眼 TF 后才能转换到机械臂基座/TCP frame。
