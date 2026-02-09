# uav_perception

`uav_perception` 是一个基于 ROS 2 的视觉感知包，当前提供 `yolo_detector` 节点。
该节点负责订阅图像、执行 YOLO 目标检测、发布标注图像，以及发布结构化 JSON 检测结果。

本 README 目标是让其他人可以在不同机器上快速复现和复用该包。

## 1. 功能概览
- 订阅输入图像（`sensor_msgs/Image`）
- 使用 Ultralytics YOLO 执行目标检测
- 发布标注图像（框、类别、置信度）
- 发布检测 JSON（`std_msgs/String`）
- 发布融合状态 JSON（`std_msgs/String`，含 target/gimbal/uav）
- 支持锁定目标 overlay（LOCKED 框、中心十字、箭头、误差文本）
- 每秒打印输入流状态（fps、分辨率、编码）

## 2. 包结构
```text
uav_perception/
├── config/
│   └── yolo_detector.yaml          # 参数模板
├── launch/
│   └── yolo_detector.launch.py     # 启动文件
├── resource/
│   └── uav_perception
├── uav_perception/
│   ├── __init__.py
│   └── yolo_detector.py            # 主节点实现
├── best.pt                         # 默认模型权重（可替换）
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 3. 环境要求
- Ubuntu 22.04（推荐）
- ROS 2 Humble（或兼容版本）
- Python 3.10+

## 4. 依赖说明
### 4.1 ROS 2 依赖（在 `package.xml` 中）
- `rclpy`
- `ament_index_python`
- `sensor_msgs`
- `std_msgs`
- `launch`
- `launch_ros`

### 4.2 Python 依赖
- `ultralytics`
- `opencv-python`
- `numpy`

## 5. 安装依赖
在工作空间根目录执行：

### 5.1 安装 ROS 依赖
```bash
cd $HOME/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5.2 安装 Python 依赖
```bash
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python numpy
```

可选自检：
```bash
python3 -c "import ultralytics, cv2, numpy; print('python deps ok')"
```

## 6. 构建包
```bash
cd $HOME/ros2_ws
colcon build --packages-select uav_perception
source install/setup.bash
```

## 7. 配置参数（推荐先改 YAML）
参数文件：`config/yolo_detector.yaml`

关键参数：
- `model_path`：YOLO 权重路径。
  - 为空时，节点会尝试使用包内默认 `best.pt`。
- `conf_thres`：检测置信度阈值，默认 `0.25`。
- `input_image_topic`：输入图像话题，默认 `/camera/image_raw`。
- `output_image_topic`：标注图像输出话题，默认 `/perception/image_annotated`。
- `detections_topic`：JSON 输出话题，默认 `/perception/detections_json`。
- `lock_target_topic`：锁定输入话题，默认 `/perception/lock_target`。
  - 设为 `""` 可禁用锁定 overlay。
- `fusion_topic`：融合状态 JSON 输出，默认 `/perception/fusion_state_json`。
- `gimbal_angle_topic`：云台角输入，默认 `/uav/gimbal/angle`。
- `uav_odom_topic`：无人机姿态/高度输入，默认 `/uav/odom`。
- `lock_color`：锁定框颜色（BGR），默认 `[255, 0, 0]`。
- `lock_thickness`：锁定框/箭头线宽，默认 `3`。

## 8. 运行方式
### 8.1 直接运行节点（不推荐）
```bash
ros2 run uav_perception yolo_detector
```

### 8.2 使用 launch + YAML（推荐）
```bash
ros2 launch uav_perception yolo_detector.launch.py
```

使用自定义参数文件：
```bash
ros2 launch uav_perception yolo_detector.launch.py \
  config_file:=/absolute/path/to/yolo_detector.yaml
```

## 9. 话题接口
### 9.1 订阅话题
- `input_image_topic`（默认 `/camera/image_raw`）
  - 类型：`sensor_msgs/Image`
- `lock_target_topic`（默认 `/perception/lock_target`）
  - 类型：`std_msgs/String`
  - 内容：JSON 字符串（见第 11 节）
- `gimbal_angle_topic`（默认 `/uav/gimbal/angle`）
  - 类型：`geometry_msgs/Vector3`
- `uav_odom_topic`（默认 `/uav/odom`）
  - 类型：`nav_msgs/Odometry`

### 9.2 发布话题
- `output_image_topic`（默认 `/perception/image_annotated`）
  - 类型：`sensor_msgs/Image`
- `detections_topic`（默认 `/perception/detections_json`）
  - 类型：`std_msgs/String`
  - 内容：JSON 字符串（见第 10 节）
- `fusion_topic`（默认 `/perception/fusion_state_json`）
  - 类型：`std_msgs/String`
  - 内容：JSON 字符串（见第 10.1 节）

## 10. 检测 JSON 输出格式
`detections_topic` 的 `data` 示例：

```json
{
  "header": {
    "stamp_sec": 1700000000.123456
  },
  "image": {
    "width": 640,
    "height": 480
  },
  "detections": [
    {
      "cls_id": 0,
      "cls_name": "person",
      "conf": 0.92,
      "xyxy": [100.5, 50.2, 220.8, 300.1],
      "cx": 160.65,
      "cy": 175.15,
      "area": 30161.38
    }
  ]
}
```

字段说明：
- `header.stamp_sec`：原始图像时间戳（秒，浮点）
- `image.width` / `image.height`：图像宽高
- `detections`：检测数组（可为空）
- `cls_id`：类别 ID
- `cls_name`：类别名
- `conf`：置信度
- `xyxy`：边框坐标 `[x1, y1, x2, y2]`
- `cx` / `cy`：边框中心
- `area`：边框面积（像素平方）

## 10.1 融合状态 JSON 输出格式
`fusion_topic` 的 `data` 示例：

```json
{
  "stamp_sec": 1700000000.123456,
  "image": {"w": 640, "h": 480},
  "target": {
    "locked": true,
    "cls_name": "bucket",
    "conf": 0.91,
    "xyxy": [120, 80, 240, 220],
    "cx": 180,
    "cy": 150,
    "area": 16800.0
  },
  "gimbal": {"pitch_deg": -37.2, "roll_deg": 0.1, "yaw_deg": 5.0},
  "uav": {"roll_deg": 1.5, "pitch_deg": -0.8, "yaw_deg": 92.0, "alt_rel_m": 12.3}
}
```

## 11. 锁定输入格式
`lock_target_topic` 需要 `std_msgs/String`，`data` 为 JSON，例如：

```json
{"locked": true, "xyxy": [120, 80, 240, 220], "cx": 180, "cy": 150, "score": 0.91}
```

字段说明：
- `locked`：是否锁定，`false` 时不显示锁定 overlay
- `xyxy`：锁定框 `[x1, y1, x2, y2]`
- `cx` / `cy`：中心点（缺省时由 `xyxy` 自动计算）
- `score`：锁定评分（调试/透传）

## 12. 图像编码兼容
当前支持输入编码：
- `bgr8`
- `rgb8`（节点内部自动转 BGR）

不支持的编码会在日志中提示 `Unsupported image encoding`，并跳过推理。

## 13. 日志说明
节点每秒输出一条状态日志，例如：

```text
[INFO] [..] [yolo_detector]: fps=30 width=640 height=480 encoding=rgb8
```

含义：
- `fps`：过去 1 秒接收帧数
- `width/height`：最近一帧分辨率
- `encoding`：最近一帧编码

## 14. 常见问题
### 14.1 启动报错：`model_path not found`
原因：模型路径无效。

处理：
- 在 `config/yolo_detector.yaml` 设置正确 `model_path`
- 或将模型放到包内并命名为 `best.pt`

### 14.2 有图像但没有检测框
可能原因：
- `conf_thres` 过高
- 模型类别与场景不匹配
- 输入图像质量较差

处理建议：
- 先将 `conf_thres` 临时降到 `0.1 ~ 0.2`
- 确认模型训练类别覆盖当前目标

### 14.3 提示 `Unsupported image encoding`
原因：输入不是 `bgr8`/`rgb8`。

处理：
- 在上游将编码转换为 `bgr8` 或 `rgb8`

### 14.4 性能不足（fps 低）
建议：
- 使用更轻量模型（如 `yolov8n`）
- 降低输入分辨率
- 在回调中跳帧
- 使用 CUDA 环境并安装匹配的 PyTorch/CUDA

## 15. 复刻到新项目（最小步骤）
1. 把 `uav_perception` 放入目标工作空间 `src/`。
2. 修改 `config/yolo_detector.yaml`：
   - `model_path`
   - `input_image_topic`
3. 安装依赖（第 5 节）。
4. 构建（第 6 节）。
5. 启动（第 8 节）。

## 16. 快速命令清单
```bash
# 1) 依赖
cd $HOME/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python numpy

# 2) 构建
colcon build --packages-select uav_perception
source install/setup.bash

# 3) 运行
ros2 launch uav_perception yolo_detector.launch.py

# 4) 观察输出
ros2 topic echo /perception/detections_json
```
