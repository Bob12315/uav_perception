# uav_perception

基于 ROS 2 的无人机视觉感知包。当前提供 `yolo_detector` 节点：订阅图像、执行 YOLO 推理、发布标注图和结构化检测结果。

## 目标
这个包已经改为“可复刻”设计：
- 不再依赖固定机器路径
- 不再写死相机话题
- 提供统一参数文件和 launch
- 支持直接被其他人 `colcon build` 后复用

## 包结构
- `uav_perception/yolo_detector.py`: 主节点
- `config/yolo_detector.yaml`: 参数模板
- `launch/yolo_detector.launch.py`: 启动文件
- `best.pt`: 示例默认权重（可替换）

## 依赖
### ROS 2 依赖
- `rclpy`
- `ament_index_python`
- `sensor_msgs`
- `std_msgs`
- `launch`
- `launch_ros`

### Python 依赖
- `ultralytics`
- `opencv-python`
- `numpy`

## 安装依赖命令
先安装 ROS 依赖：
```bash
cd $HOME/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

再安装 Python 依赖：
```bash
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python numpy
```

## 构建
```bash
cd $HOME/ros2_ws
colcon build --packages-select uav_perception
source install/setup.bash
```

## 运行
### 方式 1：直接运行
```bash
ros2 run uav_perception yolo_detector
```

### 方式 2：用 launch + yaml（推荐）
```bash
ros2 launch uav_perception yolo_detector.launch.py
```

指定自己的参数文件：
```bash
ros2 launch uav_perception yolo_detector.launch.py \
  config_file:=/absolute/path/to/yolo_detector.yaml
```

## 参数
- `model_path` (string)
  - 默认：若为空或未设置，自动尝试包内 `best.pt`
- `conf_thres` (double)
  - 默认：`0.25`
- `input_image_topic` (string)
  - 默认：`/camera/image_raw`
- `output_image_topic` (string)
  - 默认：`/perception/image_annotated`
- `detections_topic` (string)
  - 默认：`/perception/detections_json`
- `lock_target_topic` (string)
  - 默认：`/perception/lock_target`
  - 设为空字符串可禁用锁定订阅
- `lock_color` (int[3], BGR)
  - 默认：`[255, 0, 0]`
- `lock_thickness` (int)
  - 默认：`3`

## 输出数据
### 标注图像
发布到 `output_image_topic`，类型 `sensor_msgs/Image`。

### JSON 检测结果
发布到 `detections_topic`，类型 `std_msgs/String`。
`data` 为 JSON，结构如下：
```json
{
  "header": {"stamp_sec": 1700000000.123456},
  "image": {"width": 640, "height": 480},
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

## 锁定输入格式
`lock_target_topic` 需要 `std_msgs/String`，`data` 示例：
```json
{"locked": true, "xyxy": [120, 80, 240, 220], "cx": 180, "cy": 150, "score": 0.91}
```

## 迁移到新项目的最小步骤
1. 拷贝该包到你的 `ros2_ws/src`。
2. 把你的模型权重路径填到 `config/yolo_detector.yaml` 的 `model_path`。
3. 把 `input_image_topic` 改成你的相机话题。
4. `colcon build --packages-select uav_perception`。
5. `ros2 launch uav_perception yolo_detector.launch.py`。

## 常见问题
### 启动报错 `model_path not found`
模型路径无效。请设置正确的 `model_path`，或把模型放到包内并命名为 `best.pt`。

### 看到图像但无检测框
可能是 `conf_thres` 太高、模型类别不匹配或图像质量较差。先把阈值降到 `0.1~0.2` 验证。

### 报 `Unsupported image encoding`
当前仅支持 `bgr8` / `rgb8`，请在上游统一编码。
