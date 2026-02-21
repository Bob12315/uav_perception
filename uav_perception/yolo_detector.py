#!/usr/bin/env python3

import json
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


def _candidate_model_paths():
    paths = []
    try:
        share_dir = get_package_share_directory('uav_perception')
        paths.extend(
            [
                os.path.join(share_dir, 'gazebo_yolo.pt'),
                os.path.join(share_dir, 'best.pt'),
            ]
        )
    except PackageNotFoundError:
        pass

    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    paths.extend(
        [
            os.path.join(pkg_root, 'gazebo_yolo.pt'),
            os.path.join(pkg_root, 'best.pt'),
        ]
    )

    # 去重并保持顺序，便于日志输出可读。
    return list(dict.fromkeys(paths))


def _default_model_path() -> str:
    """返回首个存在的默认模型路径；若都不存在则返回首个候选路径。"""
    candidates = _candidate_model_paths()
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0] if candidates else ''


def _resolve_model_path(configured_path: str) -> str:
    candidates = []
    cfg = configured_path.strip()
    if cfg:
        candidates.append(os.path.abspath(os.path.expanduser(cfg)))
    candidates.extend(_candidate_model_paths())

    # 去重并保持顺序。
    candidates = list(dict.fromkeys(candidates))
    for path in candidates:
        if os.path.isfile(path):
            return path
    return ''


class YoloDetector(Node):
    def __init__(self) -> None:
        """ROS2 YOLO 检测节点。

        数据流:
        1) 订阅相机 `sensor_msgs/Image`
        2) 转为 OpenCV BGR 图做 YOLO 推理
        3) 发布带框图像 `/perception/image_annotated`
        4) 发布结构化 JSON `/perception/detections_json`
        """
        super().__init__('yolo_detector')

        # 仅用于状态统计:
        # - _last_msg: 最近一帧，用于打印宽高/编码
        # - _frames: 1 秒窗口内累计帧数（在 _on_timer 中清零）
        self._last_msg = None
        self._frames = 0
        # 通过参数配置模型路径与置信度阈值，便于 launch 文件覆盖:
        # - model_path: YOLO 权重文件路径
        # - conf_thres: 最低置信度阈值（低于阈值的检测会被过滤）
        configured_model_path = str(
            self.declare_parameter('model_path', _default_model_path()).value
        )
        model_path = _resolve_model_path(configured_model_path)
        self._conf_thres = float(self.declare_parameter('conf_thres', 0.25).value)
        input_image_topic = str(
            self.declare_parameter('input_image_topic', '/camera/image_raw').value
        )
        output_image_topic = str(
            self.declare_parameter('output_image_topic', '/perception/image_annotated').value
        )
        detections_topic = str(
            self.declare_parameter('detections_topic', '/perception/detections_json').value
        )
        lock_target_topic = str(
            self.declare_parameter('lock_target_topic', '/perception/lock_target').value
        )
        lock_color_param = self.declare_parameter('lock_color', [255, 0, 0]).value
        self._lock_thickness = int(self.declare_parameter('lock_thickness', 3).value)
        self._lock_color = (255, 0, 0)
        if isinstance(lock_color_param, list) and len(lock_color_param) == 3:
            try:
                self._lock_color = (
                    int(lock_color_param[0]),
                    int(lock_color_param[1]),
                    int(lock_color_param[2]),
                )
            except Exception:
                self._lock_color = (255, 0, 0)

        self._lock_active = False
        self._lock_xyxy = [0.0, 0.0, 0.0, 0.0]
        self._lock_cx = 0.0
        self._lock_cy = 0.0
        self._lock_score = 0.0
        self._lock_cls_name = ''
        self._lock_status = 'pending'
        self._lock_done_count = 0
        self._lock_target_drop_count = 0
        self._lock_task_done = False
        self._lock_ex = 0.0
        self._lock_ey = 0.0

        if not model_path:
            tried = []
            if configured_model_path.strip():
                tried.append(os.path.abspath(os.path.expanduser(configured_model_path.strip())))
            tried.extend(_candidate_model_paths())
            tried = list(dict.fromkeys(tried))
            self.get_logger().error(
                'model_path not found. tried: ' + ', '.join(tried)
            )
            raise SystemExit(1)
        if configured_model_path.strip() and os.path.abspath(
            os.path.expanduser(configured_model_path.strip())
        ) != model_path:
            self.get_logger().warning(
                f'configured model_path unavailable, fallback to: {model_path}'
            )
        self.model = YOLO(model_path)

        # 图像链路优先实时性，采用 BEST_EFFORT。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        # JSON 检测结果优先可靠送达，采用 RELIABLE。
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Image, output_image_topic, qos)
        self._json_pub = self.create_publisher(String, detections_topic, reliable_qos)
        self._sub = self.create_subscription(
            Image,
            input_image_topic,
            self._on_image,
            qos,
        )
        self._lock_sub = None
        if lock_target_topic:
            self._lock_sub = self.create_subscription(
                String,
                lock_target_topic,
                self._on_lock_target,
                reliable_qos,
            )
        self._timer = self.create_timer(1.0, self._on_timer)
        self.get_logger().info(
            f'config: model={model_path}, conf_thres={self._conf_thres}, '
            f'input={input_image_topic}, out_image={output_image_topic}, '
            f'out_json={detections_topic}, lock_topic={lock_target_topic or "(disabled)"}'
        )

    def _on_image(self, msg: Image) -> None:
        """处理每帧图像。

        输入:
        - msg: ROS 原始图像消息
        输出:
        - `_pub`: 标注后的图像（与输入时间戳保持一致）
        - `_json_pub`: 检测信息 JSON，便于非 ROS 节点消费
        """
        self._last_msg = msg
        self._frames += 1
        img_bgr = self._ros_to_bgr(msg)
        if img_bgr is None:
            # 编码不支持或数据异常时，透传原始图像避免下游断流。
            self._pub.publish(msg)
            return

        results = self.model.predict(img_bgr, conf=self._conf_thres, verbose=False)
        vis = img_bgr.copy()
        detections = []
        if len(results) > 0 and results[0].boxes is not None:
            # Ultralytics 在 results[0].names 提供类别映射: {cls_id: cls_name}
            names = results[0].names if results[0].names is not None else {}
            for box in results[0].boxes:
                # box.xyxy: [x1, y1, x2, y2] (float, 像素坐标)
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                # box.cls / box.conf 是 tensor，取第 0 个目标后转 Python 标量。
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                cls_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
                p1 = (int(x1), int(y1))
                p2 = (int(x2), int(y2))
                cv2.rectangle(vis, p1, p2, (0, 255, 0), 2)
                cv2.putText(
                    vis,
                    f'{cls_name} {conf:.2f}',
                    (p1[0], max(0, p1[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                # 理论上 x2>=x1、y2>=y1；这里加 max 保护，避免异常数据导致负面积。
                area = max(0.0, (x2 - x1)) * max(0.0, (y2 - y1))
                # 提供下游常用几何量，减少重复计算:
                # - xyxy: 原始框坐标
                # - cx/cy: 框中心
                # - area: 框面积（像素^2）
                detections.append(
                    {
                        'cls_id': cls_id,
                        'cls_name': cls_name,
                        'conf': conf,
                        'xyxy': [x1, y1, x2, y2],
                        'cx': cx,
                        'cy': cy,
                        'area': area,
                    }
                )

        if self._lock_active:
            self._draw_lock_overlay(vis)

        self._pub.publish(self._bgr_to_ros(vis, msg))
        # 将 ROS 时间戳统一为秒浮点，方便非 ROS 消费端解析。
        # 注意: 这里是系统时间戳而非帧序号，单位为秒。
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        payload = {
            # header.stamp_sec: 原始图像时间戳（秒）
            'header': {'stamp_sec': stamp},
            # image: 当前图像尺寸信息
            'image': {'width': int(msg.width), 'height': int(msg.height)},
            # detections: 当前帧所有检测框列表（可能为空）
            'detections': detections,
        }
        # ensure_ascii=True 让输出稳定为 ASCII，日志与跨语言链路更稳妥。
        self._json_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

    def _on_lock_target(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            self._lock_active = False
            return

        locked = bool(payload.get('locked', False))
        if not locked:
            self._lock_active = False
            return

        xyxy = payload.get('xyxy', None)
        if not isinstance(xyxy, list) or len(xyxy) != 4:
            self._lock_active = False
            return

        try:
            x1, y1, x2, y2 = [float(v) for v in xyxy]
            cx = payload.get('cx', None)
            cy = payload.get('cy', None)
            if cx is None or cy is None:
                cx = 0.5 * (x1 + x2)
                cy = 0.5 * (y1 + y2)
            self._lock_xyxy = [x1, y1, x2, y2]
            self._lock_cx = float(cx)
            self._lock_cy = float(cy)
            self._lock_score = float(payload.get('conf', payload.get('score', 0.0)))
            self._lock_cls_name = str(payload.get('cls_name', ''))
            self._lock_status = str(payload.get('status', 'pending'))
            self._lock_done_count = int(payload.get('done_count', 0))
            self._lock_target_drop_count = int(payload.get('target_drop_count', 0))
            self._lock_task_done = bool(payload.get('task_done', False))
            self._lock_ex = float(payload.get('ex', 0.0))
            self._lock_ey = float(payload.get('ey', 0.0))
            self._lock_active = True
        except Exception:
            self._lock_active = False

    def _draw_lock_overlay(self, vis: np.ndarray) -> None:
        height, width = vis.shape[:2]
        half_w = max(1.0, width / 2.0)
        half_h = max(1.0, height / 2.0)
        ex = (self._lock_cx - half_w) / half_w
        ey = (self._lock_cy - half_h) / half_h
        locked_text = 'true' if self._lock_active else 'false'
        status_text = self._lock_status
        cv2.putText(
            vis,
            f'ex={ex:.3f}, ey={ey:.3f}, locked={locked_text}, status={status_text}',
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        x1, y1, x2, y2 = self._lock_xyxy
        p1 = (int(x1), int(y1))
        p2 = (int(x2), int(y2))
        center = (int(self._lock_cx), int(self._lock_cy))
        image_center = (int(width / 2), int(height / 2))
        thickness = max(1, int(self._lock_thickness))
        # 已完成目标用不同颜色强调
        color = (255, 0, 255) if self._lock_status == 'done' else self._lock_color

        cv2.rectangle(vis, p1, p2, color, thickness)
        cv2.drawMarker(
            vis,
            center,
            color,
            markerType=cv2.MARKER_CROSS,
            markerSize=16,
            thickness=thickness,
            line_type=cv2.LINE_AA,
        )
        cv2.putText(
            vis,
            'LOCKED' if self._lock_status != 'done' else 'DONE',
            (p1[0], max(0, p1[1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            thickness,
            cv2.LINE_AA,
        )
        cv2.arrowedLine(
            vis,
            image_center,
            center,
            color,
            thickness,
            cv2.LINE_AA,
            tipLength=0.15,
        )
        cxcy_y = min(height - 10, max(20, p2[1] + 20))
        cv2.putText(
            vis,
            f'cx={self._lock_cx:.1f}, cy={self._lock_cy:.1f}',
            (p1[0], cxcy_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            max(1, thickness - 1),
            cv2.LINE_AA,
        )
        if self._lock_target_drop_count > 0:
            done_text = f'done={self._lock_done_count}/{self._lock_target_drop_count}'
            cv2.putText(
                vis,
                done_text,
                (10, min(height - 10, 48)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
        if self._lock_task_done:
            cv2.putText(
                vis,
                'TASK DONE',
                (10, min(height - 10, 72)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

    def _ros_to_bgr(self, msg: Image):
        """将 ROS Image 解包为 OpenCV BGR ndarray；失败返回 None。

        这里显式处理 `bgr8` / `rgb8` 两种编码，并兼容每行有 padding 的图像。
        """
        # 空图或元信息不完整，直接判为无效。
        if msg.width == 0 or msg.height == 0 or msg.step == 0:
            return None

        # Image.data 是字节串；按 uint8 建视图，不复制内存。
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        # ROS Image 语义: data 总字节数应为 height * step。
        expected = msg.height * msg.step
        if raw.size != expected:
            self.get_logger().warning(f'Image size mismatch: data={raw.size}, expected={expected}')
            return None

        row_bytes = msg.step
        # 当前仅支持 3 通道 8bit 图像，因此每行至少 width*3 字节。
        needed = msg.width * 3
        if row_bytes < needed:
            self.get_logger().warning(
                f'Invalid image step for {msg.encoding}: step={row_bytes}, needed={needed}'
            )
            return None

        # 某些驱动会给每行对齐填充，先按 step 还原二维，再裁切有效像素字节。
        img_2d = raw.reshape((msg.height, row_bytes))
        if msg.encoding == 'bgr8':
            # 直接重排为 HWC。
            return np.ascontiguousarray(img_2d[:, :needed].reshape((msg.height, msg.width, 3)))
        if msg.encoding == 'rgb8':
            # 先按 RGB 解包，再转 BGR 供 OpenCV/YOLO 使用。
            rgb = np.ascontiguousarray(img_2d[:, :needed].reshape((msg.height, msg.width, 3)))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        self.get_logger().warning(f'Unsupported image encoding: {msg.encoding}')
        return None

    def _bgr_to_ros(self, img_bgr: np.ndarray, src_msg: Image) -> Image:
        """将 OpenCV BGR 图像封装回 ROS Image，并保留原始 header。

        保留 header 的目的:
        - 让下游可以按原始时间戳做同步
        - 保持 frame_id 等坐标系信息一致
        """
        out = Image()
        out.header = src_msg.header
        out.height = int(img_bgr.shape[0])
        out.width = int(img_bgr.shape[1])
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = np.ascontiguousarray(img_bgr).tobytes()
        return out

    def _on_timer(self) -> None:
        """每秒打印一次输入流状态，用于观测实时帧率与图像格式。

        fps 统计口径:
        - 统计过去 1 秒内触发 `_on_image` 的次数
        - 打印后清零，进入下一个 1 秒窗口
        """
        if self._last_msg is None:
            self.get_logger().info('fps=0 width=0 height=0 encoding=')
            self._frames = 0
            return

        self.get_logger().info(
            f'fps={self._frames} width={self._last_msg.width} '
            f'height={self._last_msg.height} encoding={self._last_msg.encoding}'
        )
        self._frames = 0


def main(args=None) -> None:
    """ROS2 节点入口。"""
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
