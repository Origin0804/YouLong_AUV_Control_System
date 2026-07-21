#!/usr/bin/env python3
"""
AUV 可视化监控面板 — PySide6 + ROS 2

显示运行节点、当前位置、2D 地图（物体、射线、标签）、任务状态。

用法:
  source ~/YouLong_AUV_Control_System/workspace_sim/install/setup.bash
  python3 auv_visualizer.py
"""

import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── PySide6 ────────────────────────────────────────────────────
from PySide6.QtCore import QPointF, QRectF, QTimer, Qt
from PySide6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QPainter,
    QPen,
    QPixmap,
)
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QGraphicsItem,
    QGraphicsScene,
    QGraphicsTextItem,
    QGraphicsView,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

# ── ROS 2 ──────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image as RosImage
from uv_msgs.msg import ObjectPositionArray, PoseInfo, TaskStatus
from zit6_interfaces.msg import ZitSetpoint


# ══════════════════════════════════════════════════════════════════
# 线程安全数据容器
# ══════════════════════════════════════════════════════════════════

@dataclass
class PoseSnapshot:
    """当前 AUV 位姿快照（odom 系，角度为度）。"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_yaw: float = 0.0
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_z: float = 0.0
    origin_yaw: float = 0.0
    stamp: float = 0.0  # 最近更新时间


@dataclass
class ObjectSnapshot:
    """单个物体快照。"""
    class_id: int = 0
    instance_id: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    confidence: float = 0.0
    num_obs: int = 0

    @property
    def full_id(self) -> tuple:
        """(class_id, instance_id) 唯一标识。"""
        return (self.class_id, self.instance_id)


@dataclass
class TaskSnapshot:
    """任务状态快照。"""
    status: int = 0  # 0=IDLE, 1=RUNNING, 2=PAUSED, 3=DONE, 4=ERROR
    current_idx: int = 0
    total: int = 0
    name: str = ""
    error: str = ""


@dataclass
class NodeInfo:
    """节点信息。"""
    full_name: str = ""
    namespace: str = ""


# ── 颜色方案 ──────────────────────────────────────────────────
OBJECT_COLORS: Dict[int, QColor] = {
    0: QColor("#F1C40F"),  # yellow_sector
    1: QColor("#E74C3C"),  # red_sector
    2: QColor("#2ECC71"),  # green_sector
    3: QColor("#3498DB"),  # arrow
    4: QColor("#E67E22"),  # start
    5: QColor("#F39C12"),  # triangle
    6: QColor("#9B59B6"),  # square
    7: QColor("#1ABC9C"),  # basket
    8: QColor("#95A5A6"),  # aruco_tag
}
# ID → 名称
CLASS_NAMES: Dict[int, str] = {
    0: "yellow_sector", 1: "red_sector", 2: "green_sector",
    3: "arrow", 4: "start", 5: "triangle",
    6: "square", 7: "basket", 8: "aruco_tag",
}
DEFAULT_OBJECT_COLOR = QColor("#95A5A6")  # gray
AUV_COLOR = QColor("#00DDFF")
TARGET_COLOR = QColor("#F1C40F")
RAY_COLOR = QColor(255, 255, 255, 60)
GRID_COLOR = QColor(255, 255, 255, 20)
GRID_TEXT_COLOR = QColor(255, 255, 255, 100)
BG_COLOR = QColor("#1A1A2E")
PANEL_BG = QColor("#16213E")
TEXT_COLOR = QColor("#ECEFF1")

# 状态名
STATUS_NAMES = {0: "IDLE", 1: "RUNNING", 2: "PAUSED", 3: "DONE", 4: "ERROR"}

# 标注图像显示尺寸
ANNOTATED_DISPLAY_W = 160
ANNOTATED_DISPLAY_H = 120


def _ros_image_to_qpixmap(data: dict) -> QPixmap:
    """ROS Image raw data → QPixmap (BGR8/RGB8 only). 在 GUI 线程调用。"""
    w, h = data["width"], data["height"]
    enc = data.get("encoding", "bgr8")
    raw = data["data"]
    fmt = QImage.Format.Format_BGR888 if "bgr" in enc else QImage.Format.Format_RGB888
    qimg = QImage(raw, w, h, w * 3, fmt)
    if qimg.isNull():
        return QPixmap()
    return QPixmap.fromImage(qimg).scaled(
        ANNOTATED_DISPLAY_W, ANNOTATED_DISPLAY_H,
        Qt.AspectRatioMode.KeepAspectRatio,
        Qt.TransformationMode.SmoothTransformation,
    )

# ══════════════════════════════════════════════════════════════════
# ROS 2 节点（运行在独立线程）
# ══════════════════════════════════════════════════════════════════

class ROS2Node(Node):
    """接收所有话题 + 轮询节点的 ROS 2 节点。"""

    def __init__(self, data_queue: queue.Queue):
        super().__init__("auv_visualizer")
        self._q = data_queue

        # 话题订阅
        qos_sensor = QoSPresetProfiles.SENSOR_DATA.value
        self._sub_pose = self.create_subscription(
            PoseInfo, "/basic_motion/pose_info", self._pose_cb, 10
        )
        self._sub_objects = self.create_subscription(
            ObjectPositionArray, "/perception/objects", self._objects_cb, qos_sensor
        )
        self._sub_task = self.create_subscription(
            TaskStatus, "/task/status", self._task_cb, 10
        )

        # 遥控发布
        self._setpoint_pub = self.create_publisher(ZitSetpoint, "/zit6/cmd/setpoint", 10)
        self._seq = 0

        # 标注图像订阅 (2×2 四个通道)
        for cam in ["front_left", "front_right", "down_left", "down_right"]:
            self.create_subscription(
                RosImage, f"/perception/annotated/{cam}",
                lambda msg, c=cam: self._annotated_cb(c, msg),
                qos_sensor,
            )

        # 节点列表轮询
        self._node_timer = self.create_timer(2.0, self._poll_nodes)
        self._node_poll_count = 0

    def publish_force(self, fx: float, fy: float, fz: float, mz: float):
        """发布 Body-Frame 推力命令（线程安全）。"""
        msg = ZitSetpoint()
        msg.control_key = 0x12       # mode=2(FORCE) | frame=0x10(body)
        msg.type_mask = 0x00         # 所有轴生效
        msg.x = float(fx)
        msg.y = float(fy)
        msg.z = float(fz)
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = float(mz)
        msg.seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        self._setpoint_pub.publish(msg)

    def _pose_cb(self, msg: PoseInfo):
        self._q.put(("pose", {
            "x": msg.robot_x, "y": msg.robot_y, "z": msg.robot_z,
            "roll": msg.robot_roll, "pitch": msg.robot_pitch, "yaw": msg.robot_yaw,
            "target_x": msg.target_x, "target_y": msg.target_y,
            "target_z": msg.target_z, "target_yaw": msg.target_yaw,
            "origin_x": msg.origin_x, "origin_y": msg.origin_y,
            "origin_z": msg.origin_z, "origin_yaw": msg.origin_yaw,
            "stamp": time.time(),
        }))

    def _objects_cb(self, msg: ObjectPositionArray):
        objects = []
        for obj in msg.objects:
            objects.append({
                "class_id": obj.class_id,
                "instance_id": obj.instance_id,
                "x": obj.world_x,
                "y": obj.world_y,
                "z": obj.world_z,
                "confidence": obj.confidence,
                "num_obs": obj.num_observations,
            })
        self._q.put(("objects", objects))

    def _task_cb(self, msg: TaskStatus):
        self._q.put(("task", {
            "status": msg.status,
            "current_idx": msg.current_task_index,
            "total": msg.total_tasks,
            "name": msg.current_task_name,
            "error": msg.error_message,
        }))

    def _annotated_cb(self, cam: str, msg: RosImage):
        """标注图像回调 — 只传原始数据到 GUI 线程解码。"""
        self._q.put(("annotated", {
            "cam": cam,
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
            "data": bytes(msg.data),  # copy 出 bytes，避免 GC 问题
        }))

    def _poll_nodes(self):
        try:
            names = self.get_node_names_and_namespaces()
            nodes = []
            for full, ns in zip(names[0], names[1]):
                # 去掉 _container_ 和启动器节点
                if "_container" in full:
                    continue
                nodes.append({"full_name": full, "namespace": ns})
            self._q.put(("nodes", nodes))
            self._node_poll_count += 1
        except Exception:
            pass  # rclpy 偶尔在退出时抛异常


# ══════════════════════════════════════════════════════════════════
# ROS 2 线程
# ══════════════════════════════════════════════════════════════════

class ROS2Thread(threading.Thread):
    """独立线程运行 rclpy spin_once 循环。"""

    def __init__(self, data_queue: queue.Queue):
        super().__init__(daemon=True)
        self._q = data_queue
        self._running = False
        self._node: Optional[ROS2Node] = None

    def run(self):
        self._running = True
        rclpy.init()
        self._node = ROS2Node(self._q)
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception:
            pass
        finally:
            if self._node is not None:
                self._node.destroy_node()
            rclpy.shutdown()

    def stop(self):
        self._running = False

    def publish_force(self, fx: float, fy: float, fz: float, mz: float):
        """Body-frame force proxy. Thread-safe — delegates to ROS2Node."""
        if self._node is not None:
            self._node.publish_force(fx, fy, fz, mz)


# ══════════════════════════════════════════════════════════════════
# 2D 地图组件
# ══════════════════════════════════════════════════════════════════

class AUVGraphicsItem(QGraphicsItem):
    """AUV 三角形标记，旋转指示 yaw 朝向（NED: 0°=北=上, 顺时针正）。"""

    SIZE = 14.0  # 三角形外接圆半径（像素）

    def __init__(self):
        super().__init__()
        self._yaw_deg = 0.0

    def set_yaw(self, yaw_deg: float):
        """yaw 度，NED 约定（0°=北，顺时针正）。直接用在 Qt 坐标系。"""
        self._yaw_deg = yaw_deg
        self.setRotation(yaw_deg)
        self.update()

    def boundingRect(self):
        r = self.SIZE + 2
        return QRectF(-r, -r, r * 2, r * 2)

    def paint(self, painter: QPainter, option, widget):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        s = self.SIZE
        # 三角形朝上（北向，yaw=0 时）
        painter.setBrush(QBrush(AUV_COLOR))
        painter.setPen(QPen(QColor("#0088AA"), 1.5))
        painter.drawPolygon([
            QPointF(0.0, -s),
            QPointF(-s * 0.6, s * 0.7),
            QPointF(s * 0.6, s * 0.7),
        ])


class ObjectGraphicsItem(QGraphicsItem):
    """物体圆形标记 + ID 标签。"""

    RADIUS = 6.0

    def __init__(self, class_id: int, instance_id: int = 0):
        super().__init__()
        self._class_id = class_id
        self._instance_id = instance_id
        color = OBJECT_COLORS.get(class_id, DEFAULT_OBJECT_COLOR)
        self._brush = QBrush(color)
        self._pen = QPen(color.darker(150), 1.5)

        # 标签: class_id:instance_id + 名称
        name = CLASS_NAMES.get(class_id, f"?")
        self._label = QGraphicsTextItem(self)
        self._label.setDefaultTextColor(color)
        self._label.setFont(QFont("Monospace", 8, QFont.Weight.Bold))
        self._label.setPlainText(f"{class_id}:{instance_id}")
        self._label.setPos(self.RADIUS + 3, -self.RADIUS - 3)

    def boundingRect(self):
        r = self.RADIUS + 2
        return QRectF(-r, -r, r * 2 + 30, r * 2 + 20)

    def paint(self, painter: QPainter, option, widget):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(self._brush)
        painter.setPen(self._pen)
        painter.drawEllipse(-self.RADIUS, -self.RADIUS,
                            self.RADIUS * 2, self.RADIUS * 2)


class TargetGraphicsItem(QGraphicsItem):
    """目标位置标记（十字星）。"""

    SIZE = 8.0

    def boundingRect(self):
        r = self.SIZE + 2
        return QRectF(-r, -r, r * 2, r * 2)

    def paint(self, painter: QPainter, option, widget):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        pen = QPen(TARGET_COLOR, 2.0)
        painter.setPen(pen)
        # 十字
        painter.drawLine(-self.SIZE, 0, self.SIZE, 0)
        painter.drawLine(0, -self.SIZE, 0, self.SIZE)
        # 对角线
        d = self.SIZE * 0.7
        painter.drawLine(-d, -d, d, d)
        painter.drawLine(-d, d, d, -d)


class MapView(QGraphicsView):
    """2D 俯视地图（NED → Qt 坐标：Y 轴向上=北，X 轴向右=东）。"""

    # 缩放范围
    MIN_SCALE = 5.0   # px/m
    MAX_SCALE = 200.0  # px/m
    DEFAULT_SCALE = 50.0

    _grid_pen = QPen(GRID_COLOR, 1.0)
    _grid_text_font = QFont("Monospace", 8)
    _grid_text_color = GRID_TEXT_COLOR

    def __init__(self):
        super().__init__()
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setBackgroundBrush(QBrush(BG_COLOR))
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setViewportUpdateMode(
            QGraphicsView.ViewportUpdateMode.FullViewportUpdate
        )

        # 内层状态
        self._scale = self.DEFAULT_SCALE  # px/m
        self._auto_follow = True
        self._auv_origin_x = 0.0  # 视图中心在场景中的位置（世界米）
        self._auv_origin_y = 0.0

        # 场景元素（持久化，每帧更新属性）
        self._auv_item = AUVGraphicsItem()
        self._scene.addItem(self._auv_item)
        self._target_item = TargetGraphicsItem()
        self._scene.addItem(self._target_item)
        self._target_item.setVisible(False)

        # 动态元素
        self._object_items: Dict[Tuple[int, int], ObjectGraphicsItem] = {}
        self._ray_lines: List[QGraphicsItem] = []
        self._grid_items: List[QGraphicsItem] = []

        # 初始变换
        self._update_transform()

    # ── 公共方法 ───────────────────────────────────────────────

    def update_state(self, pose: PoseSnapshot, objects: List[ObjectSnapshot]):
        """每帧更新：重定位 AUV → 重绘网格 → 重绘物体/射线/目标。"""
        # 1. 更新 AUV 场景坐标
        sx, sy = self._world_to_scene(pose.x, pose.y)
        self._auv_item.setPos(sx, sy)
        self._auv_item.set_yaw(pose.yaw)

        # 2. 更新目标
        if pose.target_x != 0.0 or pose.target_y != 0.0:
            tsx, tsy = self._world_to_scene(pose.target_x, pose.target_y)
            self._target_item.setPos(tsx, tsy)
            self._target_item.setVisible(True)
        else:
            self._target_item.setVisible(False)

        # 3. 自动跟踪
        if self._auto_follow:
            self._auv_origin_x = pose.x
            self._auv_origin_y = pose.y
            self.centerOn(sx, sy)  # 实时居中

        # 4. 重绘网格
        self._redraw_grid()

        # 5. 清除旧射线
        for item in self._ray_lines:
            self._scene.removeItem(item)
        self._ray_lines.clear()

        # 6. 更新物体
        current_ids = set()
        for obj in objects:
            full_id = (obj.class_id, obj.instance_id)
            current_ids.add(full_id)
            osx, osy = self._world_to_scene(obj.x, obj.y)

            # 获取或创建物体 item
            if full_id not in self._object_items:
                item = ObjectGraphicsItem(obj.class_id, obj.instance_id)
                self._scene.addItem(item)
                self._object_items[full_id] = item
            else:
                item = self._object_items[full_id]

            item.setPos(osx, osy)

            # 画射线
            if abs(obj.x - pose.x) > 0.01 or abs(obj.y - pose.y) > 0.01:
                color = OBJECT_COLORS.get(obj.class_id, DEFAULT_OBJECT_COLOR)
                ray_pen = QPen(QColor(color.red(), color.green(), color.blue(), 80), 1.0)
                ray_line = self._scene.addLine(sx, sy, osx, osy, ray_pen)
                ray_line.setZValue(-1)
                self._ray_lines.append(ray_line)

        # 移除不再存在的物体
        for fid in list(self._object_items):
            if fid not in current_ids:
                self._scene.removeItem(self._object_items.pop(fid))

    def set_auto_follow(self, enabled: bool):
        self._auto_follow = enabled

    def zoom_in(self):
        self._scale = min(self._scale * 1.2, self.MAX_SCALE)
        self._update_transform()

    def zoom_out(self):
        self._scale = max(self._scale / 1.2, self.MIN_SCALE)
        self._update_transform()

    def reset_view(self):
        self._scale = self.DEFAULT_SCALE
        self._auv_origin_x = 0.0
        self._auv_origin_y = 0.0
        self._update_transform()

    # ── 坐标变换 ───────────────────────────────────────────────

    def _world_to_scene(self, world_x: float, world_y: float) -> Tuple[float, float]:
        """NED 世界坐标 (x=北, y=东) → Qt 场景坐标（Y上=北, X右=东）。"""
        return world_y * self._scale, -world_x * self._scale

    def _update_transform(self):
        """更新视图，使 (auv_origin_x, auv_origin_y) 居中。"""
        cx, cy = self._world_to_scene(self._auv_origin_x, self._auv_origin_y)
        self.centerOn(cx, cy)

    # ── 网格绘制 ───────────────────────────────────────────────

    def _redraw_grid(self):
        # 清除旧网格
        for item in self._grid_items:
            self._scene.removeItem(item)
        self._grid_items.clear()

        # 计算可见范围（世界米）
        view_w = self.viewport().width() if self.viewport() else 600
        view_h = self.viewport().height() if self.viewport() else 400
        w_m = view_w / self._scale
        h_m = view_h / self._scale

        # 网格间距
        grid_spacing = 1.0
        if w_m > 40:
            grid_spacing = 10.0
        elif w_m > 15:
            grid_spacing = 5.0
        elif w_m > 8:
            grid_spacing = 2.0

        # 世界范围
        x_min = self._auv_origin_x - h_m / 2
        x_max = self._auv_origin_x + h_m / 2
        y_min = self._auv_origin_y - w_m / 2
        y_max = self._auv_origin_y + w_m / 2

        # 对齐到网格
        gx0 = math.floor(x_min / grid_spacing) * grid_spacing
        gy0 = math.floor(y_min / grid_spacing) * grid_spacing

        # 垂直线（等 east = gy）：从 (x_min, gy) 到 (x_max, gy)
        gy = gy0
        while gy <= y_max:
            s1x, s1y = self._world_to_scene(x_min, gy)
            s2x, s2y = self._world_to_scene(x_max, gy)
            line = self._scene.addLine(s1x, s1y, s2x, s2y, self._grid_pen)
            line.setZValue(-2)
            self._grid_items.append(line)

            # 标签放在底部
            text = self._scene.addText(f"{gy:.0f}", self._grid_text_font)
            text.setDefaultTextColor(self._grid_text_color)
            text.setPos(s1x + 3, s2y - 18)
            text.setZValue(-1)
            self._grid_items.append(text)
            gy += grid_spacing

        # 水平线（等 north = gx）：从 (gx, y_min) 到 (gx, y_max)
        gx = gx0
        while gx <= x_max:
            s1x, s1y = self._world_to_scene(gx, y_min)
            s2x, s2y = self._world_to_scene(gx, y_max)
            line = self._scene.addLine(s1x, s1y, s2x, s2y, self._grid_pen)
            line.setZValue(-2)
            self._grid_items.append(line)

            # 标签放在左侧
            text = self._scene.addText(f"{gx:.0f}", self._grid_text_font)
            text.setDefaultTextColor(self._grid_text_color)
            text.setPos(s2x + 3, s2y)
            text.setZValue(-1)
            self._grid_items.append(text)
            gx += grid_spacing

    # ── 事件 ───────────────────────────────────────────────────

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.zoom_in()
        else:
            self.zoom_out()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_transform()


# ══════════════════════════════════════════════════════════════════
# 主窗口
# ══════════════════════════════════════════════════════════════════

# ══════════════════════════════════════════════════════════════════
# 遥控面板
# ══════════════════════════════════════════════════════════════════

class ControlPanel(QWidget):
    """Body-frame 推力遥控按钮面板。"""

    # 按钮 → (Fx, Fy, Fz, Mz) body-force 映射
    _FORCE_MAP = {
        "Fwd":   ( 1,  0,  0,  0),   # +Fx 前进
        "Back":  (-1,  0,  0,  0),   # -Fx 后退
        "Left":  ( 0, -1,  0,  0),   # -Fy 左移
        "Right": ( 0,  1,  0,  0),   # +Fy 右移
        "Asc":   ( 0,  0, -1,  0),   # -Fz 上浮 (NED up)
        "Desc":  ( 0,  0,  1,  0),   # +Fz 下潜 (NED down)
        "Yaw-L": ( 0,  0,  0,  1),   # +Mz 顺时针
        "Yaw-R": ( 0,  0,  0, -1),   # -Mz 逆时针
    }

    def __init__(self, publish_cb):
        """
        Args:
            publish_cb: callable(fx, fy, fz, mz) — 线程安全的发布函数。
        """
        super().__init__()
        self._publish = publish_cb
        self._force_magnitude = 100.0  # 默认力大小 (N/Nm)
        self._active_button: Optional[str] = None
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # 力大小设置
        mag_layout = QHBoxLayout()
        mag_layout.addWidget(QLabel("力大小:"))
        self._spin_force = QSpinBox()
        self._spin_force.setRange(10, 500)
        self._spin_force.setValue(100)
        self._spin_force.setSuffix(" N")
        self._spin_force.valueChanged.connect(self._on_magnitude_changed)
        self._spin_force.setStyleSheet("""
            QSpinBox {
                background-color: #1A1A2E; color: #00DDFF;
                border: 1px solid #1565C0; padding: 4px;
                font-size: 13px; font-weight: bold;
            }
        """)
        mag_layout.addWidget(self._spin_force)
        mag_layout.addStretch()
        layout.addLayout(mag_layout)

        layout.addSpacing(8)

        # 方向按钮（十字布局 + 垂直/旋转）
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        buttons = [
            # (name, row, col, rowspan, colspan)
            ("Fwd",   0, 1, 1, 1),
            ("Left",  1, 0, 1, 1),
            ("Right", 1, 2, 1, 1),
            ("Back",  2, 1, 1, 1),
            ("Asc",   3, 0, 1, 1),
            ("Desc",  3, 2, 1, 1),
            ("Yaw-L", 4, 0, 1, 1),
            ("Yaw-R", 4, 2, 1, 1),
        ]

        self._buttons: Dict[str, QPushButton] = {}
        for name, r, c, rs, cs in buttons:
            btn = QPushButton(name)
            btn.setMinimumHeight(44)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            # lambda default arg captures current name value
            btn.pressed.connect(lambda n=name: self._on_press(n))
            btn.setAutoRepeat(True)
            btn.setAutoRepeatDelay(200)
            btn.setAutoRepeatInterval(100)
            btn.released.connect(self._on_release)
            self._buttons[name] = btn
            grid.addWidget(btn, r, c, rs, cs)

        layout.addLayout(grid)

    def _on_magnitude_changed(self, val: int):
        self._force_magnitude = float(val)

    def _on_press(self, name: str):
        """按钮按下：发送对应力。"""
        self._active_button = name
        coeffs = self._FORCE_MAP[name]
        mag = self._force_magnitude
        self._publish(
            coeffs[0] * mag,  # fx
            coeffs[1] * mag,  # fy
            coeffs[2] * mag,  # fz
            coeffs[3] * mag,  # mz
        )

    def _on_release(self):
        """按钮释放：发送零力。"""
        self._active_button = None
        self._publish(0.0, 0.0, 0.0, 0.0)


class VisualizerGUI(QMainWindow):
    """AUV 可视化主窗口。"""

    def __init__(self, data_queue: queue.Queue, ros_thread: 'ROS2Thread'):
        super().__init__()
        self._q = data_queue
        self._ros_thread = ros_thread
        self.setWindowTitle("AUV 可视化面板 — YouLong")
        self.resize(1400, 850)

        # 数据快照
        self._pose = PoseSnapshot()
        self._objects: List[ObjectSnapshot] = []
        self._task = TaskSnapshot()
        self._nodes: List[NodeInfo] = []
        self._annotated: Dict[str, QPixmap] = {}  # cam_name → pixmap

        # 频率计数
        self._pose_count = 0
        self._obj_count = 0
        self._task_count = 0
        self._last_rate_time = time.time()
        self._pose_rate = 0.0
        self._obj_rate = 0.0
        self._task_rate = 0.0

        # UI
        self._setup_ui()

        # 定时器 — 50ms (20Hz GUI 刷新)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll_ros_data)
        self._timer.start(50)

    # ── UI 构建 ────────────────────────────────────────────────

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        central.setStyleSheet(f"""
            QWidget {{
                background-color: {PANEL_BG.name()};
                color: {TEXT_COLOR.name()};
                font-family: Monospace;
                font-size: 12px;
            }}
            QGroupBox {{
                border: 1px solid {GRID_TEXT_COLOR.name()};
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 12px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: {AUV_COLOR.name()};
            }}
            QHeaderView::section {{
                background-color: {BG_COLOR.name()};
                color: {AUV_COLOR.name()};
                border: 1px solid {GRID_TEXT_COLOR.name()};
                padding: 4px;
            }}
            QTableWidget, QTreeWidget {{
                background-color: {BG_COLOR.name()};
                alternate-background-color: #1E2D4A;
                border: 1px solid {GRID_TEXT_COLOR.name()};
                gridline-color: {GRID_TEXT_COLOR.name()};
            }}
            QTableWidget::item, QTreeWidget::item {{
                padding: 2px;
            }}
            QPushButton {{
                background-color: #0D47A1;
                border: 1px solid #1565C0;
                border-radius: 3px;
                padding: 5px 12px;
                color: {TEXT_COLOR.name()};
            }}
            QPushButton:hover {{
                background-color: #1565C0;
            }}
            QPushButton:pressed {{
                background-color: #0A3D8F;
            }}
            QCheckBox {{
                spacing: 6px;
            }}
            QCheckBox::indicator {{
                width: 16px;
                height: 16px;
            }}
        """)

        # 顶层水平分割
        hsplit = QSplitter(Qt.Orientation.Horizontal)

        # 最左：2×2 标注图像
        hsplit.addWidget(self._create_camera_panel())

        # 左侧面板：节点列表 + 位置信息
        left_widget = self._create_left_panel()
        hsplit.addWidget(left_widget)

        # 中间面板：地图
        self._map = MapView()
        hsplit.addWidget(self._map)

        # 右侧面板：物体列表 + 任务状态
        right_widget = self._create_right_panel()
        hsplit.addWidget(right_widget)

        # 比例
        hsplit.setStretchFactor(0, 1)  # camera
        hsplit.setStretchFactor(1, 1)  # left
        hsplit.setStretchFactor(2, 3)  # map
        hsplit.setStretchFactor(3, 1)  # right
        hsplit.setSizes([340, 240, 700, 280])

        layout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(hsplit)

        # 工具栏
        self._create_toolbar(layout)

        # 状态栏
        self._status_label = QLabel("等待数据...")
        self._status_label.setStyleSheet("color: #888;")
        layout.addWidget(self._status_label)

    def _create_toolbar(self, parent_layout):
        toolbar = QHBoxLayout()

        self._cb_follow = QCheckBox("自动跟随 AUV")
        self._cb_follow.setChecked(True)
        self._cb_follow.toggled.connect(self._map.set_auto_follow)
        toolbar.addWidget(self._cb_follow)

        toolbar.addStretch()

        btn_reset = QPushButton("重置视图")
        btn_reset.clicked.connect(self._map.reset_view)
        toolbar.addWidget(btn_reset)

        btn_zoom_in = QPushButton("+")
        btn_zoom_in.setFixedWidth(40)
        btn_zoom_in.clicked.connect(self._map.zoom_in)
        toolbar.addWidget(btn_zoom_in)

        btn_zoom_out = QPushButton("-")
        btn_zoom_out.setFixedWidth(40)
        btn_zoom_out.clicked.connect(self._map.zoom_out)
        toolbar.addWidget(btn_zoom_out)

        parent_layout.addLayout(toolbar)

    def _create_left_panel(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # 节点列表
        grp_nodes = QGroupBox("运行节点")
        node_layout = QVBoxLayout(grp_nodes)
        self._node_tree = QTreeWidget()
        self._node_tree.setHeaderLabels(["节点名", "命名空间"])
        self._node_tree.setAlternatingRowColors(True)
        self._node_tree.header().setStretchLastSection(True)
        # 紧凑列
        for i in range(self._node_tree.columnCount()):
            self._node_tree.resizeColumnToContents(i)
        node_layout.addWidget(self._node_tree)
        layout.addWidget(grp_nodes)

        # 位置信息
        grp_pos = QGroupBox("当前位置 (odom 系)")
        pos_layout = QVBoxLayout(grp_pos)
        self._pos_labels: Dict[str, QLabel] = {}
        for key, label in [("x", "X (北)"), ("y", "Y (东)"), ("z", "Z (深)"),
                           ("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")]:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"{label}:"))
            lbl = QLabel("--")
            lbl.setStyleSheet(f"color: {AUV_COLOR.name()}; font-weight: bold;")
            self._pos_labels[key] = lbl
            row.addWidget(lbl)
            row.addStretch()
            pos_layout.addLayout(row)

        # 目标
        row = QHBoxLayout()
        row.addWidget(QLabel("目标:"))
        self._target_pos_label = QLabel("--")
        self._target_pos_label.setStyleSheet(f"color: {TARGET_COLOR.name()};")
        row.addWidget(self._target_pos_label)
        row.addStretch()
        pos_layout.addLayout(row)

        pos_layout.addStretch()
        layout.addWidget(grp_pos)

        return widget

    def _create_right_panel(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # 遥控面板
        grp_ctrl = QGroupBox("遥控 (Body Force)")
        ctrl_layout = QVBoxLayout(grp_ctrl)
        self._control_panel = ControlPanel(self._ros_thread.publish_force)
        ctrl_layout.addWidget(self._control_panel)
        layout.addWidget(grp_ctrl)

        # 物体列表
        grp_obj = QGroupBox("检测物体")
        obj_layout = QVBoxLayout(grp_obj)
        self._obj_table = QTableWidget(0, 7)
        self._obj_table.setHorizontalHeaderLabels([
            "C:I", "名称", "X(北)", "Y(东)", "Z(深)", "置信度", "#Obs"
        ])
        self._obj_table.setAlternatingRowColors(True)
        self._obj_table.horizontalHeader().setStretchLastSection(True)
        self._obj_table.verticalHeader().setVisible(False)
        obj_layout.addWidget(self._obj_table)
        layout.addWidget(grp_obj)

        # 任务状态
        grp_task = QGroupBox("任务状态")
        task_layout = QVBoxLayout(grp_task)

        self._task_status_label = QLabel("状态: --")
        self._task_status_label.setStyleSheet(f"color: {AUV_COLOR.name()}; font-size: 14px;")
        task_layout.addWidget(self._task_status_label)

        self._task_name_label = QLabel("任务: --")
        task_layout.addWidget(self._task_name_label)

        self._task_progress_label = QLabel("进度: --")
        task_layout.addWidget(self._task_progress_label)

        self._task_error_label = QLabel("")
        self._task_error_label.setStyleSheet("color: #E74C3C;")
        self._task_error_label.setWordWrap(True)
        task_layout.addWidget(self._task_error_label)

        task_layout.addStretch()
        layout.addWidget(grp_task)

        return widget

    def _create_camera_panel(self) -> QWidget:
        """4 通道标注图像，纵向排列在左侧。"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self._cam_labels: Dict[str, QLabel] = {}
        for cam in ["front_left", "front_right", "down_left", "down_right"]:
            lbl_title = QLabel(cam)
            lbl_title.setStyleSheet(f"color: {AUV_COLOR.name()}; font-size: 10px;")
            lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl_img = QLabel("等待…")
            lbl_img.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl_img.setMinimumSize(ANNOTATED_DISPLAY_W, ANNOTATED_DISPLAY_H)
            lbl_img.setStyleSheet(f"""
                background-color: {BG_COLOR.name()};
                border: 1px solid {GRID_TEXT_COLOR.name()};
                color: #555; font-size: 10px;
            """)
            self._cam_labels[cam] = lbl_img
            layout.addWidget(lbl_title)
            layout.addWidget(lbl_img)

        layout.addStretch()
        return widget

    def _poll_ros_data(self):
        """从队列取出所有 ROS 数据，更新本地快照。"""
        while True:
            try:
                msg_type, data = self._q.get_nowait()
            except queue.Empty:
                break

            if msg_type == "pose":
                self._pose = PoseSnapshot(**data)
                self._pose_count += 1
            elif msg_type == "objects":
                self._objects = [ObjectSnapshot(**o) for o in data]
                self._obj_count += 1
            elif msg_type == "task":
                self._task = TaskSnapshot(**data)
                self._task_count += 1
            elif msg_type == "annotated":
                pix = _ros_image_to_qpixmap(data)
                if not pix.isNull():
                    self._annotated[data["cam"]] = pix
            elif msg_type == "nodes":
                self._nodes = [NodeInfo(**n) for n in data]

        # 频率统计（每秒更新）
        now = time.time()
        dt = now - self._last_rate_time
        if dt >= 1.0:
            self._pose_rate = self._pose_count / dt
            self._obj_rate = self._obj_count / dt
            self._task_rate = self._task_count / dt
            self._pose_count = 0
            self._obj_count = 0
            self._task_count = 0
            self._last_rate_time = now

        # 更新 UI
        self._update_left_panel()
        self._update_right_panel()
        self._update_camera_panel()
        self._map.update_state(self._pose, self._objects)
        self._update_status_bar()

    def _update_left_panel(self):
        # 节点列表
        current_names = set()
        for i in range(self._node_tree.topLevelItemCount()):
            current_names.add(self._node_tree.topLevelItem(i).text(0))

        new_names = set()
        for node in self._nodes:
            name = node.full_name
            new_names.add(name)
            if name not in current_names:
                item = QTreeWidgetItem([name, node.namespace])
                self._node_tree.addTopLevelItem(item)

        # 移除下线节点
        for i in range(self._node_tree.topLevelItemCount() - 1, -1, -1):
            if self._node_tree.topLevelItem(i).text(0) not in new_names:
                self._node_tree.takeTopLevelItem(i)

        # 位置数值
        self._pos_labels["x"].setText(f"{self._pose.x:.2f} m")
        self._pos_labels["y"].setText(f"{self._pose.y:.2f} m")
        self._pos_labels["z"].setText(f"{self._pose.z:.2f} m")
        self._pos_labels["roll"].setText(f"{self._pose.roll:.1f}°")
        self._pos_labels["pitch"].setText(f"{self._pose.pitch:.1f}°")
        self._pos_labels["yaw"].setText(f"{self._pose.yaw:.1f}°")

        # 目标
        if self._pose.target_x != 0 or self._pose.target_y != 0:
            self._target_pos_label.setText(
                f"({self._pose.target_x:.2f}, {self._pose.target_y:.2f}, "
                f"z={self._pose.target_z:.2f}, yaw={self._pose.target_yaw:.1f}°)"
            )
        else:
            self._target_pos_label.setText("-- (无活动目标)")

    def _update_right_panel(self):
        # 物体列表
        self._obj_table.setRowCount(len(self._objects))
        for row, obj in enumerate(self._objects):
            color = OBJECT_COLORS.get(obj.class_id, DEFAULT_OBJECT_COLOR)
            name = CLASS_NAMES.get(obj.class_id, f"?")
            id_item = QTableWidgetItem(f"{obj.class_id}:{obj.instance_id}")
            id_item.setForeground(color)
            id_item.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
            self._obj_table.setItem(row, 0, id_item)
            name_item = QTableWidgetItem(name)
            name_item.setForeground(color)
            self._obj_table.setItem(row, 1, name_item)
            for col, val in enumerate([f"{obj.x:.2f}", f"{obj.y:.2f}",
                                        f"{obj.z:.2f}", f"{obj.confidence:.2f}",
                                        str(obj.num_obs)], 2):
                item = QTableWidgetItem(val)
                item.setForeground(TEXT_COLOR)
                self._obj_table.setItem(row, col, item)

        # 任务状态
        s = STATUS_NAMES.get(self._task.status, "?")
        color_map = {
            0: "#888",           # IDLE - gray
            1: AUV_COLOR.name(),  # RUNNING - cyan
            2: "#F39C12",        # PAUSED - orange
            3: "#2ECC71",        # DONE - green
            4: "#E74C3C",        # ERROR - red
        }
        self._task_status_label.setText(f"状态: {s}")
        self._task_status_label.setStyleSheet(
            f"color: {color_map.get(self._task.status, '#888')}; font-size: 14px;"
        )
        self._task_name_label.setText(f"任务: {self._task.name or '--'}")
        self._task_progress_label.setText(
            f"进度: {self._task.current_idx}/{self._task.total}"
        )
        self._task_error_label.setText(self._task.error or "")

    def _update_camera_panel(self):
        """刷新四个通道的标注图像。"""
        for cam, lbl in self._cam_labels.items():
            if cam in self._annotated:
                lbl.setPixmap(self._annotated[cam])
            # else: keep previous image or placeholder

    def _update_status_bar(self):
        self._status_label.setText(
            f"Pose: {self._pose_rate:.1f}Hz | Objects: {self._obj_rate:.1f}Hz | "
            f"Task: {self._task_rate:.1f}Hz | Nodes: {len(self._nodes)} | "
            f"Scale: {self._map._scale:.0f} px/m"
        )

    # ── 清理 ───────────────────────────────────────────────────

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)


# ══════════════════════════════════════════════════════════════════
# 入口
# ══════════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # 暗色主题调色板
    palette = app.palette()
    palette.setColor(palette.ColorRole.Window, PANEL_BG)
    palette.setColor(palette.ColorRole.WindowText, TEXT_COLOR)
    palette.setColor(palette.ColorRole.Base, BG_COLOR)
    palette.setColor(palette.ColorRole.Text, TEXT_COLOR)
    palette.setColor(palette.ColorRole.Button, QColor("#0D47A1"))
    palette.setColor(palette.ColorRole.ButtonText, TEXT_COLOR)
    app.setPalette(palette)

    data_queue: queue.Queue = queue.Queue()

    # 启动 ROS 2 线程
    ros_thread = ROS2Thread(data_queue)
    ros_thread.start()

    # 创建 GUI
    window = VisualizerGUI(data_queue, ros_thread)
    window.show()

    # 运行
    exit_code = app.exec()

    # 清理
    ros_thread.stop()
    ros_thread.join(timeout=3.0)

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
