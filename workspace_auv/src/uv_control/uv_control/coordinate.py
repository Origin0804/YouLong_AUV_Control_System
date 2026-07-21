"""NED 坐标系 + 3D 齐次变换支持

Coordinate 类表示 NED 坐标系下的 6 自由度位姿 (x, y, z, rx, ry, rz)，
支持通过 4×4 齐次变换矩阵进行坐标系转换（to_local_frame / to_world_frame）。

角度内部以度为单位存储；数学运算时自动转为弧度。
设计参考 AUV2026 CoordinateSystem.py。

──── 快速使用 ────────────────────────────────────────────

# 构造位姿
auv = Coordinate(x=10.0, y=5.0, z=-2.0, rz=45.0)       # NED: (10,5,-2), 朝向45°
pos = Coordinate.from_zit6_array([1.0, 2.0, 0.0, 0.0])  # 从ZIT6协议数组创建

# 坐标系变换
target = Coordinate(x=15.0, y=5.0, z=-2.0)              # 世界坐标下的目标
rel = auv.to_local_frame(target)                         # 目标相对AUV的位姿
abs_ = auv.to_world_frame(Coordinate(x=1.0, y=0.0))     # "AUV前方1米"的世界坐标

# 2D位移（忽略横滚/俯仰）
off = auv.body_to_world(1.0, 0.0)                        # "向右1米" → 世界位移
off = auv.world_to_body(5.0, 0.0)                        # "向北5米" → 机体位移

# 角度归一化
from uv_control.coordinate import wrap_deg, wrap_rad
yaw = wrap_deg(450.0)                                    # → 90.0
"""

import math
from dataclasses import dataclass

PI = math.pi
DEG2RAD = PI / 180.0       # 度 → 弧度
RAD2DEG = 180.0 / PI       # 弧度 → 度


def wrap_deg(angle: float) -> float:
    """将角度归一化到 [-180, 180] 度。"""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def wrap_rad(angle: float) -> float:
    """将弧度归一化到 [-π, π]。"""
    while angle > PI:
        angle -= 2.0 * PI
    while angle < -PI:
        angle += 2.0 * PI
    return angle


def _mat4_mul(a: list, b: list) -> list:
    """两个 4×4 矩阵相乘（扁平成 16 元素的列表）。"""
    r = [0.0] * 16
    for i in range(4):
        for j in range(4):
            r[i * 4 + j] = sum(a[i * 4 + k] * b[k * 4 + j] for k in range(4))
    return r


@dataclass
class Coordinate:
    """NED 6 自由度位姿（内部角度以度为单位）。

    字段：
        x  — 北向位置 (m)
        y  — 东向位置 (m)
        z  — 深度 (m, 向下为正)
        rx — 横滚角 (deg)
        ry — 俯仰角 (deg)
        rz — 偏航角 (deg, 0°=北, 顺时针为正)
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    # ── 矩阵缓存 ──────────────────────────────────────────────
    _h: list = None      # 4×4 齐次变换矩阵（惰性计算）

    # ── 构造器 ────────────────────────────────────────────────

    @classmethod
    def from_zit6_array(cls, data: list):
        """从 ZIT6 协议 Float32MultiArray 创建 Coordinate。

        新格式：[x, y, z, roll_rad, pitch_rad, yaw_rad] (6 元素)
        旧格式：[x, y, z, yaw_rad]                      (4 元素，兼容)
        """
        if len(data) >= 6:
            return cls(x=data[0], y=data[1], z=data[2],
                       rx=math.degrees(data[3]),
                       ry=math.degrees(data[4]),
                       rz=math.degrees(data[5]))
        if len(data) >= 4:
            return cls(x=data[0], y=data[1], z=data[2],
                       rz=math.degrees(data[3]))
        return cls()

    @classmethod
    def from_dict(cls, d: dict):
        """从字典创建 Coordinate，键为 x, y, z, rx, ry, rz（角度单位为度）。"""
        return cls(x=d.get('x', 0.0), y=d.get('y', 0.0),
                   z=d.get('z', 0.0), rz=d.get('rz', 0.0))

    # ── 序列化 ────────────────────────────────────────────────

    def to_dict(self) -> dict:
        """转为字典（角度为度）。"""
        return {'x': self.x, 'y': self.y, 'z': self.z,
                'rx': self.rx, 'ry': self.ry, 'rz': self.rz}

    def to_zit6_array(self) -> list:
        """转为 ZIT6 协议 Float32MultiArray 格式 [x, y, z, yaw_rad]。"""
        return [self.x, self.y, self.z, math.radians(self.rz)]

    # ── 矩阵计算 ──────────────────────────────────────────────

    def to_matrix(self):
        """将当前位姿构建为 4×4 齐次变换矩阵（NED ZYX 旋转约定）。

        矩阵结构：[R(3×3) | t(3×1)]，旋转顺序 Rz(yaw) * Ry(pitch) * Rx(roll)。
        结果缓存在 self._h 中。
        """
        sx, cx = math.sin(self.rx * DEG2RAD), math.cos(self.rx * DEG2RAD)
        sy, cy = math.sin(self.ry * DEG2RAD), math.cos(self.ry * DEG2RAD)
        sz, cz = math.sin(self.rz * DEG2RAD), math.cos(self.rz * DEG2RAD)

        h = [0.0] * 16
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)  （ZYX 外旋 = Rz·Ry·Rx）
        h[0] = cy * cz
        h[1] = sx * sy * cz - cx * sz
        h[2] = cx * sy * cz + sx * sz
        h[3] = self.x

        h[4] = cy * sz
        h[5] = sx * sy * sz + cx * cz
        h[6] = cx * sy * sz - sx * cz
        h[7] = self.y

        h[8] = -sy
        h[9] = sx * cy
        h[10] = cx * cy
        h[11] = self.z

        h[12] = 0.0
        h[13] = 0.0
        h[14] = 0.0
        h[15] = 1.0

        self._h = h
        return h

    def from_matrix(self):
        """从内部 _h 矩阵还原位姿到 x, y, z, rx, ry, rz。

        当 pitch 接近 ±90° 时发生万向锁（gimbal lock），
        此时只提取 yaw，放弃 roll/pitch 的精确值。
        """
        if self._h is None:
            return
        h = self._h
        self.x = h[3]
        self.y = h[7]
        self.z = h[11]

        if abs(h[8]) < 0.999999:
            ry = -math.asin(h[8])
            self.ry = ry * RAD2DEG
            self.rx = math.atan2(h[9] / math.cos(ry), h[10] / math.cos(ry)) * RAD2DEG
            self.rz = math.atan2(h[4] / math.cos(ry), h[0] / math.cos(ry)) * RAD2DEG
        else:
            # 万向锁 — 放弃 roll/pitch，yaw 仍可从 h[0], h[4] 获取
            self.rz = math.atan2(h[4], h[0]) * RAD2DEG

    # ── 坐标系变换 ────────────────────────────────────────────

    def to_local_frame(self, target: 'Coordinate') -> 'Coordinate':
        """计算目标位姿相对于当前坐标系的位姿。

        即：将世界坐标下的 target 转换到以 self 为原点的局部坐标系中。
        等价于：result = inv(self) * target
        """
        self.to_matrix()
        target.to_matrix()
        inv = self._invert_matrix(self._h)
        result = Coordinate()
        result._h = _mat4_mul(inv, target._h)
        result.from_matrix()
        return result

    def to_world_frame(self, local: 'Coordinate') -> 'Coordinate':
        """将当前坐标系下的局部位姿转换为世界坐标。

        即：如果 local 是相对于 self 的位姿，计算它在世界坐标系下的绝对位姿。
        等价于：result = self * local
        """
        self.to_matrix()
        local.to_matrix()
        result = Coordinate()
        result._h = _mat4_mul(self._h, local._h)
        result.from_matrix()
        return result

    # ── 2D 位移快捷变换（忽略横滚/俯仰） ──────────────────────

    def body_to_world(self, dx_b: float, dy_b: float,
                       dz: float = 0.0) -> 'Coordinate':
        """机体坐标系位移 → 世界坐标系位移（仅考虑偏航角 yaw）。

        用于：已知 AUV 当前朝向，求"向右 1 米"在世界坐标下朝哪个方向走。
        """
        yaw_rad = self.rz * DEG2RAD
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        dx_w = cy * dx_b - sy * dy_b
        dy_w = sy * dx_b + cy * dy_b
        return Coordinate(x=dx_w, y=dy_w, z=dz)

    def world_to_body(self, dx_w: float, dy_w: float,
                       dz: float = 0.0) -> 'Coordinate':
        """世界坐标系位移 → 机体坐标系位移（仅考虑偏航角 yaw）。

        用于：已知世界坐标下的目标偏移，求 AUV 自身坐标系下该往哪个方向走。
        """
        yaw_rad = self.rz * DEG2RAD
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        dx_b = cy * dx_w + sy * dy_w
        dy_b = -sy * dx_w + cy * dy_w
        return Coordinate(x=dx_b, y=dy_b, z=dz)

    def offset(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
               drx: float = 0.0, dry: float = 0.0, drz: float = 0.0) -> 'Coordinate':
        """在世界坐标系下偏移后生成新的 Coordinate。

        直接对 x/y/z/rx/ry/rz 做加法，不涉及矩阵运算。
        """
        return Coordinate(x=self.x + dx, y=self.y + dy, z=self.z + dz,
                          rx=self.rx + drx, ry=self.ry + dry, rz=self.rz + drz)

    # ── 工具函数 ──────────────────────────────────────────────

    @staticmethod
    def _invert_matrix(m: list) -> list:
        """Gauss-Jordan 消元法求 4×4 矩阵的逆（扁平成 16 元素列表）。"""
        aug = [0.0] * 32
        for i in range(4):
            for j in range(4):
                aug[i * 8 + j] = m[i * 4 + j]
                aug[i * 8 + j + 4] = 1.0 if i == j else 0.0

        for i in range(4):
            if aug[i * 8 + i] == 0.0:
                row = i + 1
                while row < 4 and aug[row * 8 + i] == 0.0:
                    row += 1
                if row < 4:
                    for j in range(8):
                        aug[i * 8 + j], aug[row * 8 + j] = aug[row * 8 + j], aug[i * 8 + j]
            scale = aug[i * 8 + i]
            for j in range(8):
                aug[i * 8 + j] /= scale
            for j in range(4):
                if j != i:
                    factor = aug[j * 8 + i]
                    for k in range(8):
                        aug[j * 8 + k] -= factor * aug[i * 8 + k]

        inv = [0.0] * 16
        for i in range(4):
            for j in range(4):
                inv[i * 4 + j] = aug[i * 8 + j + 4]
        return inv

    @property
    def yaw_rad(self) -> float:
        """偏航角（弧度）。"""
        return self.rz * DEG2RAD

    @yaw_rad.setter
    def yaw_rad(self, value: float):
        self.rz = value * RAD2DEG

    @property
    def yaw_deg(self) -> float:
        """偏航角（度）。"""
        return self.rz

    @yaw_deg.setter
    def yaw_deg(self, value: float):
        self.rz = value

    def __repr__(self):
        return (f"Coordinate(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, "
                f"rz={self.rz:.1f}°)")
