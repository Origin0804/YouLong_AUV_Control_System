"""
basic_motion.py — 运动控制节点（合并 ZIT6 底层 + 高级运动 API）

分层架构（从底到顶）：

  Layer 5: 对外高级 API (SET / WMOVE / BMOVE / TRAVEL)
  Layer 4: 步进坐标系 (运动方向 along/lateral 分解 + 动态步长)
  Layer 3: 机器人坐标系 (body frame — set_body / set_step)
  Layer 2: 世界坐标系 (odom — start / set_world / get_state)
  Layer 1: ZIT6 底层 (map 坐标系 — _send_setpoint)

坐标系说明：
  map 系 — MCU/仿真器上报的原始位置，原点未知
  odom 系 — 以 AUV 启动位置为原点的世界坐标系（start() 时初始化）
  body 系 — 以 AUV 当前位置为原点的机体坐标系

指令路径（保证 single source of truth）：
  高级 API → set_world → set_map → _send_setpoint → /zit6/cmd/setpoint
  set_body/set_step → Coordinate 变换 → set_world → …
  set_map 是唯一的协议出口，迁移协议只需改此函数

================================================================================
系统架构
================================================================================

  ┌─────────────────────────────────────────────────────┐
  │  uv_task (任务层)                                    │
  │  JSON 任务加载 → 顺序执行 → 调用导航/运动              │
  ├─────────────────────────────────────────────────────┤
  │  uv_nav (导航层)                                     │
  │  A* 路径规划 → 避障 → 调用 basic_motion 执行           │
  ├─────────────────────────────────────────────────────┤
  │  uv_control (控制层)  ← 本文件所在层                   │
  │  basic_motion (本节点)                                │
  │    → set_world / set_body / set_step                 │
  │    → _send_setpoint → /zit6/cmd/setpoint             │
  ├─────────────────────────────────────────────────────┤
  │  uv_hm (硬件管理层)                                   │
  │  sim_bridge (仿真) / hw_manager (实车)                │
  │  级联PID + 推力混合 → 6推进器                          │
  ├─────────────────────────────────────────────────────┤
  │  Stonefish / MCU                                     │
  │  物理仿真引擎 / STM32 微控制器                         │
  └─────────────────────────────────────────────────────┘

================================================================================
三类运动函数
================================================================================

1. SET 系列 (绝对定位)
   - 直接发送 odom 系绝对坐标 + 等到达
   - 适用于已知精确目标位置的场景

2. WMOVE / BMOVE 系列 (步进移动)
   - WMOVE: 世界系步进，参数为世界系偏移量
   - BMOVE: 机体系步进，参数为机体系偏移量，内部转世界系
   - 使用动态步进算法：把长距离拆成小段逐段发送
   - 步长根据当前误差动态调整

3. TRAVEL 系列 (直线移动)
   - WTRAVEL: 世界系直线移动
   - BTRAVEL: 机体系直线移动
   - 先转向目标方向，再沿 body-X 轴前进
   - 适用于需要直线轨迹的任务（过门、巡线等）
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from uv_msgs.action import BasicMotion
from uv_msgs.msg import PoseInfo
from uv_control.coordinate import Coordinate, wrap_deg, wrap_rad

# ═════════════════════════════════════════════════════════════════════════════
# ZIT6 control_key 常量
# ═════════════════════════════════════════════════════════════════════════════
# 只走 0x00 位置模式，body/增量转换在上层完成
CK_POS = 0          # position mode (唯一的 control_key)

# ═════════════════════════════════════════════════════════════════════════════
# 轴掩码
# ═════════════════════════════════════════════════════════════════════════════
AX_X = 0x01
AX_Y = 0x02
AX_Z = 0x04
AX_RZ = 0x08
AX_XY = AX_X | AX_Y
AX_XYZ = AX_X | AX_Y | AX_Z
AX_ALL = AX_X | AX_Y | AX_Z | AX_RZ

# ═════════════════════════════════════════════════════════════════════════════
# 默认容差
# ═════════════════════════════════════════════════════════════════════════════
TOL_X = 0.1     # X 轴容差 (米)
TOL_Y = 0.1     # Y 轴容差 (米)
TOL_Z = 0.1     # Z 轴容差 (米)
TOL_RZ = 5.0    # Yaw 轴容差 (度)

# ═════════════════════════════════════════════════════════════════════════════
# 步进控制参数
# ═════════════════════════════════════════════════════════════════════════════
STEP_X = 0.6             # X 方向步长 (椭圆半轴，米)
STEP_Y = 0.2             # Y 方向步长 (椭圆半轴，米)
STEP_PERIOD = 0.2        # 目标步进间隔/收敛时间阈值 (秒)
LATERAL_LAMBDA = 2.0     # 横向误差指数衰减系数


class BasicMotionNode(Node):
    """运动控制节点：ZIT6 协议接口 + 高级运动 API，single source of truth。"""

    def __init__(self):
        super().__init__('basic_motion')

        # ── 状态 ───────────────────────────────────────────────
        self.status = ZitStatus()
        self.pose = Coordinate()        # 当前位置 (odom 系)
        self._target = Coordinate()     # 当前目标 (odom 系)
        self._origin = None             # odom 原点 (map 系 Coordinate)，start() 时设置
        self._origin_warned = False     # 防止 _pos_cb 重复打印警告
        self._state_lock = threading.Lock()
        self.vel_body = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}   # 机体速度                     # 世界速度
        self._pose_stamp = self.get_clock().now().to_msg()            # 位姿测量时间（_pos_cb 接收时刻）

        # ─────────────────────────────────────────────────────────
        # Layer 1: ZIT6 协议发布/订阅
        # ─────────────────────────────────────────────────────────
        self.pub_setpoint = self.create_publisher(
            ZitSetpoint, '/zit6/cmd/setpoint', 10)
        self.create_subscription(
            ZitStatus, '/zit6/state/status', self._status_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/pos', self._pos_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/zit6/state/vel', self._vel_cb, 10)

        # ── Action Server ────────────────────────────────────────
        self._action_server = ActionServer(
            self, BasicMotion, 'basic_motion',
            goal_callback=self._action_goal_cb,
            cancel_callback=self._action_cancel_cb,
            execute_callback=self._action_execute_cb,
        )
        self._action_goal_handle = None
        self._action_target = None       # 绝对目标 {x, y, z, yaw}，用于反馈
        self._action_axes = None         # 当前 action 的生效轴，用于反馈
        self.create_timer(0.5, self._action_feedback_cb)

        # ── PoseInfo Publisher ─────────────────────────────────
        self.pub_pose = self.create_publisher(PoseInfo, '/basic_motion/pose_info', 10)
        self.create_timer(1.0 / 30.0, self._publish_pose_info)

        self.get_logger().info('BasicMotion node started')

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 1: ZIT6 底层 (map 坐标系)
    # ═════════════════════════════════════════════════════════════════════════

    def _send_setpoint(self, control_key: int, type_mask: int,
                       x: float, y: float, z: float, yaw_rad: float):
        """发送 ZitSetpoint。坐标是 map 系，yaw 是弧度，只走 CK_POS 位置模式。"""
        msg = ZitSetpoint()
        msg.control_key = control_key
        msg.type_mask = type_mask
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw_rad)
        msg.roll = 0.0         # roll/pitch 未使用，控制栈保持 4-DOF
        msg.pitch = 0.0
        msg.seq = 0
        self._target = self._map_to_odom(Coordinate(x=x, y=y, z=z, rz=math.degrees(yaw_rad)))
        self.pub_setpoint.publish(msg)
        odom_t = self._target
        self.get_logger().info(
            f'发往ZIT6: map=({x:.2f}, {y:.2f}, {z:.2f}, {math.degrees(yaw_rad):.1f}°), '
            f'对应odom=({odom_t.x:.2f}, {odom_t.y:.2f}, {odom_t.z:.2f}, {odom_t.rz:.1f}°)')

    def _status_cb(self, msg: ZitStatus):
        with self._state_lock:
            self.status = msg

    def _vel_cb(self, msg: Float32MultiArray):
        """ZIT6 速度回调。机体速度 6-DOF [vx, vy, vz, vroll_rad, vpitch_rad, vyaw_rad_s]。"""
        with self._state_lock:
            if len(msg.data) >= 6:
                self.vel_body = {
                    'x': msg.data[0], 'y': msg.data[1], 'z': msg.data[2],
                    'rx': msg.data[3], 'ry': msg.data[4], 'rz': msg.data[5],
                }
            elif len(msg.data) >= 4:
                self.vel_body = {
                    'x': msg.data[0], 'y': msg.data[1],
                    'z': msg.data[2], 'rz': msg.data[3],
                }

    def _pos_cb(self, msg: Float32MultiArray):
        """ZIT6 位置回调。原始数据是 map 系，内部转 odom 系后存为 self.pose。

        支持 6 元素 [x, y, z, roll_rad, pitch_rad, yaw_rad] 和
        旧 4 元素 [x, y, z, yaw_rad] 两种格式。
        """
        with self._state_lock:
            if len(msg.data) < 4:
                return
            self._pose_stamp = self.get_clock().now().to_msg()  # 记录接收时刻作为测量时间
            if len(msg.data) >= 6:
                map_pos = Coordinate(
                    x=msg.data[0], y=msg.data[1], z=msg.data[2],
                    rx=math.degrees(msg.data[3]),   # roll rad
                    ry=math.degrees(msg.data[4]),   # pitch rad
                    rz=math.degrees(msg.data[5]),   # yaw rad
                )
            else:
                map_pos = Coordinate(
                    x=msg.data[0], y=msg.data[1], z=msg.data[2],
                    rz=math.degrees(msg.data[3]),   # 弧度→度
                )
            if self._origin is not None:
                self.pose = self._map_to_odom(map_pos)
            else:
                if not self._origin_warned:
                    self._origin_warned = True
                    self.get_logger().warning('里程计原点未设置，使用 map 坐标作为 odom 坐标')
                self.pose = map_pos


    def set_map(self, x: float, y: float, z: float, yaw_deg: float):
        """直接发送 map 系绝对位置。yaw 单位为度。

        所有 ZIT6 协议细节集中于此（CK_POS, type_mask=0）。
        迁移到其他协议时只需修改此函数。
        """
        self._send_setpoint(CK_POS, 0, x, y, z, math.radians(yaw_deg))


    # ═════════════════════════════════════════════════════════════════════════
    # Layer 2: 世界坐标系 (odom)
    # ═════════════════════════════════════════════════════════════════════════

    def start(self):
        """初始化 odom 原点。AUV 开始作业前第一个调用。

        记录当前 map 位置为 odom 原点，之后所有坐标都是相对于此原点。
        odom 0° = AUV 初始朝向（yaw 也做偏移）。
        例：AUV 朝东 (map 90°) start → odom 0° = 东，set_world(x=5) 向东走 5m。
        """
        with self._state_lock:
            if self._origin is not None:
                self.get_logger().info('odom origin already set. update it to current pose. ')
                self.get_logger().warning('It might be an incorrect motion.')
                
            self._origin = Coordinate(
                x=self.pose.x, y=self.pose.y, z=0.0, rz=self.pose.rz)
            self.get_logger().info(
                f'DEBUG pose before zero: x={self.pose.x:.4f}, y={self.pose.y:.4f}, '
                f'z={self.pose.z:.4f}, rz={self.pose.rz:.4f}')
            # 将当前 pose 全部置零成为 odom 原点
            self.pose.x = 0.0
            self.pose.y = 0.0
            self.pose.z = 0.0
            self.pose.rz = 0.0
        self.get_logger().info(
            f'odom origin set: map({self._origin.x:.2f}, '
            f'{self._origin.y:.2f}, {self._origin.z:.2f}), '
            f'yaw={self._origin.rz:.1f}°')

    def _odom_to_map(self, pos: Coordinate) -> Coordinate:
        """odom 坐标 → map 坐标（x/y/z/rz 全做偏移）。

        Returns: (map_x, map_y, map_z, map_yaw_deg)
        """
        if self._origin is None:
            return pos
        return self._origin.to_world_frame(pos)
    def _map_to_odom(self, pos: Coordinate) -> Coordinate:
        """map 坐标字典 → odom 坐标字典（x/y/z/rz 全做偏移）。"""
        if self._origin is None:
            return pos
        return self._origin.to_local_frame(pos)
    
    def _map_to_body(self, pos: Coordinate) -> Coordinate:
        """map 坐标 → body 坐标（以当前 pose 为原点）。"""
        return self._odom_to_body(self._map_to_odom(pos))
    
    def _body_to_map(self, pos: Coordinate) -> Coordinate:
        """body 坐标 → map 坐标（以当前 pose 为原点）。"""
        return self._odom_to_map(self._body_to_odom(pos))
    
    def _body_to_odom(self, pos: Coordinate) -> Coordinate:
        """body 坐标 → odom 坐标（2D 旋转 + 平移）。"""
        p, _, _ = self.get_state()
        return p.body_to_world(pos.x, pos.y, pos.z)

    def _odom_to_body(self, pos: Coordinate) -> Coordinate:
        """odom 绝对坐标 → body 相对坐标（先算位移差, 再 2D 旋转）。"""
        p, _, _ = self.get_state()
        dx = pos.x - p.x
        dy = pos.y - p.y
        dz = pos.z - p.z
        r = p.world_to_body(dx, dy, dz)
        r.rz = wrap_deg(pos.rz - p.rz) if pos.rz is not None else 0.0
        return r


    def set_world(self, x: float, y: float, z: float, yaw_deg: float):
        """设置 odom 系绝对位置。yaw 单位为度。"""
        odom = Coordinate(x=x, y=y, z=z, rz=yaw_deg)
        m = self._odom_to_map(odom)
        self.set_map(m.x, m.y, m.z, m.rz)

    def get_state(self):
        """获取 AUV 当前状态（odom 系，线程安全）。

        Returns:
            (pose: Coordinate, target: Coordinate, status: ZitStatus)
        """
        with self._state_lock:
            return self.pose, self._target, self.status

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 3: 机器人坐标系 (body frame)
    # ═════════════════════════════════════════════════════════════════════════

    def set_body(self, x: float, y: float, z: float, yaw_deg: float):
        """设置机体系绝对位置。yaw 单位为度。"""
        _, t, _ = self.get_state()
        body_target = Coordinate(x=x, y=y, z=z, rz=yaw_deg)
        new_target = t.to_world_frame(body_target)
        self.get_logger().info(
            f'set_body: body目标=({x:.2f}, {y:.2f}, {z:.2f}, {yaw_deg:.1f}°) '
            f'→ world目标=({new_target.x:.2f}, {new_target.y:.2f}, {new_target.z:.2f}, {new_target.rz:.1f}°)')
        self.set_world(new_target.x, new_target.y, new_target.z, new_target.rz)

    def set_step(self, dx: float, dy: float, dz: float, dyaw_deg: float):
        """设置机体系增量步进。dyaw 单位为度。"""
        _, t, _ = self.get_state()
        target_step = Coordinate(x=dx, y=dy, z=dz, rz=dyaw_deg)
        map_target = t.to_world_frame(target_step)

        self.get_logger().info(
            f'set_step: 增量=({dx:.3f}, {dy:.3f}, {dz:.3f}, {dyaw_deg:.2f}°) '
            f'→ world目标=({map_target.x:.2f}, {map_target.y:.2f}, {map_target.z:.2f}, {map_target.rz:.1f}°)')
        self.set_world(map_target.x, map_target.y, map_target.z, map_target.rz)

    # ═════════════════════════════════════════════════════════════════════════
    # 内部工具： 等待到达
    # ═════════════════════════════════════════════════════════════════════════

    def _is_cancelled(self) -> bool:
        """检查当前 action goal 是否被取消（线程安全）。"""
        gh = self._action_goal_handle
        return gh is not None and gh.is_cancel_requested

    def _cmd_and_wait(self, target_x, target_y, target_z, target_yaw_degree, timeout):
        """完成步进移动的最后一步，等待到达目标位置。"""
        p, _, _ = self.get_state()
        dist = math.sqrt((target_x-p.x)**2 + (target_y-p.y)**2 + (target_z-p.z)**2)
        self.get_logger().info(
            f'最终定位: 目标=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}, {target_yaw_degree:.1f}°), '
            f'当前位置=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}, {p.rz:.1f}°), '
            f'距离={dist:.2f}m')
        self.set_world(target_x, target_y, target_z, target_yaw_degree)
        return self._wait_reached(timeout=timeout)
    
    
    def _wait_reached(self,
                      tol_x: float = TOL_X, tol_y: float = TOL_Y,
                      tol_z: float = TOL_Z, tol_rz: float = TOL_RZ,
                      timeout: float = 60.0) -> bool:
        """阻塞等待 AUV 到达目标位置。"""
        start = time.monotonic()
        self._wait_count = 0

        while rclpy.ok():
            if self._is_cancelled():
                self.get_logger().info('等待到达: 被取消')
                return False
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                self.get_logger().warning(f'等待到达超时 ({timeout:.0f}s)')
                return False

            body_target = self._odom_to_body(self._target)

            err_x = abs(body_target.x)
            err_y = abs(body_target.y)
            err_z = abs(body_target.z)
            err_yaw = abs(wrap_deg(body_target.rz))

            reached = True
            if err_x > tol_x:
                reached = False
            if err_y > tol_y:
                reached = False
            if err_z > tol_z:
                reached = False
            if err_yaw > tol_rz:
                reached = False

            # 每秒打印一次误差
            self._wait_count += 1
            if self._wait_count % 10 == 0:
                self.get_logger().info(
                    f'等待到达: err=({err_x:.3f}, {err_y:.3f}, {err_z:.3f}, {err_yaw:.2f}°), '
                    f'容差=({tol_x}, {tol_y}, {tol_z}, {tol_rz}), '
                    f'已用{elapsed:.0f}s/{timeout:.0f}s')

            if reached:
                self.get_logger().info(f'到达目标, 共耗时{elapsed:.1f}s')
                return True
            time.sleep(0.1)

        return False

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 4: 步进坐标系 (运动方向 along/lateral 分解 + 动态步长)
    # ═════════════════════════════════════════════════════════════════════════

    def _calc_step_size(self, move_angle_rad: float) -> float:
        """根据运动方向计算基础步进距离（椭圆模型）。

        r(θ) = 1 / sqrt((cosθ/a)² + (sinθ/b)²)
        纯 X (θ=0°): a = STEP_X, 纯 Y (θ=90°): b = STEP_Y。
        """
        ca = math.cos(move_angle_rad)
        sa = math.sin(move_angle_rad)
        return 1.0 / math.sqrt((ca / STEP_X) ** 2 + (sa / STEP_Y) ** 2)

    def _wait_step_convergence(self, move_angle: float = 0.0,
                               timeout: float = 10.0) -> bool:
        """等待步进收敛（在运动方向坐标系中判断）。

        将误差投影到运动方向坐标系（along/lateral），
        估算以当前速度消除前向误差的时间，≤ STEP_PERIOD 则判为收敛。

        Args:
            move_angle: 运动方向角 (弧度)
            timeout: 超时秒数

        Returns:
            True = 收敛, False = 超时
        """
        start = time.monotonic()
        min_wait = STEP_PERIOD * 0.5
        ca = math.cos(move_angle)
        sa = math.sin(move_angle)

        while rclpy.ok():
            if self._is_cancelled():
                self.get_logger().info('_wait_step_convergence: action cancelled')
                return False
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                return False
            if elapsed < min_wait:
                time.sleep(0.05)
                continue


            body_target = self._odom_to_body(self._target)


            # 机体坐标系误差 → 运动方向坐标系
            e_along = ca * body_target.x + sa * body_target.y      # 前向剩余距离
            e_lateral = -sa * body_target.x + ca * body_target.y   # 横向偏差

            # 机体坐标系速度投影到运动方向坐标系
            with self._state_lock:
                vx_body = self.vel_body['x']
                vy_body = self.vel_body['y']
            v_along = ca * vx_body + sa * vy_body

            # 方向一致性：朝目标前进（速度 > 0）且还有路要走，或已非常近
            converging = (e_along > 0 and v_along > 0) or e_along < 0.01

            # 指数衰减：横向偏差越大，有效前向速度越小
            v_eff = v_along * math.exp(-LATERAL_LAMBDA * abs(e_lateral))

            if v_eff > 0.01:
                t_remaining = e_along / v_eff
            else:
                t_remaining = float('inf')

            step_thresh = self._calc_step_size(move_angle) * 0.3
            if converging and (t_remaining <= STEP_PERIOD * 2  or e_along < step_thresh):
                return True

            time.sleep(0.1)

        return False

    def _step_move_world(self, target_x: float, target_y: float,
                         target_z: float, target_yaw_degree: float,
                         timeout: float = 60.0) -> bool:
        """步进移动：odom 系目标，拆成小段逐段发送。"""
        step_no = 0
        start = time.monotonic()
        self.get_logger().info(
            f'步进开始: 目标=({target_x:.2f}, {target_y:.2f}, {target_z:.2f}, {target_yaw_degree:.1f}°), '
            f'总超时={timeout:.0f}s')
        while rclpy.ok():
            remaining = timeout - (time.monotonic() - start)
            if remaining <= 0:
                self.get_logger().warning(f'步进超时: 已用{time.monotonic()-start:.0f}s')
                return False
            if self._is_cancelled():
                self.get_logger().info('步进被取消')
                return False

            target_world = Coordinate(x=target_x, y=target_y, z=target_z)
            target_body = self._odom_to_body(target_world)

            p, target_odom, _ = self.get_state()
            target_target = target_odom.to_local_frame(target_world)

            # 机体坐标系误差 → 运动方向坐标系
            ex_body = target_body.x
            ey_body = target_body.y
            dist_xy = math.sqrt(target_target.x**2 + target_target.y**2)
            dist_3d = math.sqrt(dist_xy**2 + (target_z - p.z)**2)

            move_angle = math.atan2(ey_body, ex_body)
            base_step = self._calc_step_size(move_angle)

            ca = math.cos(move_angle)
            sa = math.sin(move_angle)
            e_along = ca * target_body.x + sa * target_body.y
            e_lateral = -sa * target_body.x + ca * target_body.y

            if e_along < base_step:
                self.get_logger().info(
                    f'步进结束: 前向误差{e_along:.3f}m < 基步长{base_step:.3f}m, '
                    f'剩余距离={dist_3d:.2f}m')
                break

            # 动态步长：横向误差越大步长越小
            step_along = base_step * math.exp(-LATERAL_LAMBDA * abs(e_lateral))

            dz_total = target_z - p.z
            drz_total = target_yaw_degree - p.rz
            steps_remaining = max(1, math.ceil(dist_xy / base_step))

            step_z = dz_total / steps_remaining
            step_rz = drz_total / steps_remaining

            vector_body = Coordinate(x=ca * step_along, y=sa * step_along, z=step_z, rz=step_rz)

            step_no += 1
            self.get_logger().info(
                f'步进第{step_no}步: 步长={step_along:.3f}m, '
                f'前向误差={e_along:.2f}m, 横向误差={e_lateral:.2f}m, '
                f'剩余距离={dist_3d:.2f}m, '
                f'步进向量=({vector_body.x:.3f}, {vector_body.y:.3f}, {vector_body.z:.3f}, {vector_body.rz:.2f}°)')

            # 步进目标 = 当前 target + 步进增量
            self.set_step(dx=vector_body.x, dy=vector_body.y, dz=vector_body.z, dyaw_deg=vector_body.rz)

            # 每一步用剩余时间（至少 1 秒）
            step_timeout = max(1.0, remaining)
            if not self._wait_step_convergence(move_angle, timeout=step_timeout):
                self.get_logger().warning(f'步进第{step_no}步收敛超时')

        # 最终目标
        remaining = timeout - (time.monotonic() - start)
        if remaining <= 0:
            self.get_logger().warning('步进最终段超时')
            return False
        self.get_logger().info(f'步进完成, 共{step_no}步, 发送最终目标等待到达')
        return self._cmd_and_wait(
            target_x, target_y, target_z, target_yaw_degree,
            timeout=remaining)

    # ═════════════════════════════════════════════════════════════════════════
    # Layer 5: 对外高级 API
    # ═════════════════════════════════════════════════════════════════════════

    # --- SET 系列 (绝对定位) ------------------------------------------------

    def setxyzrz(self, x: float, y: float, z: float, rz: float,
                 timeout: float = 60.0) -> bool:
        """odom 系绝对位置 + 偏航。rz 单位为度。"""
        return self._cmd_and_wait(x, y, z, rz,
                                  timeout=timeout)

    def setxyz(self, x: float, y: float, z: float,
               timeout: float = 60.0) -> bool:
        """odom 系绝对位置（不改变偏航）。"""
        p, _, _ = self.get_state()
        return self._cmd_and_wait(x, y, z, p.rz,
                                  timeout=timeout)

    def setxy(self, x: float, y: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(x, y, p.z, p.rz,
                                  timeout=timeout)

    def setxyrz(self, x: float, y: float, rz: float,
                timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(x, y, p.z, rz,
                                  timeout=timeout)

    def setz(self, z: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(p.x, p.y, z, p.rz,
                                  timeout=timeout)

    def setrz(self, rz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(p.x, p.y, p.z, rz,
                                  timeout=timeout)

    def setx(self, x: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(x, p.y, p.z, p.rz,
                                  timeout=timeout)

    def sety(self, y: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._cmd_and_wait(p.x, y, p.z, p.rz,
                                  AX_Y, timeout=timeout)

    # --- WMOVE 系列 (世界系步进) --------------------------------------------

    def wmovexyzrz(self, dx: float, dy: float, dz: float, drz: float,
                   timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y + dy, p.z + dz,
            p.rz + drz, timeout)

    def wmovexyz(self, dx: float, dy: float, dz: float,
                 timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y + dy, p.z + dz,
            p.rz, timeout)

    def wmovexy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y + dy, p.z,
            p.rz, timeout)

    def wmovexyrz(self, dx: float, dy: float, drz: float,
                  timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y + dy, p.z,
            p.rz + drz, timeout)

    def wmovez(self, dz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y, p.z + dz,
            p.rz, timeout)

    def wmoverz(self, drz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y, p.z,
            p.rz + drz, timeout)

    def wmovex(self, dx: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x + dx, p.y, p.z,
            p.rz, timeout)

    def wmovey(self, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y + dy, p.z,
            p.rz, timeout)

    # --- BMOVE 系列 (机体系步进) --------------------------------------------

    def bmovexyzrz(self, dx: float, dy: float, dz: float, drz: float,
                   timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._step_move_world(
            p.x + off.x, p.y + off.y, p.z + dz,
            p.rz + drz, timeout)

    def bmovexyz(self, dx: float, dy: float, dz: float,
                 timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._step_move_world(
            p.x + off.x, p.y + off.y, p.z + dz,
            p.rz, timeout)

    def bmovexy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._step_move_world(
            p.x + off.x, p.y + off.y, p.z,
            p.rz, timeout)

    def bmovexyrz(self, dx: float, dy: float, drz: float,
                  timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._step_move_world(
            p.x + off.x, p.y + off.y, p.z,
            p.rz + drz, timeout)

    def bmovez(self, dz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y, p.z + dz,
            p.rz, timeout)

    def bmoverz(self, drz: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        return self._step_move_world(
            p.x, p.y, p.z,
            p.rz + drz, timeout)

    def bmovex(self, dx: float, timeout: float = 60.0) -> bool:
        return self.bmovexy(dx, 0.0, timeout=timeout)

    def bmovey(self, dy: float, timeout: float = 60.0) -> bool:
        return self.bmovexy(0.0, dy, timeout=timeout)

    # --- TRAVEL 系列 (直线移动) ---------------------------------------------

    def _travel_world(self, dx_w: float, dy_w: float, dz: float = 0.0,
                      timeout: float = 60.0) -> bool:
        """直线移动基础函数：先转向目标方向，再沿 body-X 步进前进。"""
        p, _, _ = self.get_state()
        target_x = p.x + dx_w
        target_y = p.y + dy_w
        target_z = p.z + dz

        dist_xy = math.sqrt(dx_w**2 + dy_w**2)
        if dist_xy > 0.03:
            target_yaw = math.degrees(math.atan2(dy_w, dx_w))
            self.get_logger().info(
                f'直线移动 第一阶段(转向): 目标朝向{target_yaw:.1f}°, '
                f'当前朝向{p.rz:.1f}°')
            self._step_move_world(p.x, p.y, p.z, target_yaw,
                                  timeout=timeout)
            self.get_logger().info(
                f'直线移动 第二阶段(前进): 目标=({target_x:.2f}, {target_y:.2f}), '
                f'距离={dist_xy:.2f}m')
            return self._step_move_world(
                target_x, target_y, target_z, target_yaw,
                timeout=timeout)
        else:
            self.get_logger().info(
                f'直线移动: 距离过短({dist_xy:.3f}m), 直发深度/偏航')
            return self._step_move_world(
                p.x, p.y, target_z, p.rz,
                timeout=timeout)

    def wtravelxy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        return self._travel_world(dx, dy, 0.0, timeout=timeout)

    def wtravelxyz(self, dx: float, dy: float, dz: float,
                   timeout: float = 60.0) -> bool:
        return self._travel_world(dx, dy, dz, timeout=timeout)

    def wtravelx(self, dx: float, timeout: float = 60.0) -> bool:
        return self._travel_world(dx, 0.0, 0.0, timeout=timeout)

    def wtravely(self, dy: float, timeout: float = 60.0) -> bool:
        return self._travel_world(0.0, dy, 0.0, timeout=timeout)

    def wtravelz(self, dz: float, timeout: float = 60.0) -> bool:
        return self._travel_world(0.0, 0.0, dz, timeout=timeout)

    def btravelxy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._travel_world(off.x, off.y, 0.0, timeout=timeout)

    def btravelxyz(self, dx: float, dy: float, dz: float,
                   timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(dx, dy)
        return self._travel_world(off.x, off.y, dz, timeout=timeout)

    def btravelx(self, dx: float, timeout: float = 60.0) -> bool:
        return self.bmovex(dx, timeout=timeout)

    def btravely(self, dy: float, timeout: float = 60.0) -> bool:
        p, _, _ = self.get_state()
        off = p.body_to_world(0.0, dy)
        return self._travel_world(off.x, off.y, 0.0, timeout=timeout)

    def btravelz(self, dz: float, timeout: float = 60.0) -> bool:
        return self._travel_world(0.0, 0.0, dz, timeout=timeout)

    # ═════════════════════════════════════════════════════════════════════════
    # Action Server 回调
    # ═════════════════════════════════════════════════════════════════════════

    def _action_goal_cb(self, goal_request):
        self.get_logger().info(
            f'Action goal received: cmd_type={goal_request.cmd_type}, '
            f'axes="{goal_request.axes}", target={list(goal_request.target)}, '
            f'timeout={goal_request.timeout:.1f}s')
        return GoalResponse.ACCEPT

    def _action_cancel_cb(self, _goal_handle):
        self.get_logger().info('Action cancel requested, accepting')
        return CancelResponse.ACCEPT

    def _action_execute_cb(self, goal_handle):
        req = goal_handle.request
        self._action_goal_handle = goal_handle

        # START: 初始化 odom 原点，不需要任何前置校验
        if req.cmd_type == BasicMotion.Goal.START:
            self.get_logger().info('Action START: initializing odom origin')
            self.start()
            result = BasicMotion.Result()
            result.success = True
            result.message = "odom origin set"
            goal_handle.succeed()
            self._action_goal_handle = None
            self.get_logger().info('Action START: done')
            return result

        # ── 参数校验 ──────────────────────────────────────────
        if len(req.target) < 4:
            self.get_logger().error(
                f'Action rejected: target needs 4 values [x,y,z,yaw], '
                f'got {len(req.target)}')
            result = BasicMotion.Result()
            result.success = False
            result.message = f"target needs 4 values [x, y, z, yaw], got {len(req.target)}"
            goal_handle.abort()
            self._action_goal_handle = None
            return result

        if self._origin is None:
            self.get_logger().error(
                'Action rejected: odom origin not set, send START first')
            result = BasicMotion.Result()
            result.success = False
            result.message = "odom origin not set, call start() first"
            goal_handle.abort()
            self._action_goal_handle = None
            return result

        x, y, z, yaw = req.target
        timeout = req.timeout if req.timeout > 0 else 60.0
        p, t, _ = self.get_state()

        # ── 派发运动类型 ──────────────────────────────────────
        type_names = {1: 'WMOVE', 2: 'BMOVE', 3: 'SET', 4: 'WTRAVEL', 5: 'BTRAVEL'}
        type_name = type_names.get(req.cmd_type, f'UNKNOWN({req.cmd_type})')
        self.get_logger().info("============================="f'动作 {type_name}: axes="{req.axes}"开始'"=============================")
        self.get_logger().info(
            f'Action {type_name}: axes="{req.axes}", '
            f'target=[{x:.2f}, {y:.2f}, {z:.2f}, {yaw:.1f}], '
            f'pose=[{p.x:.2f}, {p.y:.2f}, {p.z:.2f}, {p.rz:.1f}], '
            f'timeout={timeout:.1f}s')

        if req.cmd_type == BasicMotion.Goal.SET:
            axes = req.axes or 'xyzrz'
            tx, ty, tz, tyaw = t.x, t.y, t.z, t.rz
            if 'x' in axes: tx = x
            if 'y' in axes: ty = y
            if 'z' in axes.replace('rz', ''): tz = z
            if 'rz' in axes: tyaw = yaw
            self._action_target = {'x': tx, 'y': ty, 'z': tz, 'yaw': tyaw}
            success = self.setxyzrz(tx, ty, tz, tyaw, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.WMOVE:
            axes = req.axes or 'xyzrz'
            dx, dy, dz, drz = 0.0, 0.0, 0.0, 0.0
            if 'x' in axes: dx = x
            if 'y' in axes: dy = y
            if 'z' in axes.replace('rz', ''): dz = z
            if 'rz' in axes: drz = yaw
            self._action_target = {'x': p.x + dx, 'y': p.y + dy,
                                   'z': p.z + dz, 'yaw': p.rz + drz}
            success = self.wmovexyzrz(dx, dy, dz, drz, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.BMOVE:
            off = p.body_to_world(x, y)
            self._action_target = {'x': p.x + off.x, 'y': p.y + off.y,
                                   'z': p.z + z, 'yaw': p.rz + yaw}
            self.get_logger().info(
                f'BMOVE坐标变换: body偏移=({x:.2f}, {y:.2f}) '
                f'→ world偏移=({off.x:.2f}, {off.y:.2f}) '
                f'当前yaw={p.rz:.1f}°')
            success = self.bmovexyzrz(x, y, z, yaw, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.WTRAVEL:
            axes = req.axes or 'xyzrz'
            dx, dy, dz = 0.0, 0.0, 0.0
            if 'x' in axes: dx = x
            if 'y' in axes: dy = y
            if 'z' in axes.replace('rz', ''): dz = z
            self._action_target = {'x': p.x + dx, 'y': p.y + dy,
                                   'z': p.z + dz, 'yaw': p.rz}
            self.get_logger().info(
                f'WTRAVEL: 偏移=({dx:.2f}, {dy:.2f}, {dz:.2f}), '
                f'方向角={math.degrees(math.atan2(dy, dx)):.1f}°')
            success = self._travel_world(dx, dy, dz, timeout=timeout)
        elif req.cmd_type == BasicMotion.Goal.BTRAVEL:
            off = p.body_to_world(x, y)
            self._action_target = {'x': p.x + off.x, 'y': p.y + off.y,
                                   'z': p.z + z, 'yaw': p.rz}
            self.get_logger().info(
                f'BTRAVEL: body偏移=({x:.2f}, {y:.2f}) '
                f'→ world偏移=({off.x:.2f}, {off.y:.2f})')
            success = self._travel_world(off.x, off.y, z, timeout=timeout)
        else:
            self.get_logger().error(f'Action rejected: unknown cmd_type={req.cmd_type}')
            result = BasicMotion.Result()
            result.success = False
            result.message = f"unknown cmd_type: {req.cmd_type}"
            goal_handle.abort()
            self._action_goal_handle = None
            return result

        result = BasicMotion.Result()
        result.success = success
        if success:
            result.message = ""
            self.get_logger().info(
                f'Action {type_name}: SUCCESS, '
                f'final_target=({self._action_target["x"]:.2f}, '
                f'{self._action_target["y"]:.2f}, '
                f'{self._action_target["z"]:.2f}, '
                f'{self._action_target["yaw"]:.1f})')
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            result.message = "cancelled"
            self.get_logger().info(f'Action {type_name}: CANCELLED')
            goal_handle.canceled()
        else:
            result.message = "motion timeout"
            self.get_logger().error(f'Action {type_name}: TIMEOUT')
            goal_handle.abort()
        self._action_goal_handle = None
        self._action_target = None
        return result

    def _action_feedback_cb(self):
        if self._action_goal_handle is None or self._action_target is None:
            return
        t = self._action_target
        with self._state_lock:
            dx = t['x'] - self.pose.x
            dy = t['y'] - self.pose.y
            dz = t['z'] - self.pose.z
        feedback = BasicMotion.Feedback()
        feedback.distance_remaining = float(math.sqrt(dx**2 + dy**2 + dz**2))
        self._action_goal_handle.publish_feedback(feedback)


    def _publish_pose_info(self):
        """Publish origin, robot pose, and target pose at 30Hz."""
        msg = PoseInfo()
        msg.stamp = self._pose_stamp
        with self._state_lock:
            if self._origin is not None:
                msg.origin_x = float(self._origin.x)
                msg.origin_y = float(self._origin.y)
                msg.origin_z = float(self._origin.z)
                msg.origin_yaw = float(self._origin.rz)
            msg.robot_x = float(self.pose.x)
            msg.robot_y = float(self.pose.y)
            msg.robot_z = float(self.pose.z)
            msg.robot_roll = float(self.pose.rx)
            msg.robot_pitch = float(self.pose.ry)
            msg.robot_yaw = float(self.pose.rz)
            msg.target_x = float(self._target.x)
            msg.target_y = float(self._target.y)
            msg.target_z = float(self._target.z)
            msg.target_yaw = float(self._target.rz)
        self.pub_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMotionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
