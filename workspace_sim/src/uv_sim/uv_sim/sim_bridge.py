"""Simulation bridge: emulates ZIT6 MCU firmware for Stonefish simulation.

Protocol matches real AUV: ZitSetpoint in → ZitStatus + Float32MultiArray out.
Cascade PID: pos → yaw-rate cascade, velocity inner loops, direct force mode.
Thruster mixer: xunyun 6-thruster geometry (direct NED body force → thrusts).
"""

import json
import math
from dataclasses import dataclass

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Image, Imu
from std_msgs.msg import Float64MultiArray, Float32, UInt8, UInt32, Float32MultiArray

try:
    from stonefish_ros2.msg import DVL
except ImportError:
    DVL = None
from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from zit6_interfaces.srv import GetParams, UpdateParams


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class _Pid:
    kp: float
    ki: float
    kd: float
    i_limit: float = 0.5
    z_thrust_ff: float = 0.0

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, err: float, dt: float) -> float:
        if dt <= 0.0:
            return self.kp * err + self.z_thrust_ff
        self.integral = _clamp(self.integral + err * dt, -self.i_limit, self.i_limit)
        derivative = (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * derivative + self.z_thrust_ff


class SimBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_bridge")

        self.declare_parameter('hil_mode', False)
        self._hil_mode = self.get_parameter('hil_mode').value

        if self._hil_mode:
            self._init_hil()
            self.get_logger().info("sim_bridge started in HIL mode (camera + thrust mixing)")
        else:
            self._init_full()
            self.get_logger().info("sim_bridge started (ZIT6 protocol, xunyun mixer)")

    def _init_full(self) -> None:
        """Full SIL mode: PID, thruster mixing, ZIT6 state machine + camera passthrough."""
        # Internal state
        self._tick = 0  # for rate-gated publishing

        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}     # NED deg
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}  # body, deg/s
        self.vel_world = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.thrust = [0.0] * 6
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.target_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.force_4dof = [0.0, 0.0, 0.0, 0.0]
        self._current_mode = 0  # 0=NONE, 1=POS, 2=VEL, 3=FORCE
        self.position_ctrl_enabled = False
        self.vel_ctrl_enabled = False
        self.current_pose_ready = False
        self.last_pid_time = self.get_clock().now()

        # ARM / heartbeat state (always armed, heartbeat is optional)
        self._armed = True
        self._arm_mode = 3
        self._last_heartbeat_time = self.get_clock().now()
        self._hb_seq = 0

        # INS state: 0=待机 1=粗对准 2=精对准 3=SINS/GPS/DVL 4=SINS/DVL 5=MRU
        self._ins_state = 1          # start in coarse alignment
        self._ins_nav_ready = False  # not ready until coarse done
        self._dvl_enabled = False    # DVL starts off
        self._ins_boot_time = self.get_clock().now()  # used for initial 1→5 transition
        self._ins_align_request = None  # time when SINS/DVL alignment was requested

        # === Publishers ===
        self.thruster_pub = self.create_publisher(
            Float64MultiArray, "/auv/thrusters_cmd", 10
        )
        self.zit6_status_pub = self.create_publisher(ZitStatus, "/zit6/state/status", 10)
        self.zit6_pos_pub = self.create_publisher(Float32MultiArray, "/zit6/state/pos", 10)
        self.zit6_vel_pub = self.create_publisher(Float32MultiArray, "/zit6/state/vel", 10)
        self.zit6_thr_pub = self.create_publisher(Float32MultiArray, "/zit6/state/thr", 10)
        self.zit6_hbt_pub = self.create_publisher(UInt32, "/zit6/state/zithbt", 10)

        # Camera republishers
        self._init_camera_publishers()

        # Image stitching
        self._init_camera_state()

        # === PID controllers ===
        # Gains stored in dict for service access, synced to _Pid objects
        self._pid_params = {
            'chassis.pid.pos.x':   {'kp': 800,   'ki': 130.0, 'kd': 200.0,  'i_limit': 1000.0},
            'chassis.pid.pos.y':   {'kp': 600.0, 'ki': 120.0, 'kd': 150.0,  'i_limit': 1000.0},
            'chassis.pid.pos.z':   {'kp': 600.0, 'ki': 30.0,  'kd': 800.0,  'i_limit': 6200.0},
            'chassis.pid.pos.yaw': {'kp': 1.5,   'ki': 1.0,   'kd': 0.1,    'i_limit': 30.0},
            'chassis.pid.vel.x':   {'kp': 300.0, 'ki': 50.0,  'kd': 10.0,   'i_limit': 500.0},
            'chassis.pid.vel.y':   {'kp': 300.0, 'ki': 50.0,  'kd': 10.0,   'i_limit': 500.0},
            'chassis.pid.vel.z':   {'kp': 300.0, 'ki': 30.0,  'kd': 50.0,   'i_limit': 500.0},
            'chassis.pid.vel.yaw': {'kp': 40.0,  'ki': 20.0,  'kd': 2.0,    'i_limit': 300.0},
        }
        # Map param paths to _Pid instances for runtime updates
        self._pid_objects = {}  # populated after PID creation

        # Position: body-frame error → body force
        self.pid_x = _Pid(800, 130.0, 200.0, i_limit=1000.0)
        self.pid_y = _Pid(600.0, 120.0, 150.0, i_limit=1000.0)
        self.pid_z = _Pid(600.0, 80.0, 800.0, i_limit=6200.0, z_thrust_ff=0)
        self.pid_yaw = _Pid(1.5, 1.0, 0.1, i_limit=30.0)
        self.pid_yaw_rate = _Pid(30.0, 20.0, 1.0, i_limit=600.0)

        # Velocity: body-frame velocity error → body force
        self.pid_vx = _Pid(600.0, 50.0, 10.0, i_limit=500.0)
        self.pid_vy = _Pid(300.0, 50.0, 10.0, i_limit=500.0)
        self.pid_vz = _Pid(300.0, 30.0, 50.0, i_limit=500.0, z_thrust_ff=0)
        self.pid_vyaw = _Pid(40.0, 20.0, 2.0, i_limit=300.0)

        # Map param paths to PID objects for write-through
        self._pid_objects = {
            'chassis.pid.pos.x':   self.pid_x,
            'chassis.pid.pos.y':   self.pid_y,
            'chassis.pid.pos.z':   self.pid_z,
            'chassis.pid.pos.yaw': self.pid_yaw,
            'chassis.pid.vel.x':   self.pid_vx,
            'chassis.pid.vel.y':   self.pid_vy,
            'chassis.pid.vel.z':   self.pid_vz,
            'chassis.pid.vel.yaw': self.pid_vyaw,
        }

        # === Subscriptions ===
        self.create_subscription(ZitSetpoint, "/zit6/cmd/setpoint", self._setpoint_cb, 10)
        self.create_subscription(Float32, "/zit6/cmd/servo", self._servo_cb, 10)
        self.create_subscription(UInt8, "/zit6/cmd/light", self._light_cb, 10)
        self.create_subscription(Odometry, "/auv/odometry", self._odom_cb, 10)
        self.create_subscription(Imu, "/auv/imu", self._imu_cb, 10)
        if DVL is not None:
            self.create_subscription(DVL, "/auv/dvl", self._dvl_cb, 10)
        self.create_subscription(FluidPressure, "/auv/pressure", self._pressure_cb, 10)
        self.create_subscription(UInt32, "/zit6/cmd/agxhbt", self._agxhbt_cb, 10)
        self.create_subscription(UInt8, "/zit6/cmd/ins", self._ins_cb, 10)

        # === Services ===
        self.create_service(GetParams, "/zit6/get_params", self._get_params_cb)
        self.create_service(UpdateParams, "/zit6/update_params", self._update_params_cb)

        # Camera subscriptions
        self._init_camera_subscriptions()

        self.create_timer(0.1, self._publish_zithbt)   # 10Hz heartbeat
        self.create_timer(0.0167, self._tick_60hz)      # 60Hz control + state

    def _init_hil(self) -> None:
        """HIL mode: camera passthrough + thrust mixing + nav aggregation for MCU."""
        self._init_camera_publishers()
        self._init_camera_state()
        self._init_camera_subscriptions()

        # Thrust mixing: MCU publishes 4-DOF forces → we mix to 6 thrusters
        self.thrust = [0.0] * 6
        self.force_4dof = [0.0, 0.0, 0.0, 0.0]
        self.thruster_pub = self.create_publisher(
            Float64MultiArray, "/auv/thrusters_cmd", 10
        )
        self.create_subscription(
            Float32MultiArray, "/zit6/state/thr", self._thrust_cb, 10
        )

        # Nav aggregation: subscribe Stonefish odometry + IMU, publish /zit6/sim/nav
        self._sim_pos = [0.0] * 6   # x, y, z, roll, pitch, yaw (NED)
        self._sim_vel = [0.0] * 6   # u, v, w, p, q, r (body FRD)
        self.sim_nav_pub = self.create_publisher(
            Float32MultiArray, "/zit6/sim/nav", 10
        )
        self.create_subscription(Odometry, "/auv/odometry", self._sim_nav_odom_cb, 10)
        self.create_subscription(Imu, "/auv/imu", self._sim_nav_imu_cb, 10)

    def _thrust_cb(self, msg: Float32MultiArray) -> None:
        """Receive 4-DOF forces from MCU, run thrust mixing, publish to Stonefish."""
        if len(msg.data) >= 4:
            self._publish_thrust_from_4dof(
                msg.data[0], msg.data[1], msg.data[2], msg.data[3]
            )

    def _sim_nav_odom_cb(self, msg: Odometry) -> None:
        """Aggregate Stonefish odometry into /zit6/sim/nav position + body velocity."""
        # Position (NED world frame)
        self._sim_pos[0] = msg.pose.pose.position.x
        self._sim_pos[1] = msg.pose.pose.position.y
        self._sim_pos[2] = msg.pose.pose.position.z
        # Orientation → roll, pitch, yaw
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self._sim_pos[3] = roll
        self._sim_pos[4] = pitch
        self._sim_pos[5] = yaw

        # World-frame linear velocity → body-frame (NED FRD)
        vx_w = msg.twist.twist.linear.x
        vy_w = msg.twist.twist.linear.y
        cy, sy = math.cos(yaw), math.sin(yaw)
        self._sim_vel[0] = vx_w * cy + vy_w * sy    # u (surge)
        self._sim_vel[1] = -vx_w * sy + vy_w * cy   # v (sway)
        self._sim_vel[2] = msg.twist.twist.linear.z  # w (heave)
        # p, q, r filled by IMU callback

        self._publish_sim_nav()

    def _sim_nav_imu_cb(self, msg: Imu) -> None:
        """Store angular velocity from IMU for /zit6/sim/nav."""
        self._sim_vel[3] = msg.angular_velocity.x  # p (roll rate)
        self._sim_vel[4] = msg.angular_velocity.y  # q (pitch rate)
        self._sim_vel[5] = msg.angular_velocity.z  # r (yaw rate)

    def _publish_sim_nav(self) -> None:
        """Publish aggregated nav data (12 floats) on /zit6/sim/nav."""
        nav = Float32MultiArray()
        nav.data = self._sim_pos + self._sim_vel
        self.sim_nav_pub.publish(nav)

    def _init_camera_publishers(self) -> None:
        self.front_rect_left_pub = self.create_publisher(Image, "/auv/front_cam/left", 10)
        self.front_rect_right_pub = self.create_publisher(Image, "/auv/front_cam/right", 10)
        self.down_rect_left_pub = self.create_publisher(Image, "/auv/down_cam/left", 10)
        self.down_rect_right_pub = self.create_publisher(Image, "/auv/down_cam/right", 10)
        self.front_rect_pub = self.create_publisher(Image, "/auv/front_cam/stitched", 10)
        self.down_rect_pub = self.create_publisher(Image, "/auv/down_cam/stitched", 10)

    def _init_camera_state(self) -> None:
        self.bridge = CvBridge()
        self.front_left_img = None
        self.front_right_img = None
        self.down_left_img = None
        self.down_right_img = None

    def _init_camera_subscriptions(self) -> None:
        self.create_subscription(Image, "/sim/front_cam/left/image_color", self._front_left_img_cb, 10)
        self.create_subscription(Image, "/sim/front_cam/right/image_color", self._front_right_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/left/image_color", self._down_left_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/right/image_color", self._down_right_img_cb, 10)

    # ── ZIT6 setpoint callback ──────────────────────────────────────

    def _setpoint_cb(self, msg: ZitSetpoint) -> None:
        mode = msg.control_key & 0x03
        frame = msg.control_key & 0x10       # 0x10 = body
        incremental = msg.control_key & 0x20  # 0x20 = incremental

        if mode == 2:
            # Force mode: bit=1 = keep current, bit=0 = apply
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = False
            self._current_mode = 3
            x = self.force_4dof[0] if (msg.type_mask & 0x01) else msg.x
            y = self.force_4dof[1] if (msg.type_mask & 0x02) else msg.y
            z = self.force_4dof[2] if (msg.type_mask & 0x04) else msg.z
            rz = self.force_4dof[3] if (msg.type_mask & 0x08) else msg.yaw
            self._publish_thrust_from_4dof(x, y, z, rz)

        elif mode == 1:
            # Velocity mode: bit=1 = ignore, bit=0 = apply
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = True
            self._current_mode = 2
            if not (msg.type_mask & 0x01):
                self.target_vel['x'] = msg.x
            if not (msg.type_mask & 0x02):
                self.target_vel['y'] = msg.y
            if not (msg.type_mask & 0x04):
                self.target_vel['z'] = msg.z
            if not (msg.type_mask & 0x08):
                self.target_vel['rz'] = math.degrees(msg.yaw)
            self.last_pid_time = self.get_clock().now()
            self.pid_vx.reset(); self.pid_vy.reset()
            self.pid_vz.reset(); self.pid_vyaw.reset()

        elif mode == 0:
            # Position mode: bit=1 = ignore, bit=0 = apply
            self.vel_ctrl_enabled = False
            self._current_mode = 1
            yaw_deg = math.degrees(msg.yaw)

            if incremental:
                dx = 0.0 if (msg.type_mask & 0x01) else msg.x
                dy = 0.0 if (msg.type_mask & 0x02) else msg.y
                yaw_rad = math.radians(self.pos['rz'])
                cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                self.target_pos['x'] += cy * dx - sy * dy
                self.target_pos['y'] += sy * dx + cy * dy
                if not (msg.type_mask & 0x04):
                    self.target_pos['z'] += msg.z
                if not (msg.type_mask & 0x08):
                    self.target_pos['rz'] += yaw_deg
            else:
                if frame:
                    # Absolute body → world
                    yaw_rad = math.radians(self.pos['rz'])
                    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                    if not (msg.type_mask & 0x01):
                        self.target_pos['x'] = self.pos['x'] + cy * msg.x - sy * msg.y
                    if not (msg.type_mask & 0x02):
                        self.target_pos['y'] = self.pos['y'] + sy * msg.x + cy * msg.y
                else:
                    # Absolute world
                    if not (msg.type_mask & 0x01):
                        self.target_pos['x'] = msg.x
                    if not (msg.type_mask & 0x02):
                        self.target_pos['y'] = msg.y
                if not (msg.type_mask & 0x04):
                    self.target_pos['z'] = msg.z
                if not (msg.type_mask & 0x08):
                    self.target_pos['rz'] = yaw_deg

            self.position_ctrl_enabled = True
            self.last_pid_time = self.get_clock().now()
            self.pid_x.reset(); self.pid_y.reset(); self.pid_z.reset()
            self.pid_yaw.reset(); self.pid_yaw_rate.reset()

    def _servo_cb(self, msg: Float32) -> None:
        self.get_logger().info(f"Servo: {msg.data:.3f} rad")

    def _light_cb(self, msg: UInt8) -> None:
        colors = {1: "red", 2: "yellow", 3: "green"}
        name = colors.get(msg.data, f"unknown ({msg.data})")
        self.get_logger().info(f"Light: {name}")

    # ── Heartbeat / ARM ─────────────────────────────────────────────

    def _agxhbt_cb(self, msg: UInt32) -> None:
        self._last_heartbeat_time = self.get_clock().now()
        arm_val = msg.data
        if arm_val == 3:
            # Force arm (bypass INS)
            self._arm_mode = 3
            if not self._armed:
                self.get_logger().info("ARM: force arm (mode 3)")
            self._armed = True
        elif arm_val == 1 and self._ins_nav_ready:
            # Normal arm (INS must be ready)
            self._arm_mode = 0
            if not self._armed:
                self.get_logger().info("ARM: normal arm (mode 0)")
            self._armed = True

    def _publish_zithbt(self) -> None:
        # Always armed — heartbeat timeout disabled

        msg = UInt32()
        self._hb_seq += 1
        msg.data = self._hb_seq
        self.zit6_hbt_pub.publish(msg)

    # ── INS command ──────────────────────────────────────────────────

    def _ins_cb(self, msg: UInt8) -> None:
        cmd = msg.data
        if cmd == 1:
            self._dvl_enabled = True
            self.get_logger().info("INS: DVL enabled, requesting SINS/DVL alignment")
            if self._ins_state == 5:
                self._ins_align_request = self.get_clock().now()
            elif self._ins_state == 1:
                self.get_logger().info("INS: waiting for coarse alignment first")
        elif cmd == 2:
            self._dvl_enabled = False
            self._ins_align_request = None
            self.get_logger().info("INS: DVL disabled, fallback to MRU")
            if self._ins_state == 4:
                self._ins_state = 5
        elif cmd == 3:
            self.get_logger().info("INS: restarting alignment...")
            self._ins_state = 1
            self._ins_nav_ready = False
            self._dvl_enabled = False
            self._ins_align_request = None
            self._ins_boot_time = self.get_clock().now()

    def _update_ins_alignment(self) -> None:
        """INS state machine: 1→5→4 (or 1→5 if no DVL)."""
        now = self.get_clock().now()

        # Phase 1: coarse alignment (1 → 5, ~1.5s after boot)
        if self._ins_state == 1:
            elapsed = (now - self._ins_boot_time).nanoseconds / 1e9
            if elapsed >= 1.5:
                self._ins_state = 5
                self._ins_nav_ready = True
                self.get_logger().info("INS: coarse alignment done → MRU mode")
                # If DVL was already requested while in state 1, start alignment now
                if self._dvl_enabled:
                    self._ins_align_request = now

        # Phase 2: SINS/DVL alignment (5 → 4, ~1.5s after request)
        if self._ins_state == 5 and self._ins_align_request is not None:
            elapsed = (now - self._ins_align_request).nanoseconds / 1e9
            if elapsed >= 1.5:
                self._ins_state = 4
                self._ins_align_request = None
                self.get_logger().info("INS: SINS/DVL alignment complete")

    # ── Parameter services ───────────────────────────────────────────

    def _get_params_cb(self, request, response):
        """Return PID parameters as JSON for requested paths."""
        if not request.paths:
            # Return full config
            response.success = True
            response.config_json = json.dumps(self._pid_params)
            return response

        result = {}
        for path in request.paths:
            value = self._pid_params.get(path)
            if value is not None:
                result[path] = value

        response.success = True
        response.config_json = json.dumps(result)
        return response

    def _update_params_cb(self, request, response):
        """Update PID parameters from JSON or path/value pairs."""
        try:
            if request.json:
                updates = json.loads(request.json)
            else:
                updates = dict(zip(request.paths, request.values))

            for path, value in updates.items():
                if path in self._pid_params:
                    gains = self._pid_params[path]
                    if isinstance(value, dict):
                        # Full gain dict: {"kp": 800, "ki": 130.0, ...}
                        for k, v in value.items():
                            if k in gains:
                                gains[k] = float(v)
                    elif isinstance(value, (int, float)):
                        # Single key: interpret as kp
                        gains['kp'] = float(value)

                    # Write-through to live PID object
                    pid = self._pid_objects.get(path)
                    if pid is not None:
                        pid.kp = gains['kp']
                        pid.ki = gains.get('ki', 0.0)
                        pid.kd = gains.get('kd', 0.0)
                        pid.i_limit = gains.get('i_limit', 0.5)
                    self.get_logger().info(f"Params: {path} → {gains}")

            response.success = True
            response.message = "Parameters updated"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    # ── Image callbacks ─────────────────────────────────────────────

    def _front_left_img_cb(self, msg: Image) -> None:
        self.front_rect_left_pub.publish(msg)
        self.front_left_img = msg
        self._publish_stitched_front()

    def _front_right_img_cb(self, msg: Image) -> None:
        self.front_rect_right_pub.publish(msg)
        self.front_right_img = msg
        self._publish_stitched_front()

    def _down_left_img_cb(self, msg: Image) -> None:
        self.down_rect_left_pub.publish(msg)
        self.down_left_img = msg
        self._publish_stitched_down()

    def _down_right_img_cb(self, msg: Image) -> None:
        self.down_rect_right_pub.publish(msg)
        self.down_right_img = msg
        self._publish_stitched_down()

    def _publish_stitched_front(self) -> None:
        if self.front_left_img and self.front_right_img:
            try:
                left = self.bridge.imgmsg_to_cv2(self.front_left_img, "bgr8")
                right = self.bridge.imgmsg_to_cv2(self.front_right_img, "bgr8")
                stitched = np.hstack((left, right))
                out = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                out.header = self.front_left_img.header
                self.front_rect_pub.publish(out)
            except Exception as e:
                self.get_logger().error(f"Front stitch failed: {e}")

    def _publish_stitched_down(self) -> None:
        if self.down_left_img and self.down_right_img:
            try:
                left = self.bridge.imgmsg_to_cv2(self.down_left_img, "bgr8")
                right = self.bridge.imgmsg_to_cv2(self.down_right_img, "bgr8")
                stitched = np.hstack((left, right))
                out = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                out.header = self.down_left_img.header
                self.down_rect_pub.publish(out)
            except Exception as e:
                self.get_logger().error(f"Down stitch failed: {e}")

    # ── Thruster mixer (xunyun 6-thruster geometry) ────────────────

    def _publish_thrust_from_4dof(self, x: float, y: float, z: float, rz: float) -> None:
        # DISABLED: always armed for now
        # if not self._armed:
        #     x = y = z = rz = 0.0
        self.force_4dof = [x, y, z, rz]
        MAX_T = 1000.0
        if self._hil_mode:
            MAX_T = 1.0  # HIL mode: forces are normalized to [-1, 1]

        # NED body force → 6 thrusters (xunyun geometry)
        h0 = (x + y - rz )     # T0: aft-stbd diagonal
        h1 = (x - y + rz )     # T1: aft-port diagonal
        h4 = -(x - y - rz )    # T4: fwd-stbd diagonal
        h5 = -(x + y + rz )    # T5: fwd-port diagonal
        h2 = z*0.8;                        # HeaveBow
        h3 = z*0.8;                        # HeaveStern

        raw = [h0, h1, h2, h3, h4, h5]
        for i in range(len(raw)):
            raw[i] = _clamp(raw[i] / MAX_T, -1.0, 1.0)

        cmd = Float64MultiArray()
        cmd.data = raw
        self.thruster_pub.publish(cmd)

        for i in range(6):
            self.thrust[i] = float(raw[i])

    # ── Control step ────────────────────────────────────────────────

    def _tick_60hz(self) -> None:
        """60Hz control step + rate-gated state publishing."""
        self._tick = (self._tick + 1) % 60

        # State publishing (gate by tick count)
        #   status: 10Hz (every 6 ticks, tick 0)
        #   pos:    30Hz (every 2 ticks, tick 0,2,4...)
        #   vel:    60Hz (every tick)
        #   thr:    30Hz (every 2 ticks, tick 0,2,4...)
        if self._tick % 6 == 0:
            self._publish_state()
        if self._tick % 2 == 0:
            self._publish_pos()
            self._publish_thr()
        self._publish_vel()

        # Control step
        self._update_ins_alignment()
        if not self.current_pose_ready:
            return

        now = self.get_clock().now()
        dt = (now - self.last_pid_time).nanoseconds / 1e9
        self.last_pid_time = now
        dt = _clamp(dt, 0.001, 0.2)

        if self.position_ctrl_enabled:
            self._position_pid_step(dt)
        elif self.vel_ctrl_enabled:
            self._velocity_pid_step(dt)

    def _position_pid_step(self, dt: float) -> None:
        x, y, z, yaw = self.pos['x'], self.pos['y'], self.pos['z'], self.pos['rz']

        ex_w = self.target_pos['x'] - x
        ey_w = self.target_pos['y'] - y

        yaw_rad = math.radians(yaw)
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        ex_body = cy * ex_w + sy * ey_w
        ey_body = -sy * ex_w + cy * ey_w

        ez = self.target_pos['z'] - z
        eyaw = self._wrap_angle_deg(self.target_pos['rz'] - yaw)

        target_yaw_rate = self.pid_yaw.step(eyaw, dt)
        target_yaw_rate = _clamp(target_yaw_rate, -45.0, 45.0)
        eyaw_rate = target_yaw_rate - self.vel['rz']

        cmd_x = float(self.pid_x.step(ex_body, dt))
        cmd_y = float(self.pid_y.step(ey_body, dt))
        cmd_z = float(self.pid_z.step(ez, dt))
        cmd_rz = float(self.pid_yaw_rate.step(eyaw_rate, dt))

        self._publish_thrust_from_4dof(cmd_x, cmd_y, cmd_z, cmd_rz)

    def _velocity_pid_step(self, dt: float) -> None:
        evx = self.target_vel['x'] - self.vel['x']
        evy = self.target_vel['y'] - self.vel['y']
        evz = self.target_vel['z'] - self.vel['z']
        evyaw = self.target_vel['rz'] - self.vel['rz']

        cmd_x = float(self.pid_vx.step(evx, dt))
        cmd_y = float(self.pid_vy.step(evy, dt))
        cmd_z = float(self.pid_vz.step(evz, dt))
        cmd_rz = float(self.pid_vyaw.step(evyaw, dt))

        self._publish_thrust_from_4dof(cmd_x, cmd_y, cmd_z, cmd_rz)

    # ── Sensor callbacks ────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self.pos['x'] = msg.pose.pose.position.x
        self.pos['y'] = msg.pose.pose.position.y
        self.pos['z'] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self.pos['rx'] = math.degrees(roll)
        self.pos['ry'] = math.degrees(pitch)
        self.pos['rz'] = math.degrees(yaw)

        vx_w = msg.twist.twist.linear.x
        vy_w = msg.twist.twist.linear.y
        self.vel_world['x'] = vx_w
        self.vel_world['y'] = vy_w
        self.vel_world['z'] = msg.twist.twist.linear.z
        self.vel_world['rx'] = math.degrees(msg.twist.twist.angular.x)
        self.vel_world['ry'] = math.degrees(msg.twist.twist.angular.y)
        self.vel_world['rz'] = math.degrees(msg.twist.twist.angular.z)

        # World → body velocity rotation (yaw-only for XY linear, direct for angular)
        cy, sy = math.cos(yaw), math.sin(yaw)
        self.vel['x'] = vx_w * cy + vy_w * sy
        self.vel['y'] = -vx_w * sy + vy_w * cy
        self.vel['z'] = msg.twist.twist.linear.z
        self.vel['rx'] = self.vel_world['rx']
        self.vel['ry'] = self.vel_world['ry']
        self.vel['rz'] = self.vel_world['rz']

        self.current_pose_ready = True

    def _imu_cb(self, msg: Imu) -> None:
        self.vel['rx'] = math.degrees(msg.angular_velocity.x)
        self.vel['ry'] = math.degrees(msg.angular_velocity.y)
        self.vel['rz'] = math.degrees(msg.angular_velocity.z)

    def _dvl_cb(self, msg: DVL) -> None:
        self.vel['x'] = msg.velocity.x
        self.vel['y'] = msg.velocity.y
        self.vel['z'] = msg.velocity.z

    def _pressure_cb(self, msg: FluidPressure) -> None:
        pass

    # ── ZIT6 state publishing ───────────────────────────────────────
    # Published at different rates: status=10Hz, pos=30Hz, vel=60Hz, thr=30Hz

    def _publish_state(self) -> None:
        """Publish ZitStatus (10Hz)."""
        status = ZitStatus()
        status.is_armed = self._armed
        status.arm_mode = self._arm_mode
        status.control_level = self._current_mode
        status.ins_state = self._ins_state
        status.navigation_ready = self._ins_nav_ready
        status.forces = [self.force_4dof[0], self.force_4dof[1], self.force_4dof[2],
                          0.0, 0.0, self.force_4dof[3]]
        status.cycle_time_ms = 50.0
        status.battery_voltage = 16.8
        status.error_flags = 0
        self.zit6_status_pub.publish(status)

    def _publish_pos(self) -> None:
        """Publish position (30Hz). 6-DOF: [x, y, z, roll_rad, pitch_rad, yaw_rad]."""
        pos_msg = Float32MultiArray()
        pos_msg.data = [
            self.pos['x'], self.pos['y'], self.pos['z'],
            math.radians(self.pos.get('rx', 0.0)),
            math.radians(self.pos.get('ry', 0.0)),
            math.radians(self.pos['rz']),
        ]
        self.zit6_pos_pub.publish(pos_msg)

    def _publish_vel(self) -> None:
        """Publish body velocity 6-DOF [vx, vy, vz, vroll_rad, vpitch_rad, vyaw_rad] (60Hz)."""
        vel_msg = Float32MultiArray()
        vel_msg.data = [
            self.vel['x'], self.vel['y'], self.vel['z'],
            math.radians(self.vel['rx']),
            math.radians(self.vel['ry']),
            math.radians(self.vel['rz']),
        ]
        self.zit6_vel_pub.publish(vel_msg)

    def _publish_thr(self) -> None:
        """Publish thrust 6-DOF [Fx, Fy, Fz, Mx(roll), My(pitch), Mz(yaw)] (30Hz)."""
        thr_msg = Float32MultiArray()
        # force_4dof = [Fx, Fy, Fz, Mz] → 6 元素, Mx/My 留空填 0
        thr_msg.data = [
            self.force_4dof[0], self.force_4dof[1], self.force_4dof[2],
            0.0, 0.0, self.force_4dof[3],
        ]
        self.zit6_thr_pub.publish(thr_msg)

    # ── Utilities ───────────────────────────────────────────────────

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(_clamp(sinp, -1.0, 1.0))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _wrap_angle_deg(angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
