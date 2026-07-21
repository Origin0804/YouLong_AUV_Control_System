"""Position node: monocular ray intersection for 3D object positioning.

Subscribes to YOLO detection results + robot pose.
Implements ray accumulation, median filtering, and pairwise intersection.
Tracks and remembers object world positions.
"""

import bisect
import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node

from uv_msgs.msg import DetectionArray, ObjectPosition, ObjectPositionArray, PoseInfo


# Camera intrinsic + extrinsic parameters (from xunyun_fixed.scn)
# Each camera has its own body offset; optical_to_body is shared per pair
CAM_PARAMS = {
    'front_left': {
        'width': 1280, 'height': 960, 'hfov': 57.19,
        'offset': np.array([0.23, -0.05, 0.076]),
        'optical_to_body': np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),
    },
    'front_right': {
        'width': 1280, 'height': 960, 'hfov': 57.19,
        'offset': np.array([0.23, 0.05, 0.076]),
        'optical_to_body': np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),
    },
    'down_left': {
        'width': 1280, 'height': 960, 'hfov': 87.19,
        'offset': np.array([-0.13, -0.05, 0.0645]),
        'optical_to_body': np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
    },
    'down_right': {
        'width': 1280, 'height': 960, 'hfov': 87.19,
        'offset': np.array([-0.13, 0.05, 0.0645]),
        'optical_to_body': np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
    },
}

MIN_BASELINE = 0.1  # minimum baseline between ray origins (meters)
MAX_DISTANCE = 50.0  # maximum object distance (meters)
MIN_RAYS_FOR_INTERSECTION = 3


def _euler_to_rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """ZYX Euler angles (degrees) to rotation matrix."""
    rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    R = np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                 cy * cx],
    ])
    return R


CLASS_NAMES = {
    0: "yellow_sector", 1: "red_sector", 2: "green_sector",
    3: "arrow", 4: "start", 5: "triangle",
    6: "square", 7: "basket", 8: "aruco_tag",
}

# Class IDs that may appear multiple times in the same scene.
# Only these use angle-based instance matching; all others always go to instance 0.
MULTI_INSTANCE_CLASSES = {3, 5, 6}   # arrow, triangle, square


class PositionNode(Node):
    """Position node: 3D object localization via monocular ray intersection."""

    def __init__(self):
        super().__init__('position')

        self.declare_parameter('max_history', 30)
        self._max_history = self.get_parameter('max_history').get_parameter_value().integer_value

        self.declare_parameter('angle_match_threshold', 50.0)
        self._angle_match_threshold = self.get_parameter(
            'angle_match_threshold').get_parameter_value().double_value

        # Robot pose (NED)
        self.robot_pos = np.array([0.0, 0.0, 0.0])
        self.robot_roll = 0.0   # degrees
        self.robot_pitch = 0.0  # degrees
        self.robot_yaw = 0.0    # degrees
        self._pose_received = False

        # Pose history ring buffer for time-aligned lookup
        # Each entry: (timestamp_sec, pos_xyz, roll, pitch, yaw)
        self._pose_buffer: deque = deque()
        self._pose_buffer_duration = 2.0  # keep 2 seconds of history
        self._pose_buffer_max = 150       # hard cap at 30Hz input rate

        # Ray history: (class_id, camera_pair, instance_id) -> list of rays
        self.ray_history: dict[tuple, list] = {}

        # Object memory: (class_id, camera_pair, instance_id) -> position info
        self.object_memory: dict[tuple, dict] = {}

        # Instance ID counter per (class_id, camera_pair)
        self._next_instance_id: dict[tuple, int] = {}

        # Debug counters
        self._det_msg_count: dict[str, int] = {}
        self._ray_skipped_baseline: int = 0
        self._ray_added_total: int = 0
        self._intersect_attempts: int = 0
        self._intersect_fails: int = 0
        self._debug_timer = None  # set after first detection

        # Precompute focal lengths and centers per camera
        self._cam_params = {}
        for name, p in CAM_PARAMS.items():
            fx = p['width'] / (2.0 * math.tan(math.radians(p['hfov']) / 2.0))
            self._cam_params[name] = {
                'fx': fx, 'fy': fx,  # square pixels
                'cx': p['width'] / 2.0, 'cy': p['height'] / 2.0,
                'width': p['width'], 'height': p['height'],
                'offset': p['offset'],
                'optical_to_body': p['optical_to_body'],
            }

        # Subscribers
        self.create_subscription(DetectionArray, '/perception/detection/front_left', self._front_left_cb, 10)
        self.create_subscription(DetectionArray, '/perception/detection/front_right', self._front_right_cb, 10)
        self.create_subscription(DetectionArray, '/perception/detection/down_left', self._down_left_cb, 10)
        self.create_subscription(DetectionArray, '/perception/detection/down_right', self._down_right_cb, 10)
        self.create_subscription(PoseInfo, '/basic_motion/pose_info', self._pose_cb, 10)

        # Publisher
        self.pub_objects = self.create_publisher(ObjectPositionArray, '/perception/objects', 10)

        # Broadcast timer
        self.create_timer(0.1, self._broadcast_objects)  # 10Hz

        self.get_logger().info('Position node started')

    def _pose_cb(self, msg: PoseInfo):
        # Append to time-aligned buffer
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        pos = np.array([msg.robot_x, msg.robot_y, msg.robot_z])
        self._pose_buffer.append((t, pos, msg.robot_roll, msg.robot_pitch, msg.robot_yaw))

        # Prune entries older than buffer duration
        cutoff = t - self._pose_buffer_duration
        while self._pose_buffer and self._pose_buffer[0][0] < cutoff:
            self._pose_buffer.popleft()

        # Hard cap safety
        while len(self._pose_buffer) > self._pose_buffer_max:
            self._pose_buffer.popleft()

        # Still update latest pose (fallback and debug use)
        self.robot_pos = pos
        self.robot_roll = msg.robot_roll
        self.robot_pitch = msg.robot_pitch
        self.robot_yaw = msg.robot_yaw

        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info(
                f'First pose received: robot=({msg.robot_x:.2f}, {msg.robot_y:.2f}, '
                f'{msg.robot_z:.2f}), roll={msg.robot_roll:.1f}°, pitch={msg.robot_pitch:.1f}°, yaw={msg.robot_yaw:.1f}°')

    def _front_left_cb(self, msg: DetectionArray):
        self._on_detection(msg, 'front_left')

    def _front_right_cb(self, msg: DetectionArray):
        self._on_detection(msg, 'front_right')

    def _down_left_cb(self, msg: DetectionArray):
        self._on_detection(msg, 'down_left')

    def _down_right_cb(self, msg: DetectionArray):
        self._on_detection(msg, 'down_right')

    def _on_detection(self, msg: DetectionArray, camera: str):
        # Track arrival count per camera
        n = self._det_msg_count.get(camera, 0)
        if n == 0:
            self.get_logger().info(
                f'First detection msg from {camera}: '
                f'{len(msg.detections)} detections')
            # Start debug summary timer on first detection
            if self._debug_timer is None:
                self._debug_timer = self.create_timer(3.0, self._debug_summary)
        self._det_msg_count[camera] = n + 1

        self._process_detection(msg, camera)

    def _lookup_pose(self, stamp) -> tuple:
        """Find pose closest in time to the given stamp using binary search.

        Args:
            stamp: builtin_interfaces/Time with sec, nanosec fields.

        Returns:
            (pos_array, roll, pitch, yaw) — time-aligned pose,
            or latest pose if buffer is empty.
        """
        if not self._pose_buffer:
            return self.robot_pos, self.robot_roll, self.robot_pitch, self.robot_yaw

        t_target = stamp.sec + stamp.nanosec * 1e-9
        timestamps = [e[0] for e in self._pose_buffer]
        idx = bisect.bisect_left(timestamps, t_target)

        # Check idx-1 and idx for closest match
        candidates = []
        if idx > 0:
            candidates.append(self._pose_buffer[idx - 1])
        if idx < len(self._pose_buffer):
            candidates.append(self._pose_buffer[idx])

        best = min(candidates, key=lambda e: abs(e[0] - t_target))
        return best[1], best[2], best[3], best[4]

    def _associate_ray(self, class_id: int, camera_pair: str,
                       ray_origin: np.ndarray, v_world: np.ndarray) -> int:
        """Assign a ray to an existing instance or create a new one.

        Single-instance classes always use instance 0.
        Multi-instance classes use a cascaded strategy:
          1. Match against known 3D positions by angle.
          2. If no position match, try to fill incomplete instances
             (those with rays but no position yet) by comparing ray directions.
          3. If still no match, create a new instance.

        Returns:
            instance_id (int): 0-based per (class_id, camera_pair).
        """
        if class_id not in MULTI_INSTANCE_CLASSES:
            return 0

        pair_key = (class_id, camera_pair)
        threshold_rad = math.radians(self._angle_match_threshold)
        cos_threshold = math.cos(threshold_rad)

        # Gather all existing instance IDs for this class/camera pair
        all_instance_ids = set()
        positions: dict[int, np.ndarray] = {}  # iid -> position
        for (cid, cp, iid), data in self.object_memory.items():
            if cid == class_id and cp == camera_pair:
                all_instance_ids.add(iid)
                positions[iid] = data['position']
        for (cid, cp, iid) in self.ray_history:
            if cid == class_id and cp == camera_pair:
                all_instance_ids.add(iid)

        # Phase 1: no instances at all → start from 0
        if not all_instance_ids:
            return 0

        # Phase 2: match against known 3D positions
        best_pos_match = None
        best_pos_cos = -2.0  # cos(angle), higher is closer
        for iid, obj_pos in positions.items():
            dir_to_obj = obj_pos - ray_origin
            dist = float(np.linalg.norm(dir_to_obj))
            if dist < 1e-6:
                return iid
            cos_angle = float(np.dot(v_world, dir_to_obj / dist))
            if cos_angle > best_pos_cos:
                best_pos_cos = cos_angle
                best_pos_match = iid

        if best_pos_match is not None and best_pos_cos > cos_threshold:
            return best_pos_match

        # Phase 3: no position match — try to fill incomplete instances
        # (those with rays in ray_history but no 3D position yet)
        incomplete: dict[int, list] = {}
        for iid in all_instance_ids:
            if iid not in positions:
                key = (class_id, camera_pair, iid)
                if key in self.ray_history:
                    incomplete[iid] = self.ray_history[key]

        if incomplete:
            # Try to match by ray direction similarity
            best_ray_match = None
            best_ray_cos = -2.0
            for iid, rays in incomplete.items():
                for r in rays:
                    cos = float(np.dot(v_world, r['direction']))
                    if cos > best_ray_cos:
                        best_ray_cos = cos
                        best_ray_match = iid

            if best_ray_match is not None and best_ray_cos > cos_threshold:
                return best_ray_match

            # No clear match — fill the instance with fewest rays first
            fewest_iid = min(incomplete.keys(),
                             key=lambda i: len(incomplete[i]))
            return fewest_iid

        # Phase 4: all existing instances have positions but none matched
        # → this is genuinely a new object
        new_id = self._next_instance_id.get(pair_key, 0)
        self._next_instance_id[pair_key] = new_id + 1
        return new_id

    def _process_detection(self, msg: DetectionArray, camera: str):
        """Process detection results: generate rays and intersect.

        camera: one of 'front_left', 'front_right', 'down_left', 'down_right'.
        Rays are grouped by (class_id, camera_pair) where camera_pair is 'front' or 'down'
        so left and right channels of the same stereo pair contribute to the same queue.
        """
        cp = self._cam_params[camera]
        fx, fy = cp['fx'], cp['fy']
        cx, cy = cp['cx'], cp['cy']
        offset = cp['offset']
        optical_to_body = cp['optical_to_body']

        # camera_pair for ray grouping: 'front' or 'down'
        camera_pair = 'front' if camera.startswith('front') else 'down'

        # Look up pose at detection time (time-aligned, not latest)
        det_pos, det_roll, det_pitch, det_yaw = self._lookup_pose(msg.header.stamp)

        # Robot rotation matrix (world NED) — full 6-DOF attitude
        R_robot = _euler_to_rotation_matrix(det_roll, det_pitch, det_yaw)

        # Precompute Gaussian decay sigma for pixel weighting
        R_max = math.hypot(cp['width'] / 2.0, cp['height'] / 2.0)
        sigma = R_max * 0.5

        for det in msg.detections:
            # Pixel to camera ray
            px, py = det.pixel_x, det.pixel_y

            # Pixel weight: Gaussian decay from optical center (edge pixels less reliable)
            dist_to_center = math.hypot(px - cx, py - cy)
            weight = math.exp(-0.5 * (dist_to_center / sigma) ** 2)
            weight = max(0.1, weight)

            v_cam = np.array([(px - cx) / fx, (py - cy) / fy, 1.0])
            v_cam = v_cam / np.linalg.norm(v_cam)

            # Camera to body NED
            v_body = optical_to_body @ v_cam

            # Body to world NED
            v_world = R_robot @ v_body
            v_world = v_world / np.linalg.norm(v_world)

            # Ray origin: robot position + camera offset in world frame
            offset_world = R_robot @ offset
            ray_origin = det_pos + offset_world

            # Manage ray history — keyed by (class_id, camera_pair, instance_id)
            class_id = det.class_id
            instance_id = self._associate_ray(
                class_id, camera_pair, ray_origin, v_world)
            key = (class_id, camera_pair, instance_id)
            history = self.ray_history.setdefault(key, [])

            # Check baseline distance
            if history:
                last_origin = history[-1]['origin']
                dist = np.linalg.norm(ray_origin - last_origin)
                if dist < MIN_BASELINE:
                    self._ray_skipped_baseline += 1
                    continue  # Too close to last ray

            # Add ray
            self._ray_added_total += 1
            history.append({
                'origin': ray_origin,
                'direction': v_world,
                'weight': weight,
            })

            # Prune if too many
            if len(history) > self._max_history:
                self._prune_rays(history)

            # Try intersection
            if len(history) >= MIN_RAYS_FOR_INTERSECTION:
                self._intersect_attempts += 1
                position = self._intersect_rays_pairwise(history)
                if position is None:
                    self._intersect_fails += 1
                else:
                    # Validate distance
                    dist = np.linalg.norm(position - det_pos)
                    if dist < MAX_DISTANCE:
                        alpha = 0.3  # EMA smoothing factor
                        is_new = (key not in self.object_memory)
                        if is_new:
                            self.object_memory[key] = {
                                'position': position,
                                'confidence': det.confidence,
                                'observations': len(history),
                            }
                        else:
                            old_pos = self.object_memory[key]['position']
                            smoothed_pos = old_pos * (1.0 - alpha) + position * alpha
                            self.object_memory[key] = {
                                'position': smoothed_pos,
                                'confidence': det.confidence,
                                'observations': len(history),
                            }
                        if is_new:
                            cls_name = CLASS_NAMES.get(class_id, str(class_id))
                            self.get_logger().info(
                                f'New object: {cls_name} (id={class_id}) '
                                f'via {camera_pair}, '
                                f'pos=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}), '
                                f'dist={dist:.2f}m, rays={len(history)}')

    def _prune_rays(self, history: list):
        """Prune redundant rays using weighted pairwise scoring.

        Edge rays (low pixel weight) are penalized and pruned first.
        """
        if len(history) <= self._max_history:
            return

        # Score each ray by redundancy with others
        scores = []
        for i, ri in enumerate(history):
            score = 0.0
            wi = ri.get('weight', 1.0)
            for j, rj in enumerate(history):
                if i == j:
                    continue
                cos_sim = abs(np.dot(ri['direction'], rj['direction']))
                dist = np.linalg.norm(ri['origin'] - rj['origin'])
                score += cos_sim / (dist + 1e-3)

            # Edge rays (low weight) get amplified score → pruned first
            score = score / wi
            scores.append(score)

        # Remove highest redundancy ray
        worst_idx = np.argmax(scores)
        history.pop(worst_idx)

    def _intersect_rays_pairwise(self, rays: list) -> np.ndarray | None:
        """Weighted pairwise ray intersection with outlier rejection.

        Pair weight = w1 * w2.  Rough median filters outliers > 1.5m,
        then weighted average of inliers.
        """
        intersections = []
        pair_weights = []

        for i in range(len(rays)):
            for j in range(i + 1, len(rays)):
                p1, d1 = rays[i]['origin'], rays[i]['direction']
                p2, d2 = rays[j]['origin'], rays[j]['direction']

                # Check parallel
                cos_angle = abs(np.dot(d1, d2))
                if cos_angle > 0.9995:
                    continue

                # Common perpendicular
                w = p1 - p2
                a = np.dot(d1, d1)
                b = np.dot(d1, d2)
                c = np.dot(d2, d2)
                d = np.dot(d1, w)
                e = np.dot(d2, w)

                denom = a * c - b * b
                if abs(denom) < 1e-10:
                    continue

                t1 = (b * e - c * d) / denom
                t2 = (a * e - b * d) / denom

                # Both must be forward
                if t1 <= 0 or t2 <= 0:
                    continue

                # Midpoint
                mid1 = p1 + t1 * d1
                mid2 = p2 + t2 * d2
                mid = (mid1 + mid2) / 2.0

                # Check perpendicular distance
                perp_dist = np.linalg.norm(mid1 - mid2)
                if perp_dist > 5.0:
                    continue

                w1 = rays[i].get('weight', 1.0)
                w2 = rays[j].get('weight', 1.0)

                intersections.append(mid)
                pair_weights.append(w1 * w2)

        if len(intersections) == 0:
            return self._intersect_rays_3d(rays)

        intersections = np.array(intersections)
        pair_weights = np.array(pair_weights)

        # Rough median → reject outliers > 1.5m from median
        rough_median = np.median(intersections, axis=0)
        dists = np.linalg.norm(intersections - rough_median, axis=1)
        valid_mask = dists < 1.5

        if np.sum(valid_mask) == 0:
            return rough_median

        # Weighted average of inliers
        return np.average(intersections[valid_mask], axis=0,
                          weights=pair_weights[valid_mask])

    def _intersect_rays_3d(self, rays: list) -> np.ndarray | None:
        """Weighted least-squares 3D ray intersection fallback."""
        if len(rays) < 2:
            return None

        # Build system: sum w_i * (I - d_i * d_i^T) * p = sum w_i * (I - d_i * d_i^T) * o_i
        A = np.zeros((3, 3))
        b = np.zeros(3)

        for ray in rays:
            o = ray['origin']
            d = ray['direction']
            w = ray.get('weight', 1.0)
            d = d / np.linalg.norm(d)
            P = np.eye(3) - np.outer(d, d)
            A += w * P
            b += w * (P @ o)

        try:
            cond = np.linalg.cond(A)
            if cond > 1e5:
                return None
            return np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            return None

    def _debug_summary(self):
        """Periodic debug: log detection rates, ray history, object count."""
        det_info = ', '.join(
            f'{cam}={n}' for cam, n in sorted(self._det_msg_count.items()))
        ray_info = ', '.join(
            f'{CLASS_NAMES.get(cid, cid)}({cid})#{iid}/{cp}={len(h)}'
            for (cid, cp, iid), h in sorted(self.ray_history.items()))
        total_rays = sum(len(h) for h in self.ray_history.values())
        self.get_logger().info(
            f'[DEBUG] pose_ok={self._pose_received}, '
            f'robot_pos=({self.robot_pos[0]:.1f}, {self.robot_pos[1]:.1f}, {self.robot_pos[2]:.1f}) '
            f'rpy=({self.robot_roll:.1f}°, {self.robot_pitch:.1f}°, {self.robot_yaw:.1f}°), '
            f'objects={len(self.object_memory)}, '
            f'total_rays={total_rays} '
            f'(added={self._ray_added_total}, '
            f'skipped_base={self._ray_skipped_baseline}), '
            f'intersect: attempts={self._intersect_attempts} '
            f'fails={self._intersect_fails}, '
            f'history_keys={len(self.ray_history)}, '
            f'det_msgs=[{det_info}], '
            f'rays=[{ray_info or "(none)"}]')

    def _broadcast_objects(self):
        """Publish tracked object positions at 10Hz."""
        msg = ObjectPositionArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        for (class_id, camera_pair, instance_id), data in self.object_memory.items():
            obj = ObjectPosition()
            obj.class_id = class_id
            obj.instance_id = instance_id
            obj.class_name = CLASS_NAMES.get(class_id, str(class_id))
            obj.world_x = float(data['position'][0])
            obj.world_y = float(data['position'][1])
            obj.world_z = float(data['position'][2])
            obj.confidence = float(data['confidence'])
            obj.num_observations = int(data['observations'])
            msg.objects.append(obj)

        self.pub_objects.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
