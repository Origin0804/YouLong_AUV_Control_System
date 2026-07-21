"""Vision node: camera image processing and YOLO detection.

Two modes (sim_mode parameter):
  sim_mode=true  (default): subscribe to /auv/*/stitched ROS topics
  sim_mode=false (real HW): capture from V4L2 devices via OpenCV

Stitched stereo images are split into left/right halves, YOLO runs on each
independently. Output: 4 detection channels + 4 debug image channels.

Parameters:
    model_path (str): Path to YOLO model .pt file.
    publish_images (bool): Whether to forward split images (default: false).
    sim_mode (bool): true=ROS images, false=V4L2 capture (default: true).
    front_cam_path (str): V4L2 device path for front camera (real mode only).
    down_cam_path (str): V4L2 device path for down camera (real mode only).
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from uv_msgs.msg import Detection, DetectionArray

CvBridge = None


def _import_cv_bridge():
    global CvBridge
    if CvBridge is None:
        try:
            from cv_bridge import CvBridge as _CB
            CvBridge = _CB
        except Exception as e:
            raise ImportError(f'cv_bridge not available: {e}')


class VisionNode(Node):
    """Vision node: YOLO detection on stitched camera images, split into L/R."""

    def __init__(self):
        super().__init__('vision')

        self.declare_parameter('publish_images', False)
        self._publish_images = self.get_parameter('publish_images').get_parameter_value().bool_value

        self.declare_parameter('publish_annotated', False)
        self._publish_annotated = self.get_parameter('publish_annotated').get_parameter_value().bool_value

        self.declare_parameter('sim_mode', True)
        self._sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value

        self.declare_parameter('save_dataset', False)
        self._save_dataset = self.get_parameter('save_dataset').get_parameter_value().bool_value
        self._dataset_dir = ''
        self._dataset_last_s = {}  # channel -> last save time (second bucket)
        self._dataset_count_s = {}  # channel -> count in current second
        if self._save_dataset:
            # img/ at same level as src/
            self._dataset_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'img')
            os.makedirs(self._dataset_dir, exist_ok=True)
            self.get_logger().info(f'Dataset saving enabled: {self._dataset_dir}')

        self.bridge = None
        self._model = None
        self._confidence = 0.9
        self._model_loaded = False
        self._cv_bridge_ok = False
        self._front_cap = None
        self._down_cap = None

        # Try to import cv_bridge
        try:
            _import_cv_bridge()
            self.bridge = CvBridge()
            self._cv_bridge_ok = True
        except Exception as e:
            self.get_logger().warn(f'cv_bridge not available: {e}')

        # Distortion correction — per camera pair (applied to each half after split)
        self._init_undistort()

        # Load YOLO model
        self._load_model()

        # Image source: ROS topics (sim) or V4L2 devices (real)
        if self._sim_mode:
            self.create_subscription(Image, '/auv/front_cam/stitched', self._front_img_cb, 10)
            self.create_subscription(Image, '/auv/down_cam/stitched', self._down_img_cb, 10)
            self.get_logger().info('Vision node started (sim mode: ROS topics)')
        else:
            self.declare_parameter('front_cam_path', '/dev/video0')
            self.declare_parameter('down_cam_path', '/dev/video2')
            front_path = self.get_parameter('front_cam_path').get_parameter_value().string_value
            down_path = self.get_parameter('down_cam_path').get_parameter_value().string_value

            self._front_cap = cv2.VideoCapture(front_path)
            self._down_cap = cv2.VideoCapture(down_path)
            if not self._front_cap.isOpened():
                self.get_logger().error(f'Failed to open front camera: {front_path}')
            if not self._down_cap.isOpened():
                self.get_logger().error(f'Failed to open down camera: {down_path}')

            # 10Hz capture timer
            self.create_timer(0.1, self._capture_real_cams)
            self.get_logger().info(
                f'Vision node started (real mode: front={front_path}, down={down_path})')

        # Publishers — 4 detection + 4 image (common to both modes)
        self.pub_det = {
            'front_left':  self.create_publisher(DetectionArray, '/perception/detection/front_left', 10),
            'front_right': self.create_publisher(DetectionArray, '/perception/detection/front_right', 10),
            'down_left':   self.create_publisher(DetectionArray, '/perception/detection/down_left', 10),
            'down_right':  self.create_publisher(DetectionArray, '/perception/detection/down_right', 10),
        }
        self.pub_img = {
            'front_left':  self.create_publisher(Image, '/perception/image/front_left', 10),
            'front_right': self.create_publisher(Image, '/perception/image/front_right', 10),
            'down_left':   self.create_publisher(Image, '/perception/image/down_left', 10),
            'down_right':  self.create_publisher(Image, '/perception/image/down_right', 10),
        }
        self.pub_annotated = {
            'front_left':  self.create_publisher(Image, '/perception/annotated/front_left', 10),
            'front_right': self.create_publisher(Image, '/perception/annotated/front_right', 10),
            'down_left':   self.create_publisher(Image, '/perception/annotated/down_left', 10),
            'down_right':  self.create_publisher(Image, '/perception/annotated/down_right', 10),
        }

    def _init_undistort(self):
        """Load camera matrix and distortion coefficients from parameters.

        Each camera pair (front/down) has a 3x3 camera matrix and distortion
        coefficients (k1,k2,p1,p2,k3).  Applied to each half after splitting.
        Defaults: identity matrix, zero distortion.
        """
        identity_3x3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        zero_5 = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.declare_parameter('front_camera_matrix', identity_3x3)
        self.declare_parameter('front_dist_coeffs', zero_5)
        self.declare_parameter('down_camera_matrix', identity_3x3)
        self.declare_parameter('down_dist_coeffs', zero_5)

        def _load_calib(prefix):
            K = np.array(self.get_parameter(f'{prefix}_camera_matrix')
                         .get_parameter_value().double_array_value,
                         dtype=np.float32).reshape(3, 3)
            D = np.array(self.get_parameter(f'{prefix}_dist_coeffs')
                         .get_parameter_value().double_array_value,
                         dtype=np.float32)
            return K, D

        self._front_K, self._front_D = _load_calib('front')
        self._down_K, self._down_D = _load_calib('down')

        self.get_logger().info(
            f'Undistort: front K={self._front_K.tolist()}, D={self._front_D.tolist()}')

    def _load_model(self):
        """Load YOLO model from parameter path or default locations."""
        try:
            from ultralytics import YOLO

            self.declare_parameter('model_path', '')
            model_path = self.get_parameter('model_path').get_parameter_value().string_value

            if not model_path:
                for candidate in [
                    os.path.expanduser('~/YouLong_AUV_Control_System/workspace_auv/src/datas/WUURC2026_sim_719.pt'),
                ]:
                    if os.path.exists(candidate):
                        model_path = candidate
                        break

            if model_path and os.path.exists(model_path):
                self._model = YOLO(model_path)
                self._model_loaded = True
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            else:
                self.get_logger().warn(f'YOLO model not found, detection disabled')
        except ImportError:
            self.get_logger().warn('ultralytics not installed, detection disabled')

    def _front_img_cb(self, msg: Image):
        self._process_image(msg, 'front')

    def _down_img_cb(self, msg: Image):
        self._process_image(msg, 'down')

    def _process_image(self, msg: Image, camera: str):
        """Split stitched image into left/right halves, run YOLO on each."""
        if not self._model_loaded or not self._cv_bridge_ok:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        K = self._front_K if camera == 'front' else self._down_K
        D = self._front_D if camera == 'front' else self._down_D

        h, w = cv_img.shape[:2]
        mid = w // 2
        left_img = cv2.undistort(cv_img[:, :mid], K, D)
        right_img = cv2.undistort(cv_img[:, mid:], K, D)

        left_name = f'{camera}_left'
        right_name = f'{camera}_right'

        # Save dataset frames (5 per second per channel)
        if self._save_dataset:
            self._save_frame(left_img, left_name)
            self._save_frame(right_img, right_name)

        # Run detection on left half
        det_left = self._detect(msg.header, left_name, left_img)
        self.pub_det[left_name].publish(det_left)
        if self._publish_images:
            self.pub_img[left_name].publish(self.bridge.cv2_to_imgmsg(left_img, 'bgr8'))
        if self._publish_annotated:
            annotated = self._draw_boxes(left_img, det_left)
            self.pub_annotated[left_name].publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

        # Run detection on right half
        det_right = self._detect(msg.header, right_name, right_img)
        self.pub_det[right_name].publish(det_right)
        if self._publish_images:
            self.pub_img[right_name].publish(self.bridge.cv2_to_imgmsg(right_img, 'bgr8'))
        if self._publish_annotated:
            annotated = self._draw_boxes(right_img, det_right)
            self.pub_annotated[right_name].publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

    def _draw_boxes(self, cv_img, det_array: DetectionArray):
        """Draw YOLO detection boxes and labels on image."""
        annotated = cv_img.copy()
        for det in det_array.detections:
            x1, y1 = int(det.bbox_x1), int(det.bbox_y1)
            x2, y2 = int(det.bbox_x2), int(det.bbox_y2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'{det.class_id}:{det.confidence:.2f}'
            cv2.putText(annotated, label, (x1, max(y1 - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return annotated

    def _detect(self, header, camera_name: str, cv_img) -> DetectionArray:
        """Run YOLO on a single image, return DetectionArray."""
        det_array = DetectionArray()
        det_array.header = header
        det_array.camera_name = camera_name

        try:
            results = self._model(cv_img, conf=self._confidence, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed ({camera_name}): {e}')
            return det_array

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                det = Detection()
                det.class_id = int(box.cls[0])
                det.confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.bbox_x1 = x1
                det.bbox_y1 = y1
                det.bbox_x2 = x2
                det.bbox_y2 = y2
                det.pixel_x = (x1 + x2) / 2.0
                det.pixel_y = (y1 + y2) / 2.0
                det_array.detections.append(det)

        return det_array


    def _save_frame(self, img, channel: str):
        """Save a dataset frame at up to 5 FPS per channel."""
        now_s = int(time.time())
        count = self._dataset_count_s.get(channel, 0)
        last_s = self._dataset_last_s.get(channel, 0)

        # Reset counter on new second
        if now_s != last_s:
            self._dataset_count_s[channel] = 0
            self._dataset_last_s[channel] = now_s
            count = 0

        if count >= 5:
            return

        self._dataset_count_s[channel] = count + 1
        ts = time.strftime('%Y%m%d%H%M%S') + f'{time.time() % 1:.6f}'[2:5]
        fname = os.path.join(self._dataset_dir, f'{channel}_{ts}.jpg')
        cv2.imwrite(fname, img)

    def _capture_real_cams(self):
        """Real mode: capture from V4L2 devices at 10Hz, feed to processing pipeline."""
        now = self.get_clock().now().to_msg()
        for cap, camera in [(self._front_cap, 'front'), (self._down_cap, 'down')]:
            if cap is None or not cap.isOpened():
                continue
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f'{camera} camera read failed', throttle_duration_sec=5.0)
                continue
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            img_msg.header.stamp = now
            self._process_image(img_msg, camera)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
