"""
warehouse_robot.vision_node
============================
ROS2 node: vision_node

Detects ArUco markers in /image_raw, applies temporal
filtering (3 consecutive frames), and publishes confirmed
tasks as JSON on /detected_markers.

Run:  ros2 run warehouse_robot vision_node
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge

# Marker ID → task metadata
TASK_DB = {
    0: {'task_id': 'A', 'weight': 10.0, 'priority': 3, 'label': 'HIGH'},
    1: {'task_id': 'B', 'weight':  2.0, 'priority': 1, 'label': 'LOW'},
    2: {'task_id': 'C', 'weight':  7.0, 'priority': 2, 'label': 'MEDIUM'},
    3: {'task_id': 'D', 'weight':  5.0, 'priority': 3, 'label': 'HIGH'},
}

MARKER_SIZE = 0.20  # metres – must match physical marker

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('confirm_frames',       3)
        self.declare_parameter('max_marker_dist',      3.0)
        self.declare_parameter('reproj_error_thresh',  2.5)
        self.declare_parameter('debug_image',          True)
        self.declare_parameter('frame_id',             'camera_optical_link')

        self._confirm   = self.get_parameter('confirm_frames').value
        self._max_dist  = self.get_parameter('max_marker_dist').value
        self._reproj_th = self.get_parameter('reproj_error_thresh').value
        self._debug     = self.get_parameter('debug_image').value

        # ArUco detector (OpenCV 4.7+)
        aruco_dict    = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        aruco_params  = aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self._detector = aruco.ArucoDetector(aruco_dict, aruco_params)

        # Camera intrinsics (updated from /camera_info)
        self._K = np.array([
            [554.25, 0.0,    320.0],
            [0.0,    554.25, 240.0],
            [0.0,    0.0,    1.0  ]], dtype=np.float64)
        self._D = np.zeros((5, 1), dtype=np.float64)

        # Temporal filter state
        self._cnt:       dict[int, int] = {}   # marker_id → consecutive frames
        self._confirmed: set[int]       = set()

        self._bridge = CvBridge()

        # Subscribers
        self.create_subscription(Image,      '/image_raw',   self._image_cb,  SENSOR_QOS)
        self.create_subscription(CameraInfo, '/camera_info', self._caminfo_cb, SENSOR_QOS)

        # Publishers
        self._pub_markers = self.create_publisher(String, '/detected_markers', RELIABLE_QOS)
        self._pub_debug   = self.create_publisher(Image,  '/vision_debug',     SENSOR_QOS)

        self.get_logger().info('[vision_node] Ready — DICT_6X6_250 detection active')

    # ── Camera info ──────────────────────────────────────────
    def _caminfo_cb(self, msg: CameraInfo) -> None:
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    # ── Main image callback ──────────────────────────────────
    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'CvBridge: {exc}')
            return

        # CLAHE normalisation for lighting robustness
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray  = clahe.apply(gray)

        corners, ids, _ = self._detector.detectMarkers(gray)

        seen_ids  = set()
        task_list = []

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, self._K, self._D)

            for i, raw_id in enumerate(ids.flatten()):
                mid  = int(raw_id)
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                dist = float(np.linalg.norm(tvec))

                if dist > self._max_dist:
                    continue
                if self._reproj_err(corners[i], rvec, tvec) > self._reproj_th:
                    continue

                seen_ids.add(mid)
                self._cnt[mid] = self._cnt.get(mid, 0) + 1

                if self._cnt[mid] >= self._confirm and mid not in self._confirmed:
                    self._confirmed.add(mid)
                    self.get_logger().info(
                        f'[vision_node] Marker {mid} confirmed '
                        f'(dist={dist:.2f} m)')

                if mid in self._confirmed:
                    meta = TASK_DB.get(mid, {
                        'task_id': f'X{mid}', 'weight': 1.0,
                        'priority': 1, 'label': 'LOW'})
                    task_list.append({
                        'marker_id': mid,
                        'task_id':   meta['task_id'],
                        'weight':    meta['weight'],
                        'priority':  meta['priority'],
                        'label':     meta['label'],
                        'distance':  round(dist, 3),
                        'tvec':      tvec.tolist(),
                        'rvec':      rvec.tolist(),
                    })

                if self._debug:
                    cv2.aruco.drawDetectedMarkers(
                        frame, [corners[i]], np.array([[mid]]))
                    cv2.drawFrameAxes(
                        frame, self._K, self._D, rvec, tvec, MARKER_SIZE * 0.5)
                    cv2.putText(
                        frame,
                        f'ID:{mid} {dist:.2f}m',
                        (int(corners[i][0][0][0]),
                         int(corners[i][0][0][1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Reset counters for unseen markers
        for mid in list(self._cnt):
            if mid not in seen_ids:
                self._cnt[mid] = 0

        if task_list:
            payload = json.dumps({
                'stamp': self.get_clock().now().nanoseconds,
                'tasks': task_list,
            })
            self._pub_markers.publish(String(data=payload))

        if self._debug:
            try:
                dbg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                dbg.header = msg.header
                self._pub_debug.publish(dbg)
            except Exception:
                pass

    def _reproj_err(self, corners, rvec, tvec) -> float:
        half = MARKER_SIZE / 2.0
        obj  = np.array([[-half, half, 0], [half, half, 0],
                          [half, -half, 0], [-half, -half, 0]],
                         dtype=np.float32)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self._K, self._D)
        return float(np.mean(
            np.linalg.norm(proj.reshape(4, 2) - corners[0].astype(np.float32), axis=1)))


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
