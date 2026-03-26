"""
warehouse_robot.camera_node
============================
ROS2 node: camera_node

Run:  ros2 run warehouse_robot camera_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_width',  640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps',          30.0)
        self.declare_parameter('frame_id',     'camera_optical_link')
        self.declare_parameter('sim_topic',    '/rgb_camera/image_raw')

        self._frame_id = self.get_parameter('frame_id').value
        self._fps      = self.get_parameter('fps').value
        use_sim        = self.get_parameter('use_sim_time').value

        self._pub_image = self.create_publisher(Image,      '/image_raw',   SENSOR_QOS)
        self._pub_info  = self.create_publisher(CameraInfo, '/camera_info', SENSOR_QOS)
        self._bridge    = CvBridge()
        self._build_camera_info()

        if use_sim:
            sim_topic = self.get_parameter('sim_topic').value
            self._sub = self.create_subscription(Image, sim_topic, self._sim_cb, SENSOR_QOS)
            self.get_logger().info(f'[camera_node] SIM mode → relaying {sim_topic}')
        else:
            idx = self.get_parameter('camera_index').value
            self._cap = cv2.VideoCapture(idx)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.get_parameter('image_width').value)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('image_height').value)
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)
            self.create_timer(1.0 / self._fps, self._hw_cb)
            self.get_logger().info(f'[camera_node] HW mode → VideoCapture({idx})')

    def _sim_cb(self, msg: Image) -> None:
        msg.header.frame_id = self._frame_id
        self._pub_image.publish(msg)
        self._pub_info.publish(self._make_info(msg.header))

    def _hw_cb(self) -> None:
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('[camera_node] Frame grab failed')
            return
        stamp  = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._frame_id)
        img    = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img.header = header
        self._pub_image.publish(img)
        self._pub_info.publish(self._make_info(header))

    def _build_camera_info(self) -> None:
        fx = fy = 554.254691191187
        cx, cy  = 320.0, 240.0
        ci = CameraInfo()
        ci.width  = 640
        ci.height = 480
        ci.distortion_model = 'plumb_bob'
        ci.d = [0.0] * 5
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._ci_template = ci

    def _make_info(self, header: Header) -> CameraInfo:
        ci = CameraInfo()
        ci.header = header
        ci.width  = self._ci_template.width
        ci.height = self._ci_template.height
        ci.distortion_model = self._ci_template.distortion_model
        ci.d = list(self._ci_template.d)
        ci.k = list(self._ci_template.k)
        ci.r = list(self._ci_template.r)
        ci.p = list(self._ci_template.p)
        return ci

    def destroy_node(self) -> None:
        if hasattr(self, '_cap'):
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
