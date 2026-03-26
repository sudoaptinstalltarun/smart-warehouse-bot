"""
warehouse_robot.robot_controller
==================================
ROS2 node: robot_controller

Safety layer between Nav2 and the robot base:
  - Obstacle detection via /scan → E-stop at < 0.30 m
  - Velocity clamping (linear 0.26 m/s, angular 0.80 rad/s)
  - Vision watchdog: zeros cmd_vel if /image_raw stalls > 3 s

Run:  ros2 run warehouse_robot robot_controller
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('stop_distance',          0.30)
        self.declare_parameter('slow_distance',          0.60)
        self.declare_parameter('max_linear_vel',         0.26)
        self.declare_parameter('max_angular_vel',        0.80)
        self.declare_parameter('vision_watchdog_timeout', 3.0)

        self._stop_d  = self.get_parameter('stop_distance').value
        self._slow_d  = self.get_parameter('slow_distance').value
        self._max_lin = self.get_parameter('max_linear_vel').value
        self._max_ang = self.get_parameter('max_angular_vel').value
        self._wd_tout = self.get_parameter('vision_watchdog_timeout').value

        self._estop   = False
        self._slow    = False
        self._last_img_t = time.time()

        # Subscribers
        self.create_subscription(LaserScan, '/scan',        self._scan_cb, SENSOR_QOS)
        self.create_subscription(Twist,     '/cmd_vel_nav', self._cmd_cb,  RELIABLE_QOS)
        # Vision watchdog subscriber (just timestamps)
        self.create_subscription(
            'sensor_msgs/msg/Image', '/image_raw', self._img_wd_cb, SENSOR_QOS)

        # Publishers
        self._pub_cmd   = self.create_publisher(Twist, '/cmd_vel',        RELIABLE_QOS)
        self._pub_estop = self.create_publisher(Bool,  '/emergency_stop', RELIABLE_QOS)

        self.create_timer(0.5, self._watchdog_tick)

        self.get_logger().info(
            f'[robot_controller] Ready | '
            f'stop={self._stop_d}m slow={self._slow_d}m '
            f'max_lin={self._max_lin}m/s max_ang={self._max_ang}rad/s')

    # ── LaserScan → obstacle detection ───────────────────────
    def _scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid:
            return
        min_r = min(valid)

        if min_r < self._stop_d:
            if not self._estop:
                self.get_logger().warn(
                    f'[robot_controller] OBSTACLE {min_r:.2f}m — E-STOP')
            self._estop = True
            self._zero_vel()
            self._pub_estop.publish(Bool(data=True))
        else:
            if self._estop:
                self.get_logger().info('[robot_controller] Obstacle cleared — resuming')
            self._estop = False
            self._slow  = (min_r < self._slow_d)
            self._pub_estop.publish(Bool(data=False))

    # ── cmd_vel passthrough with clamping ─────────────────────
    def _cmd_cb(self, msg: Twist):
        if self._estop:
            self._zero_vel()
            return
        scale = 0.4 if self._slow else 1.0
        out   = Twist()
        out.linear.x  = max(-self._max_lin,
                             min(self._max_lin,  msg.linear.x  * scale))
        out.angular.z = max(-self._max_ang,
                             min(self._max_ang, msg.angular.z * scale))
        self._pub_cmd.publish(out)

    # ── Vision watchdog ───────────────────────────────────────
    def _img_wd_cb(self, _):
        self._last_img_t = time.time()

    def _watchdog_tick(self):
        elapsed = time.time() - self._last_img_t
        if elapsed > self._wd_tout:
            self.get_logger().warn(
                f'[robot_controller] Vision timeout ({elapsed:.1f}s) — zeroing cmd_vel',
                throttle_duration_sec=5.0)
            self._zero_vel()

    def _zero_vel(self):
        self._pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
