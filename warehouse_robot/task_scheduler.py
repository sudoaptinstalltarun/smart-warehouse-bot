"""
warehouse_robot.task_scheduler
================================
ROS2 node: task_scheduler   ← CORE INNOVATION

Scoring formula:
    score = 0.6 * distance + 0.2 * weight - 0.5 * priority
Lower score → higher execution priority.

Dynamic replanning: re-evaluates queue whenever a new task arrives.
Replans in < 200 ms.

Run:  ros2 run warehouse_robot task_scheduler
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

import tf_transformations

# ── Scoring weights ───────────────────────────────────────────
W_DIST = 0.6
W_WT   = 0.2
W_PRI  = 0.5

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)


# ── Data class ────────────────────────────────────────────────
class Task:
    __slots__ = ('marker_id', 'task_id', 'weight', 'priority',
                 'distance', 'tvec', 'score', 'status')

    def __init__(self, marker_id, task_id, weight, priority, distance, tvec):
        self.marker_id = marker_id
        self.task_id   = task_id
        self.weight    = weight
        self.priority  = priority
        self.distance  = distance
        self.tvec      = tvec
        self.status    = 'PENDING'
        self.score     = self._calc()

    def _calc(self):
        return W_DIST * self.distance + W_WT * self.weight - W_PRI * self.priority

    def update_dist(self, d):
        self.distance = d
        self.score    = self._calc()

    def __repr__(self):
        return (f'Task({self.task_id} mid={self.marker_id} '
                f'w={self.weight}kg p={self.priority} '
                f'd={self.distance:.2f}m → score={self.score:.3f})')


# ── Node ─────────────────────────────────────────────────────
class TaskScheduler(Node):

    def __init__(self):
        super().__init__('task_scheduler')

        self.declare_parameter('replan_tolerance', 0.05)
        self.declare_parameter('map_frame',        'map')
        self.declare_parameter('base_frame',       'base_footprint')
        self.declare_parameter('marker_map_file',  '')

        self._replan_tol  = self.get_parameter('replan_tolerance').value
        self._map_frame   = self.get_parameter('map_frame').value

        self._queue:       dict[int, Task] = {}
        self._active:      Task | None     = None
        self._robot_pose:  PoseStamped | None = None
        self._estop:       bool            = False
        self._marker_map:  dict[int, tuple] = {}

        self._load_marker_map()

        # Nav2 action client
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.create_subscription(String,   '/detected_markers', self._markers_cb, RELIABLE_QOS)
        self.create_subscription(Odometry, '/odom',             self._odom_cb,    SENSOR_QOS)
        self.create_subscription(Bool,     '/emergency_stop',   self._estop_cb,   RELIABLE_QOS)

        # Publishers
        self._pub_goal   = self.create_publisher(PoseStamped, '/goal_pose',    RELIABLE_QOS)
        self._pub_status = self.create_publisher(String,      '/task_status',  RELIABLE_QOS)
        self._pub_viz    = self.create_publisher(MarkerArray, '/task_queue_viz', RELIABLE_QOS)

        self.create_timer(0.5, self._scheduler_loop)
        self.create_timer(1.0, self._publish_viz)

        self.get_logger().info('[task_scheduler] Ready — waiting for tasks...')

    # ── Load prebuilt marker→pose map ────────────────────────
    def _load_marker_map(self):
        import os
        path = self.get_parameter('marker_map_file').value
        if path and os.path.exists(path):
            with open(path) as f:
                for e in json.load(f):
                    self._marker_map[e['marker_id']] = (
                        e['x'], e['y'], e.get('yaw', 0.0))
            self.get_logger().info(
                f'[task_scheduler] Loaded {len(self._marker_map)} marker positions')

    # ── Callbacks ────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose   = msg.pose.pose
        self._robot_pose = ps

    def _estop_cb(self, msg: Bool):
        self._estop = msg.data
        if msg.data:
            self.get_logger().warn('[task_scheduler] E-STOP received')

    def _markers_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON error: {e}')
            return

        new_task = False
        for t in data.get('tasks', []):
            mid = t['marker_id']
            if mid in self._queue and self._queue[mid].status == 'DONE':
                continue
            if mid not in self._queue:
                task = Task(mid, t['task_id'], t['weight'],
                            t['priority'], t['distance'], t['tvec'])
                self._queue[mid] = task
                self.get_logger().info(f'[task_scheduler] NEW → {task}')
                new_task = True
            else:
                self._queue[mid].update_dist(t['distance'])

        if new_task:
            self._check_replan()

    # ── Scheduler loop (2 Hz) ────────────────────────────────
    def _scheduler_loop(self):
        if self._estop:
            return
        pending = [t for t in self._queue.values() if t.status == 'PENDING']
        if not pending or self._active is not None:
            if not pending and self._active is None:
                self.get_logger().info(
                    '[task_scheduler] Queue empty — idle',
                    throttle_duration_sec=10.0)
            return
        self._dispatch(pending)

    def _dispatch(self, pending: list):
        ranked = sorted(pending, key=lambda t: t.score)
        best   = ranked[0]
        self.get_logger().info(f'[task_scheduler] DISPATCHING {best}')
        self._log_queue(ranked)
        best.status  = 'ACTIVE'
        self._active = best
        self._send_nav_goal(best)
        self._publish_status()

    def _check_replan(self):
        if self._active is None:
            return
        pending = [t for t in self._queue.values() if t.status == 'PENDING']
        if not pending:
            return
        best   = min(pending, key=lambda t: t.score)
        delta  = self._active.score - best.score
        if delta > self._replan_tol:
            self.get_logger().info(
                f'[task_scheduler] REPLAN: {best.task_id} '
                f'(score={best.score:.3f}) preempts '
                f'{self._active.task_id} (score={self._active.score:.3f}) '
                f'Δ={delta:.3f}')
            self._active.status = 'PENDING'
            self._active = None
            all_pending = [t for t in self._queue.values() if t.status == 'PENDING']
            self._dispatch(all_pending)

    # ── Nav2 goal ────────────────────────────────────────────
    def _send_nav_goal(self, task: Task):
        goal_pose = self._compute_goal(task)
        if goal_pose is None:
            self.get_logger().error(
                f'[task_scheduler] Cannot compute goal for {task.task_id}')
            task.status  = 'PENDING'
            self._active = None
            return

        self._pub_goal.publish(goal_pose)

        if not self._nav.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(
                '[task_scheduler] Nav2 not ready — using /goal_pose topic only')
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        fut = self._nav.send_goal_async(nav_goal, feedback_callback=self._nav_feedback)
        fut.add_done_callback(self._nav_goal_response)

    def _nav_feedback(self, fb):
        self.get_logger().debug(
            f'[task_scheduler] Nav2 dist_remaining='
            f'{fb.feedback.distance_remaining:.2f}m',
            throttle_duration_sec=2.0)

    def _nav_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('[task_scheduler] Nav2 rejected goal')
            if self._active:
                self._active.status = 'PENDING'
                self._active = None
            return
        gh.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        if self._active:
            self.get_logger().info(
                f'[task_scheduler] DONE: {self._active.task_id}')
            self._active.status = 'DONE'
            self._active = None
            self._publish_status()

    # ── Goal pose computation ────────────────────────────────
    def _compute_goal(self, task: Task) -> PoseStamped | None:
        # Prefer preloaded map position
        if task.marker_id in self._marker_map:
            x, y, yaw = self._marker_map[task.marker_id]
            ax = x - 0.4 * math.cos(yaw)
            ay = y - 0.4 * math.sin(yaw)
            return self._pose(ax, ay, yaw)

        # Fallback: estimate from tvec + odometry
        if self._robot_pose is None:
            return None
        rx = self._robot_pose.pose.position.x
        ry = self._robot_pose.pose.position.y
        _, _, ryaw = tf_transformations.euler_from_quaternion([
            self._robot_pose.pose.orientation.x,
            self._robot_pose.pose.orientation.y,
            self._robot_pose.pose.orientation.z,
            self._robot_pose.pose.orientation.w,
        ])
        dx = task.tvec[2] * math.cos(ryaw) - task.tvec[0] * math.sin(ryaw)
        dy = task.tvec[2] * math.sin(ryaw) + task.tvec[0] * math.cos(ryaw)
        gx = rx + dx - 0.4 * math.cos(ryaw)
        gy = ry + dy - 0.4 * math.sin(ryaw)
        return self._pose(gx, gy, ryaw)

    def _pose(self, x, y, yaw) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = self._map_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

    # ── Helpers ──────────────────────────────────────────────
    def _log_queue(self, ranked: list):
        rows = [f"  {'Task':>6} {'Score':>7} {'Dist':>6} {'Wt':>5} {'Pri':>4}"]
        for t in ranked:
            rows.append(f'  {t.task_id:>6} {t.score:>7.3f} '
                        f'{t.distance:>5.2f}m {t.weight:>4.1f}kg {t.priority:>4}')
        self.get_logger().info('Queue:\n' + '\n'.join(rows))

    def _publish_status(self):
        payload = json.dumps({
            'active':  self._active.task_id if self._active else None,
            'pending': [t.task_id for t in self._queue.values() if t.status == 'PENDING'],
            'done':    [t.task_id for t in self._queue.values() if t.status == 'DONE'],
        })
        self._pub_status.publish(String(data=payload))

    def _publish_viz(self):
        ma = MarkerArray()
        for i, task in enumerate(self._queue.values()):
            m         = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns      = 'task_queue'
            m.id      = i
            m.type    = Marker.TEXT_VIEW_FACING
            m.action  = Marker.ADD
            m.scale.z = 0.3
            m.pose.position.z = 1.5
            if task.status == 'DONE':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 0.8
            elif task.status == 'ACTIVE':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 1.0
            m.text = f'Task {task.task_id}\nscore={task.score:.2f}\n{task.status}'
            if task.marker_id in self._marker_map:
                x, y, _ = self._marker_map[task.marker_id]
                m.pose.position.x = x
                m.pose.position.y = y
            ma.markers.append(m)
        self._pub_viz.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
