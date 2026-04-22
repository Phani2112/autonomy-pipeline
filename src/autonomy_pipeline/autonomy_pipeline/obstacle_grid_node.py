#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
obstacle_grid_node.py
---------------------
Perception -> local occupancy grid node for a Jetson-class mobile robot.

Subscribes:
    /scan   (sensor_msgs/LaserScan)   -- 2D LiDAR
    /odom   (nav_msgs/Odometry)       -- robot pose estimate (world->base)

Publishes:
    /local_costmap (nav_msgs/OccupancyGrid)   -- fused log-odds grid
    /obstacles     (visualization_msgs/MarkerArray) -- close-range obstacles

Design notes:
    * Log-odds Bayesian update per Thrun/Probabilistic Robotics.
    * Grid is body-centred (robot always at the middle cell). We do NOT try to
      build a full SLAM map here -- that is a separate component.
    * Free-space ray tracing uses Bresenham on the grid.
    * All magic numbers are ROS2 parameters (see config/params.yaml).
"""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (Z-axis rotation) from a quaternion. ZYX intrinsic."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def bresenham(x0: int, y0: int, x1: int, y1: int):
    """Yield (x, y) cells along the line, endpoint inclusive."""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ObstacleGridNode(Node):
    """2D occupancy grid builder from a single LiDAR."""

    def __init__(self) -> None:
        super().__init__('obstacle_grid_node')

        # -- Parameters (all overridable via YAML / CLI) --------------------
        self.declare_parameter('grid.resolution_m', 0.05)      # 5 cm/cell
        self.declare_parameter('grid.size_m', 10.0)            # 10m x 10m
        self.declare_parameter('grid.frame_id', 'odom')
        self.declare_parameter('robot.frame_id', 'base_link')
        self.declare_parameter('scan.topic', '/scan')
        self.declare_parameter('odom.topic', '/odom')
        self.declare_parameter('publish.grid_topic', '/local_costmap')
        self.declare_parameter('publish.markers_topic', '/obstacles')
        self.declare_parameter('publish.rate_hz', 5.0)
        self.declare_parameter('scan.max_range_m', 8.0)
        self.declare_parameter('scan.min_range_m', 0.10)
        # Log-odds -- conservative values that converge quickly but not
        # instantly so a single spurious return does not mark a cell.
        self.declare_parameter('logodds.l_occ', 0.85)          # ln(0.7/0.3)
        self.declare_parameter('logodds.l_free', -0.40)        # ln(0.4/0.6)
        self.declare_parameter('logodds.l_min', -2.0)
        self.declare_parameter('logodds.l_max', 3.5)
        self.declare_parameter('markers.danger_radius_m', 0.6)

        self._load_params()

        # -- Internal state -------------------------------------------------
        self._pose = Pose2D()
        self._have_odom = False
        self._have_scan = False
        self._log_odds = np.zeros((self._cells, self._cells), dtype=np.float32)

        # -- QoS: scan/odom from a real driver is BEST_EFFORT; subscribing
        #        with RELIABLE would silently drop messages. Match sensor QoS.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # -- I/O ------------------------------------------------------------
        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, sensor_qos)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_odom, sensor_qos)

        self._grid_pub = self.create_publisher(
            OccupancyGrid, self._grid_topic, latched_qos)
        self._marker_pub = self.create_publisher(
            MarkerArray, self._markers_topic, 10)

        self._timer = self.create_timer(
            1.0 / self._rate_hz, self._publish_outputs)

        self.get_logger().info(
            f"obstacle_grid_node up: grid {self._size_m}m @ {self._res}m "
            f"({self._cells}x{self._cells} cells), rate {self._rate_hz} Hz"
        )

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------
    def _load_params(self) -> None:
        gp = self.get_parameter
        self._res = float(gp('grid.resolution_m').value)
        self._size_m = float(gp('grid.size_m').value)
        self._cells = int(round(self._size_m / self._res))
        self._grid_frame = gp('grid.frame_id').value
        self._robot_frame = gp('robot.frame_id').value
        self._scan_topic = gp('scan.topic').value
        self._odom_topic = gp('odom.topic').value
        self._grid_topic = gp('publish.grid_topic').value
        self._markers_topic = gp('publish.markers_topic').value
        self._rate_hz = float(gp('publish.rate_hz').value)
        self._max_range = float(gp('scan.max_range_m').value)
        self._min_range = float(gp('scan.min_range_m').value)
        self._l_occ = float(gp('logodds.l_occ').value)
        self._l_free = float(gp('logodds.l_free').value)
        self._l_min = float(gp('logodds.l_min').value)
        self._l_max = float(gp('logodds.l_max').value)
        self._danger_r = float(gp('markers.danger_radius_m').value)

        # Sanity
        if self._cells <= 0 or self._cells > 2000:
            raise ValueError(
                f"Unreasonable grid size {self._cells} cells. "
                "Check grid.size_m / grid.resolution_m."
            )
        if self._l_occ <= 0 or self._l_free >= 0:
            self.get_logger().warn(
                "logodds signs look inverted -- l_occ should be >0, l_free <0"
            )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose.x = p.position.x
        self._pose.y = p.position.y
        self._pose.yaw = quaternion_to_yaw(
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w)
        self._have_odom = True

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._have_odom:
            # We need a pose to place returns in the world frame.
            self.get_logger().warn_once(
                "Scan received before any odometry -- waiting for odom")
            return

        self._integrate_scan(msg)
        self._have_scan = True

    # ------------------------------------------------------------------
    # Core update: ray-trace every beam, apply log-odds to each cell
    # ------------------------------------------------------------------
    def _world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Origin of grid is (pose.x - size/2, pose.y - size/2)."""
        ox = self._pose.x - self._size_m / 2.0
        oy = self._pose.y - self._size_m / 2.0
        gx = int((wx - ox) / self._res)
        gy = int((wy - oy) / self._res)
        return gx, gy

    def _integrate_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        angle = msg.angle_min
        rx, ry = self._world_to_grid(self._pose.x, self._pose.y)
        if not (0 <= rx < self._cells and 0 <= ry < self._cells):
            # Robot left the grid -- the grid recenters each cycle
            # via _world_to_grid, so this mainly guards numerical noise.
            return

        max_r = min(self._max_range, float(msg.range_max))
        min_r = max(self._min_range, float(msg.range_min))
        cos_y = math.cos(self._pose.yaw)
        sin_y = math.sin(self._pose.yaw)

        for i in range(n):
            r = msg.ranges[i]
            beam_angle = angle + i * msg.angle_increment

            # Reject non-finite and out-of-spec returns.
            if not math.isfinite(r):
                r = max_r
                hit = False
            elif r < min_r:
                angle = msg.angle_min  # keep angle var honest
                continue
            elif r >= max_r:
                r = max_r
                hit = False
            else:
                hit = True

            # Rotate beam into world frame
            lx = r * math.cos(beam_angle)
            ly = r * math.sin(beam_angle)
            wx = self._pose.x + cos_y * lx - sin_y * ly
            wy = self._pose.y + sin_y * lx + cos_y * ly

            ex, ey = self._world_to_grid(wx, wy)
            ex = max(0, min(self._cells - 1, ex))
            ey = max(0, min(self._cells - 1, ey))

            # Free along the beam
            for cx, cy in bresenham(rx, ry, ex, ey):
                if cx == ex and cy == ey:
                    break
                if 0 <= cx < self._cells and 0 <= cy < self._cells:
                    self._log_odds[cy, cx] += self._l_free

            # Occupied at the endpoint (only if the return was a real hit)
            if hit:
                self._log_odds[ey, ex] += self._l_occ

        # Clamp to avoid overconfidence / unrecoverable cells
        np.clip(self._log_odds, self._l_min, self._l_max, out=self._log_odds)

    # ------------------------------------------------------------------
    # Outputs
    # ------------------------------------------------------------------
    def _publish_outputs(self) -> None:
        if not self._have_scan:
            return

        self._publish_grid()
        self._publish_markers()

    def _publish_grid(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._grid_frame

        msg.info.resolution = self._res
        msg.info.width = self._cells
        msg.info.height = self._cells
        msg.info.origin.position.x = self._pose.x - self._size_m / 2.0
        msg.info.origin.position.y = self._pose.y - self._size_m / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Log-odds -> probability -> [0..100]. Unknown (prob ~ 0.5) -> -1.
        prob = 1.0 - 1.0 / (1.0 + np.exp(self._log_odds))
        cells = np.full(prob.shape, -1, dtype=np.int8)
        known = np.abs(self._log_odds) > 1e-3
        cells[known] = np.clip(
            np.round(prob[known] * 100.0), 0, 100).astype(np.int8)
        msg.data = cells.flatten().tolist()

        self._grid_pub.publish(msg)

    def _publish_markers(self) -> None:
        # Close-range occupied cells -> sphere markers to flag danger zone.
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear marker keeps RViz clean between cycles
        clear = Marker()
        clear.header.frame_id = self._grid_frame
        clear.header.stamp = now
        clear.ns = 'obstacles'
        clear.id = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # Threshold: probability > 0.65
        mask = (self._log_odds > math.log(0.65 / 0.35))
        if not mask.any():
            self._marker_pub.publish(ma)
            return

        ys, xs = np.where(mask)
        ox = self._pose.x - self._size_m / 2.0
        oy = self._pose.y - self._size_m / 2.0
        wx = ox + (xs + 0.5) * self._res
        wy = oy + (ys + 0.5) * self._res

        # Only show cells within danger radius of the robot
        d2 = (wx - self._pose.x) ** 2 + (wy - self._pose.y) ** 2
        inside = d2 <= self._danger_r ** 2
        wx, wy = wx[inside], wy[inside]
        if wx.size == 0:
            self._marker_pub.publish(ma)
            return

        m = Marker()
        m.header.frame_id = self._grid_frame
        m.header.stamp = now
        m.ns = 'obstacles'
        m.id = 1
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = self._res * 1.8
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.1, a=0.9)
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(x), y=float(y), z=0.05)
                    for x, y in zip(wx, wy)]
        ma.markers.append(m)

        self._marker_pub.publish(ma)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"Fatal: {exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
