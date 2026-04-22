#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_sensor_publisher.py
------------------------
Synthetic sensor feed for evaluating obstacle_grid_node without hardware
or a rosbag. Simulates a differential-drive robot moving in a rectangular
room with a handful of circular obstacles.

Publishes:
    /scan (sensor_msgs/LaserScan)   -- 360-deg 2D LiDAR, 1 deg resolution
    /odom (nav_msgs/Odometry)       -- ground-truth pose as odom

Noise model:
    * Range returns have additive Gaussian noise (sigma configurable).
    * A small fraction of beams are randomly dropped to NaN to exercise
      the range validation paths in the consumer.

This is intentionally simple. It is *not* a physics simulator.
"""

import math
import random
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


@dataclass
class Circle:
    x: float
    y: float
    r: float


@dataclass
class Wall:
    """Axis-aligned wall segment from (x1, y1) to (x2, y2)."""
    x1: float
    y1: float
    x2: float
    y2: float


def ray_segment_intersect(
    ox: float, oy: float, dx: float, dy: float,
    x1: float, y1: float, x2: float, y2: float,
) -> float:
    """Return t >= 0 where ray (ox,oy)+t*(dx,dy) hits segment, else inf."""
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return math.inf
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return math.inf


def ray_circle_intersect(
    ox: float, oy: float, dx: float, dy: float, c: Circle,
) -> float:
    """Return nearest t >= 0 where ray hits the circle, else inf."""
    fx, fy = ox - c.x, oy - c.y
    a = dx * dx + dy * dy
    b = 2.0 * (fx * dx + fy * dy)
    cc = fx * fx + fy * fy - c.r * c.r
    disc = b * b - 4.0 * a * cc
    if disc < 0.0:
        return math.inf
    disc = math.sqrt(disc)
    t1 = (-b - disc) / (2.0 * a)
    t2 = (-b + disc) / (2.0 * a)
    for t in (t1, t2):
        if t >= 0.0:
            return t
    return math.inf


class FakeSensorPublisher(Node):

    def __init__(self) -> None:
        super().__init__('fake_sensor_publisher')

        self.declare_parameter('scan.rate_hz', 10.0)
        self.declare_parameter('odom.rate_hz', 50.0)
        self.declare_parameter('scan.range_max_m', 10.0)
        self.declare_parameter('scan.noise_sigma_m', 0.02)
        self.declare_parameter('scan.dropout_prob', 0.01)
        self.declare_parameter('scan.num_beams', 360)
        self.declare_parameter('robot.linear_speed_mps', 0.25)
        self.declare_parameter('robot.angular_speed_radps', 0.20)
        self.declare_parameter('frames.odom', 'odom')
        self.declare_parameter('frames.base', 'base_link')

        self._scan_rate = float(self.get_parameter('scan.rate_hz').value)
        self._odom_rate = float(self.get_parameter('odom.rate_hz').value)
        self._range_max = float(self.get_parameter('scan.range_max_m').value)
        self._noise_sigma = float(self.get_parameter('scan.noise_sigma_m').value)
        self._dropout_p = float(self.get_parameter('scan.dropout_prob').value)
        self._num_beams = int(self.get_parameter('scan.num_beams').value)
        self._v = float(self.get_parameter('robot.linear_speed_mps').value)
        self._w = float(self.get_parameter('robot.angular_speed_radps').value)
        self._odom_frame = self.get_parameter('frames.odom').value
        self._base_frame = self.get_parameter('frames.base').value

        # World: 6x6m room with four obstacles
        self._walls: List[Wall] = [
            Wall(-3, -3,  3, -3),
            Wall( 3, -3,  3,  3),
            Wall( 3,  3, -3,  3),
            Wall(-3,  3, -3, -3),
        ]
        self._obstacles: List[Circle] = [
            Circle( 1.2,  0.8, 0.30),
            Circle(-1.5,  1.7, 0.25),
            Circle( 0.0, -1.8, 0.35),
            Circle(-0.8, -0.5, 0.20),
        ]

        # Pose state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_odom_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self._odom_pub = self.create_publisher(Odometry, '/odom', sensor_qos)

        self.create_timer(1.0 / self._scan_rate, self._tick_scan)
        self.create_timer(1.0 / self._odom_rate, self._tick_odom)

        self.get_logger().info(
            f"fake_sensor_publisher up: "
            f"scan {self._scan_rate} Hz, odom {self._odom_rate} Hz"
        )

    # ------------------------------------------------------------------
    def _tick_odom(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now

        # Lazy figure-8ish trajectory -- keeps the robot inside the room.
        self._x += self._v * math.cos(self._yaw) * dt
        self._y += self._v * math.sin(self._yaw) * dt
        self._yaw += self._w * math.sin(0.5 * now.nanoseconds * 1e-9) * dt

        # Keep yaw in [-pi, pi]
        if self._yaw > math.pi:
            self._yaw -= 2 * math.pi
        elif self._yaw < -math.pi:
            self._yaw += 2 * math.pi

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        # Quaternion from yaw
        msg.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        msg.twist.twist.linear.x = self._v
        msg.twist.twist.angular.z = self._w
        self._odom_pub.publish(msg)

    # ------------------------------------------------------------------
    def _tick_scan(self) -> None:
        now = self.get_clock().now()

        msg = LaserScan()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._base_frame
        msg.angle_min = -math.pi
        msg.angle_max =  math.pi
        msg.angle_increment = 2 * math.pi / float(self._num_beams)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self._scan_rate
        msg.range_min = 0.10
        msg.range_max = self._range_max

        ranges: List[float] = []
        for i in range(self._num_beams):
            a = msg.angle_min + i * msg.angle_increment + self._yaw
            dx, dy = math.cos(a), math.sin(a)

            best = self._range_max
            for w in self._walls:
                t = ray_segment_intersect(
                    self._x, self._y, dx, dy,
                    w.x1, w.y1, w.x2, w.y2)
                if t < best:
                    best = t
            for c in self._obstacles:
                t = ray_circle_intersect(self._x, self._y, dx, dy, c)
                if t < best:
                    best = t

            if random.random() < self._dropout_p:
                ranges.append(float('nan'))
                continue

            # Add sensor noise
            best += random.gauss(0.0, self._noise_sigma)
            if best < msg.range_min:
                best = msg.range_min
            if best > self._range_max:
                best = float('inf')
            ranges.append(float(best))

        msg.ranges = ranges
        self._scan_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
