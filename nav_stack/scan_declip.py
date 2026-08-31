#!/usr/bin/env python3
"""Republish /scan_front with the robot's own chassis masked out.

WHY THIS EXISTS, measured 2026-08-20:
  The front scanner is mounted at the FRONT-LEFT CORNER of the chassis, yawed
  45 deg (urdf: yaw="0.785"), and sweeps 270 deg (angle_min -2.356 ..
  angle_max 2.356). The far end of that sweep necessarily crosses the robot's
  own body, so 26 of 541 beams returned 0.09-0.25 m in ONE CONTIGUOUS ARC from
  122.5 deg to 135.0 deg. A real obstacle is scattered or centred; a single arc
  at the sweep limit is a self-hit.

  nav2's collision_monitor believed those returns and zeroed every command:
      /cmd_vel_nav      -0.132   (MPPI producing a command)
      /cmd_vel_smoothed -0.135   (smoother passing it)
      /cmd_vel          -0.0     (monitor stopping the robot)
  with "Robot to approach for 1.200000 seconds away from collision".

  This is the standard real-world answer: filter the scan before anything
  consumes it, rather than changing the robot or telling the safety layer to
  ignore things. laser_filters is not installed on this machine, so this node
  does the one job we need.

MASK: beams whose angle is above MASK_FROM_DEG are set to +inf (no return),
which is how a laser reports "nothing there" and is what every consumer expects.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

MASK_FROM_DEG = 118.0     # measured self-hit arc starts at 122.5; 4.5 deg margin

# TOPICS ARE RELATIVE, not absolute. This node was written for ONE robot with
# '/scan_front' hardcoded, which is the same mistake the gazebo xacro made and
# 11.3 had to undo: a hardcoded absolute name cannot serve two robots, and a
# namespace has no power over it. Relative names let ONE node definition run
# twice, once per robot:
#     ros2 run ... scan_declip.py --ros-args -r __ns:=/r1
IN_TOPIC = 'scan_front'
OUT_TOPIC = 'scan_front_filtered'


class Declip(Node):
    def __init__(self):
        super().__init__('scan_declip')
        q = QoSProfile(depth=10)
        q.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(LaserScan, OUT_TOPIC, q)
        self.create_subscription(LaserScan, IN_TOPIC, self.cb, q)
        self.get_logger().info('declip: %s -> %s (ns=%s)' % (
            self.resolve_topic_name(IN_TOPIC),
            self.resolve_topic_name(OUT_TOPIC),
            self.get_namespace()))
        self.masked = 0
        self.msgs = 0

    def cb(self, m):
        lim = math.radians(MASK_FROM_DEG)
        r = list(m.ranges)
        n = 0
        for i in range(len(r)):
            if m.angle_min + i * m.angle_increment >= lim:
                r[i] = float('inf')
                n += 1
        m.ranges = r
        self.pub.publish(m)
        self.msgs += 1
        self.masked = n
        if self.msgs % 50 == 1:
            self.get_logger().info(
                'masked %d of %d beams at >= %.1f deg' % (n, len(r), MASK_FROM_DEG))


def main():
    rclpy.init()
    rclpy.spin(Declip())


if __name__ == '__main__':
    main()
