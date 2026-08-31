#!/usr/bin/env python3
"""Republish /odom with a believable covariance as /odom_cov.

WHY THIS EXISTS. `gz-sim-diff-drive-system` publishes /odom with an ALL-ZERO
covariance matrix. A zero covariance asserts INFINITE certainty, and
robot_localization will not fuse a measurement that claims to be perfect:
measured 2026-08-19, `ros2 topic echo /diagnostics` reported "Events since
startup: 0" through an entire 3-lap run while /odometry/filtered published dead
reckoning at 31 Hz and looked completely healthy on `ros2 topic hz`. The fused
result came out four times WORSE than odometry alone, which would have taught the
opposite of an earlier module's thesis.

The numbers are not invented. The EKF is fed only three channels of this message
(vx, vy, vyaw -- see odom0_config in config/ekf.yaml), so only those three
diagonal entries matter, and they are set from what the wheels can actually
resolve:

  encoder resolution on a 90 mm wheel at the plugin's 50 Hz publish rate gives
  about 0.05 m/s of velocity noise, so var = 0.05^2 = 2.5e-3
  yaw rate over the 0.445 m track from two such wheels: 0.05*sqrt(2)/0.445
  = 0.16 rad/s worst case, taken at 0.02 rad/s typical, so var = 4.0e-4

Everything the filter is told to ignore keeps a large variance so that if the
config is ever changed to believe those channels, they arrive honestly weak
rather than silently authoritative.

Usage:  odom_cov_fill.py [--in /odom] [--out /odom_cov]
"""
import argparse
import sys

# Diagonal for the 6x6 twist covariance, row-major indices 0,7,14,21,28,35.
VAR_VX = 2.5e-3        # (0.05 m/s)^2
VAR_VY = 2.5e-3
# MEASURED, not guessed. Differencing run 1's wheel yaw against ground truth over
# 4700 samples gives a yaw-rate error of stddev 0.0301 rad/s, i.e. variance
# 9.07e-04. The first version of this file guessed 4.0e-04, which told the filter
# the wheels were 2.3x better at rotation than they are.
VAR_VZ = 1.0e3         # not fed to the filter; kept deliberately weak
VAR_VROLL = 1.0e3
VAR_VPITCH = 1.0e3
VAR_VYAW = 9.07e-4     # measured: stddev 0.0301 rad/s against truth
# The pose half of the message is NOT fed to the filter either (feeding it would
# count the same evidence twice, which is an earlier module's point), so it stays weak.
VAR_POSE = 1.0e3


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="src", default="/odom")
    ap.add_argument("--out", dest="dst", default="/odom_cov")
    args = ap.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    class Fill(Node):
        def __init__(self):
            super().__init__("odom_cov_fill")
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.pub = self.create_publisher(Odometry, args.dst, qos)
            self.sub = self.create_subscription(Odometry, args.src, self.cb, qos)
            self.n = 0

        def cb(self, msg):
            pose = list(msg.pose.covariance)
            twist = list(msg.twist.covariance)
            for i in (0, 7, 14, 21, 28, 35):
                pose[i] = VAR_POSE
            twist[0] = VAR_VX
            twist[7] = VAR_VY
            twist[14] = VAR_VZ
            twist[21] = VAR_VROLL
            twist[28] = VAR_VPITCH
            twist[35] = VAR_VYAW
            msg.pose.covariance = pose
            msg.twist.covariance = twist
            self.pub.publish(msg)
            self.n += 1
            if self.n % 500 == 0:
                self.get_logger().info("republished %d" % self.n)

    rclpy.init(args=None)
    node = Fill()
    node.get_logger().info("filling covariance %s -> %s" % (args.src, args.dst))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
