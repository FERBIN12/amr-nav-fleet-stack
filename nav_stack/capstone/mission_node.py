#!/usr/bin/env python3
"""Capstone mission node: loop a robot between pick stations, forever, safely.

This is the thing an earlier module builds and 12.6 and 12.7 exercise. It is written
to be READ, because the point of the capstone is that every piece of it was
explained earlier in this project:

  the station poses are on the map from an earlier module
  the goals go through NavigateToPose, the action Nav2 exposes (an earlier module)
  the recovery policy is the one an earlier module tuned and an earlier module stress-tested
  and the failure handling exists because 11.4 measured what a stall looks like

WHAT IT DOES NOT DO, deliberately: it does not implement its own planner, its own
collision check or its own recovery behaviours. Nav2 has all three, tuned, and a
mission node that re-implements them is the single most common way to end up with
two controllers fighting over one robot.

DESIGN DECISIONS WORTH THE WORDS:

  ONE GOAL IN FLIGHT. The node sends a goal, waits for the result, then sends the
  next. Queuing several NavigateToPose goals looks efficient and is not: Nav2
  pre-empts, so goal N+1 cancels goal N and the robot never finishes anything.

  A GOAL THAT ABORTS IS NOT A GOAL THAT FAILED. 11.4 measured an accepted goal
  aborting instantly because planner_server was inactive. So an abort is retried
  a bounded number of times, and only then treated as a station to skip -- with
  the reason logged, because "the robot stopped" is not a diagnosis.

  PROGRESS IS MEASURED, NOT ASSUMED. A robot can hold an active goal and not be
  moving, which is exactly the deadlock signature. The node watches its own
  odometry and treats "active goal, no displacement for STALL_S" as a stall,
  reporting it separately from an abort. Those are different faults with
  different fixes and a mission node that conflates them teaches the wrong
  lesson.

  IT IS INTERRUPTIBLE. SIGINT cancels the in-flight goal before shutting down,
  because a mission node killed mid-goal leaves Nav2 driving to a station nobody
  is waiting at.
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

# Station poses on the an earlier module map, in metres. These are the same four the
# allocation experiment used, so the numbers in that module and the
# behaviour here describe one warehouse.
STATIONS = [
    ("pick_a", -3.0, 5.0, 0.0),
    ("pick_b", 8.0, 1.25, 0.0),
    ("pick_c", 1.0, -3.0, 0.0),
    ("dock", -5.5, 1.25, math.pi),
]

RETRIES = 2            # per station, then skip it and log why
STALL_S = 20.0         # active goal, no displacement, for this long = stalled
STALL_EPS = 0.05       # metres of movement that counts as progress
DWELL_S = 3.0          # simulated pick time at a station


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Mission(Node):
    def __init__(self, loops=0):
        super().__init__("capstone_mission")
        self.loops_target = loops          # 0 = forever
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        q = QoSProfile(depth=10)
        q.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, "odom", self._odom, q)
        self.xy = None
        self.last_move_t = None
        self.last_xy = None
        self.stats = {"reached": 0, "aborted": 0, "stalled": 0,
                      "skipped": [], "loops": 0}
        # Keep the in-flight handle so an interrupt can cancel the REAL goal.
        # There is no client-level "cancel everything" call: ActionClient exposes
        # only _cancel_goal_async(handle), which is private and takes a handle.
        # I wrote self.client._cancel_all_goals_async() first, from memory, and
        # it does not exist -- dir(ActionClient) has _cancel_goal,
        # _cancel_goal_async and _remove_pending_cancel_request. An invented
        # method inside a try/except would have swallowed the AttributeError and
        # left Nav2 driving to a station nobody is waiting at.
        self.live = None

    def _odom(self, msg):
        p = msg.pose.pose.position
        now = time.time()
        if self.last_xy is None:
            self.last_xy, self.last_move_t = (p.x, p.y), now
        elif math.dist((p.x, p.y), self.last_xy) > STALL_EPS:
            self.last_xy, self.last_move_t = (p.x, p.y), now
        self.xy = (p.x, p.y)

    def goal_for(self, name, x, y, yaw):
        g = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        g.pose = ps
        return g

    def drive_to(self, name, x, y, yaw):
        """Send ONE goal and wait for it. Returns 'reached' | 'aborted' | 'stalled'."""
        send = self.client.send_goal_async(self.goal_for(name, x, y, yaw))
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            # An unaccepted goal is NOT a navigation failure. 11.4 measured this
            # exact case: bt_navigator's action server had not activated yet, so
            # every goal came back rejected while the robot looked broken.
            self.get_logger().error(
                "%s: goal REJECTED (or send timed out). The action server is "
                "probably not active yet -- check the navigation lifecycle "
                "manager, not the robot." % name)
            return "aborted"

        self.live = handle
        self.last_move_t = time.time()
        result_fut = handle.get_result_async()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_fut.done():
                break
            # STALL WATCH. An active goal with no displacement is the deadlock
            # signature measured earlier, and it is a different fault from an abort.
            if (self.last_move_t is not None
                    and time.time() - self.last_move_t > STALL_S):
                self.get_logger().warn(
                    "%s: STALLED -- goal is active and the robot has not moved "
                    "%.2f m in %.0f s. Cancelling. This is not the same as an "
                    "abort: the planner still believes it is working."
                    % (name, STALL_EPS, STALL_S))
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
                self.live = None
                return "stalled"
        self.live = None
        res = result_fut.result()
        status = getattr(res, "status", None)
        # 4 == SUCCEEDED in action_msgs/GoalStatus
        return "reached" if status == 4 else "aborted"

    def run(self):
        self.get_logger().info("waiting for navigate_to_pose ...")
        if not self.client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error(
                "navigate_to_pose never appeared in 60 s. The stack is not "
                "active; a mission node cannot fix that and should not pretend "
                "to by retrying forever.")
            return 2
        self.get_logger().info("server up, starting mission")
        while rclpy.ok():
            for name, x, y, yaw in STATIONS:
                outcome = None
                for attempt in range(RETRIES + 1):
                    outcome = self.drive_to(name, x, y, yaw)
                    if outcome == "reached":
                        break
                    self.get_logger().warn(
                        "%s: %s on attempt %d of %d"
                        % (name, outcome, attempt + 1, RETRIES + 1))
                if outcome == "reached":
                    self.stats["reached"] += 1
                    self.get_logger().info("%s reached; dwelling %.0fs"
                                           % (name, DWELL_S))
                    t0 = time.time()
                    while rclpy.ok() and time.time() - t0 < DWELL_S:
                        rclpy.spin_once(self, timeout_sec=0.1)
                else:
                    self.stats[outcome] += 1
                    self.stats["skipped"].append(name)
                    self.get_logger().error(
                        "%s: giving up after %d attempts (%s). SKIPPING and "
                        "continuing the loop -- a mission that halts on one bad "
                        "station strands the whole fleet."
                        % (name, RETRIES + 1, outcome))
            self.stats["loops"] += 1
            self.get_logger().info("loop %d complete: %s"
                                   % (self.stats["loops"], self.stats))
            if self.loops_target and self.stats["loops"] >= self.loops_target:
                return 0
        return 0


def main():
    loops = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    rclpy.init()
    node = Mission(loops=loops)
    rc = 0
    try:
        rc = node.run()
    except KeyboardInterrupt:
        # Cancel in flight rather than dropping out: a killed mission node leaves
        # Nav2 driving to a station nobody is waiting at.
        node.get_logger().info("interrupted; cancelling any in-flight goal")
        if node.live is not None:
            try:
                fut = node.live.cancel_goal_async()
                rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                node.get_logger().info("in-flight goal cancelled")
            except Exception as e:
                node.get_logger().error("cancel failed: %r" % (e,))
        else:
            node.get_logger().info("no goal in flight")
    finally:
        node.get_logger().info("final: %s" % node.stats)
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
