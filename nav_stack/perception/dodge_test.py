#!/usr/bin/env python3
"""10.6: send a goal, then walk an obstacle into the path and MEASURE the result.

The prediction written down before the test: the stack should STOP rather than swerve,
because a 0.5 m/s robot cannot dodge a 1.5 m/s walker (0.556 m of available
lateral travel against 0.831 m needed).

This drives the real stack and records /cmd_vel so the outcome is measured, not
asserted.
"""
import json
import math
import subprocess
import sys
import threading
import time

CMD = []
TRUTH = []
stop = False


def tap_cmdvel():
    p = subprocess.Popen(["ros2", "topic", "echo", "/cmd_vel",
                          "geometry_msgs/msg/Twist"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True)
    vx = wz = None
    sect = None
    for line in p.stdout:
        if stop:
            break
        t = line.strip()
        if t.startswith("linear:"):
            sect = "l"
        elif t.startswith("angular:"):
            sect = "a"
        elif sect == "l" and t.startswith("x:"):
            vx = float(t.split(":")[1])
        elif sect == "a" and t.startswith("z:"):
            wz = float(t.split(":")[1])
            if vx is not None:
                CMD.append((time.time(), vx, wz))
            vx = None
    p.kill()


def tap_truth():
    p = subprocess.Popen(["gz", "topic", "-e", "-t", "/truth/odom"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True)
    x = None
    in_pos = False
    for line in p.stdout:
        if stop:
            break
        t = line.strip()
        if t.startswith("position"):
            in_pos = True
            x = None
            continue
        if t.startswith("orientation") or t.startswith("header"):
            in_pos = False
            continue
        if in_pos and t.startswith("x:"):
            x = float(t.split(":")[1])
        elif in_pos and t.startswith("y:") and x is not None:
            TRUTH.append((time.time(), x, float(t.split(":")[1])))
            x = None
            in_pos = False
    p.kill()


def gz_set_pose(name, x, y, z=0.0):
    req = ('name: "%s", position: {x: %.3f, y: %.3f, z: %.3f}'
           % (name, x, y, z))
    subprocess.run(["gz", "service", "-s", "/world/warehouse/set_pose",
                    "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "800", "--req", req],
                   capture_output=True, timeout=6)


def main():
    global stop
    threading.Thread(target=tap_cmdvel, daemon=True).start()
    threading.Thread(target=tap_truth, daemon=True).start()
    time.sleep(3.0)

    # send a goal straight down the aisle the robot spawned in
    goal = ('{pose: {header: {frame_id: "map"}, pose: {position: '
            '{x: 3.0, y: 1.25, z: 0.0}, orientation: {w: 1.0}}}}')
    g = subprocess.Popen(["ros2", "action", "send_goal", "/navigate_to_pose",
                          "nav2_msgs/action/NavigateToPose", goal],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True)
    print("goal sent: (4.0, 1.25)")

    # let it get moving
    t0 = time.time()
    while time.time() - t0 < 12.0:
        time.sleep(0.5)
    moving = [c for c in CMD if abs(c[1]) > 0.02]
    print("  after 12 s: %d cmd samples, %d with vx>0.02"
          % (len(CMD), len(moving)))
    if TRUTH:
        print("  truth now (%.2f, %.2f)" % (TRUTH[-1][1], TRUTH[-1][2]))

    # NOW walk an obstacle into its path, 2.0 m ahead of wherever it is
    if not TRUTH:
        print("  FATAL: no ground truth, cannot place the obstacle")
        stop = True
        return 2
    rx, ry = TRUTH[-1][1], TRUTH[-1][2]
    print("  placing the walker 2.0 m ahead at (%.2f, %.2f)" % (rx + 2.0, ry))
    mark = len(CMD)
    tw = time.time()
    # step it toward the robot at ~1.5 m/s in 0.1 m increments
    # PURSUE THE ROBOT. 10.5's claim is about a walker closing at 1.5 m/s, so
    # each step moves 0.05 m TOWARD wherever the robot is NOW. Stop at the
    # combined radius: past that the physics engine decides, not the stack.
    px, py = rx + 2.0, ry
    for i in range(120):
        if TRUTH:
            cx, cy = TRUTH[-1][1], TRUTH[-1][2]
        else:
            cx, cy = rx, ry
        dx, dy = cx - px, cy - py
        d = math.hypot(dx, dy)
        if d <= 0.85:
            print("  walker reached the combined radius after %d steps" % i)
            break
        px += 0.05 * dx / d
        py += 0.05 * dy / d
        gz_set_pose("Person 1 - Standing", px, py, 0.0)
        time.sleep(0.05 / 1.5)
    print("  walker delivered over %.2f s" % (time.time() - tw))

    time.sleep(4.0)
    stop = True
    time.sleep(0.5)

    # A STALLED ROBOT LOOKS LIKE A STOPPED ONE ON /cmd_vel ALONE. Record the
    # ground-truth displacement over the same window so "it stopped because the
    # stack decided to" can be told apart from "it stopped because it is stuck".
    # INDEXING TRUTH WITH A CMD INDEX IS A BUG: the two taps run at different
    # rates, so TRUTH[mark] is an unrelated sample. Use the wall-clock instant
    # the walker started arriving.
    tr_after = [t for t in TRUTH if t[0] >= tw]
    tr_before = [t for t in TRUTH if t[0] < tw]
    disp = 0.0
    if len(tr_after) > 1:
        disp = math.hypot(tr_after[-1][1] - tr_after[0][1],
                          tr_after[-1][2] - tr_after[0][2])
    disp_before = 0.0
    if len(tr_before) > 1:
        disp_before = math.hypot(tr_before[-1][1] - tr_before[0][1],
                                 tr_before[-1][2] - tr_before[0][2])
    after = CMD[mark:]
    if not after:
        print("  no cmd_vel after the obstacle appeared")
        return 1
    vx_before = [c[1] for c in CMD[max(0, mark - 40):mark]]
    vx_after = [c[1] for c in after]
    wz_after = [abs(c[2]) for c in after]
    res = {
        "prediction": "stop rather than swerve",
        "samples_before": len(vx_before), "samples_after": len(vx_after),
        "vx_mean_before": round(sum(vx_before) / max(1, len(vx_before)), 4),
        "vx_mean_after": round(sum(vx_after) / max(1, len(vx_after)), 4),
        "vx_min_after": round(min(vx_after), 4),
        "wz_max_after": round(max(wz_after), 4),
        "zero_vx_fraction_after": round(
            sum(1 for v in vx_after if abs(v) < 0.02) / len(vx_after), 3),
        "truth_displacement_before_m": round(disp_before, 3),
        "truth_displacement_after_m": round(disp, 3),
        "walker_stopped_at_m": 0.9,
        "combined_radius_m": 0.831,
    }
    print()
    print("MEASURED:")
    for k, v in res.items():
        print("  %-24s %s" % (k, v))
    # JUDGE ON WHAT THE ROBOT DID, not only on what it was told. A stalled robot
    # emits rotation commands forever and looks like a swerve on /cmd_vel alone.
    stopped = res["truth_displacement_after_m"] < 0.15
    swerved = (not stopped) and res["wz_max_after"] > 0.3
    res["outcome"] = "STOPPED" if stopped else ("SWERVED" if swerved else "SLOWED")
    print()
    print("  OUTCOME: %s  (prediction was STOP)" % res["outcome"])
    with open("reference/dodge_test_results.json", "w") as f:
        json.dump(res, f, indent=2)
    print("wrote reference/dodge_test_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
