#!/usr/bin/env python3
"""11.3: two namespaced AMRs in one warehouse, and what actually happens.

11.1 predicted, from the inflation curve, that two of our robots pass cleanly in
a 1.70 m aisle at cost 0. 11.2 got two namespaced stacks running. This runs both
at once and measures whether the prediction holds, because 10.6 taught me that
writing the prediction down first is worth more than explaining it afterwards.

PREDICTION, recorded before the run:
  they should pass without either stopping, because at 1.12 m separation the
  inflation cost each reads from the other is 0
  the risk is not collision, it is that the GLOBAL planner routes them both down
  the same aisle at the same time and neither yields

Measures for each robot: /cmd_vel history, ground-truth displacement, minimum
separation reached, and whether either dropped to zero velocity.
"""
import json
import math
import subprocess
import sys
import threading
import time

CMD = {"r1": [], "r2": []}
POSE = {"r1": [], "r2": []}
stop = False


def tap_cmd(ns):
    p = subprocess.Popen(["ros2", "topic", "echo", "/%s/cmd_vel" % ns,
                          "geometry_msgs/msg/Twist"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True)
    vx = None
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
        elif sect == "a" and t.startswith("z:") and vx is not None:
            CMD[ns].append((time.time(), vx, float(t.split(":")[1])))
            vx = None
    p.kill()


def tap_pose(name, ns):
    """gz model pose. Parses pose>position, skipping the orientation block."""
    p = subprocess.Popen(["gz", "topic", "-e", "-t",
                          "/world/warehouse/pose/info"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True)
    cur = None
    in_pos = False
    x = None
    for line in p.stdout:
        if stop:
            break
        t = line.strip()
        if t.startswith("name:"):
            cur = t.split('"')[1] if '"' in t else None
            in_pos = False
            x = None
        elif t.startswith("position"):
            in_pos = (cur == name)
            x = None
        elif t.startswith("orientation"):
            in_pos = False
        elif in_pos and t.startswith("x:"):
            x = float(t.split(":")[1])
        elif in_pos and t.startswith("y:") and x is not None:
            POSE[ns].append((time.time(), x, float(t.split(":")[1])))
            x = None
            in_pos = False
    p.kill()


def goal(ns, x, y):
    g = ('{pose: {header: {frame_id: "map"}, pose: {position: '
         '{x: %.2f, y: %.2f, z: 0.0}, orientation: {w: 1.0}}}}' % (x, y))
    return subprocess.Popen(
        ["ros2", "action", "send_goal", "/%s/navigate_to_pose" % ns,
         "nav2_msgs/action/NavigateToPose", g],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    global stop
    threading.Thread(target=tap_cmd, args=("r1",), daemon=True).start()
    threading.Thread(target=tap_cmd, args=("r2",), daemon=True).start()
    threading.Thread(target=tap_pose, args=("cortex_r1", "r1"), daemon=True).start()
    threading.Thread(target=tap_pose, args=("cortex_r2", "r2"), daemon=True).start()
    time.sleep(4.0)

    # head-on down the same aisle: r1 goes east, r2 goes west
    print("sending opposing goals down the same aisle")
    g1 = goal("r1", 2.0, 1.25)
    g2 = goal("r2", -5.0, 1.25)

    t0 = time.time()
    minsep = 99.0
    while time.time() - t0 < 60.0:
        time.sleep(0.5)
        if POSE["r1"] and POSE["r2"]:
            a, b = POSE["r1"][-1], POSE["r2"][-1]
            s = math.hypot(a[1] - b[1], a[2] - b[2])
            minsep = min(minsep, s)
    stop = True
    time.sleep(0.5)

    res = {"prediction": "both pass, neither stops, cost 0 at 1.12 m",
           "min_separation_m": round(minsep, 3)}
    for ns in ("r1", "r2"):
        c = CMD[ns]
        pz = POSE[ns]
        disp = 0.0
        if len(pz) > 1:
            disp = math.hypot(pz[-1][1] - pz[0][1], pz[-1][2] - pz[0][2])
        vx = [x[1] for x in c]
        res[ns] = {
            "cmd_samples": len(c),
            "vx_mean": round(sum(vx) / len(vx), 4) if vx else 0.0,
            "vx_min": round(min(vx), 4) if vx else 0.0,
            "zero_frac": round(sum(1 for v in vx if abs(v) < 0.02)
                               / max(1, len(vx)), 3),
            "displacement_m": round(disp, 3),
            "pose_samples": len(pz),
        }
        print("  %s: %d cmds, vx mean %.3f min %.3f, zero %.0f%%, moved %.2f m"
              % (ns, len(c), res[ns]["vx_mean"], res[ns]["vx_min"],
                 100 * res[ns]["zero_frac"], disp))
    print("  minimum separation reached: %.3f m" % minsep)
    both_moved = all(res[n]["displacement_m"] > 0.3 for n in ("r1", "r2"))
    either_stopped = any(res[n]["zero_frac"] > 0.5 for n in ("r1", "r2"))
    res["outcome"] = ("BOTH PASSED" if both_moved and not either_stopped
                      else ("ONE STOPPED" if either_stopped else "NEITHER MOVED"))
    print("  OUTCOME: %s  (predicted BOTH PASSED)" % res["outcome"])
    with open("reference/two_robots_run_results.json", "w") as f:
        json.dump(res, f, indent=2)
    print("wrote reference/two_robots_run_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
