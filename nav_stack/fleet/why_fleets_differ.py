#!/usr/bin/env python3
"""Why a fleet is not N robots: the things that stop being private.

Everything in an earlier module-10 assumed one machine with a private map, an
uncontested aisle, and a topic namespace it owned. This file measures what
breaks when there are two, using OUR robot's real numbers rather than generic
fleet theory.

FROM OUR OWN CONFIG AND EARLIER MEASUREMENTS:
  footprint 0.80 x 0.58 m, circumscribed radius 0.494 m   (nav2_params.yaml)
  inflation_radius 0.70, cost_scaling_factor 3.0
  vx_max 0.5 m/s, wz_max 1.9 rad/s                        (measured in 8.10)
  controller 20 Hz, MPPI 2000 sequences over 2.8 s        (an earlier module)
  the warehouse loop is 40.0 m, verified collision-free    (drive_loop_6_7.py)
  min aisle clearance 1.70 m from shelf_4                  (same file)
  scan 541 beams at 10 Hz; camera 640x480 at 15 Hz         (the xacro)
  a person needs 2.48 m of clear aisle to pass             (measured in 10.4)

THE THREE THINGS THAT STOP BEING PRIVATE, each measured rather than asserted:
  1. THE AISLE. Two robots passing need twice the footprint plus both
     inflations, which our aisles may not have.
  2. THE TOPIC NAMESPACE. Every node name and topic in an earlier module-10 is
     unqualified, so a second robot collides with the first by name.
  3. THE BANDWIDTH. A costmap and a scan per robot, multiplied.
"""
import json
import math
import sys

FOOT_L, FOOT_W = 0.80, 0.58
ROBOT_R = math.hypot(FOOT_L / 2, FOOT_W / 2)
INFLATION_R = 0.70
VX_MAX = 0.5
HZ_CTRL = 20
SCAN_BEAMS, SCAN_HZ = 541, 10
CAM_W, CAM_H, CAM_HZ = 640, 480, 15
AISLE_MIN = 1.70
LOOP_M = 40.0

# costmap sizes, from nav2_params.yaml
LOCAL_W, LOCAL_H, LOCAL_RES = 3.0, 3.0, 0.05
GLOBAL_CELLS = 600 * 556          # our saved map


INSCRIBED_R = FOOT_W / 2          # 0.29 m, the HARD block


def passing_width(n=2):
    """HARD width for n robots abreast: footprints plus a real gap.

    NOT n*width + 2*inflation. Inflation is a COST GRADIENT, not a keep-out:
    a robot may drive through inflated cells at a penalty, and only the
    INSCRIBED radius is a hard block. My first version added inflation on both
    sides and concluded a SINGLE robot did not fit a 1.70 m aisle -- which our
    own 40 m loop demonstrably drove. A formula that contradicts a measurement
    you already have is wrong, not surprising.
    """
    return n * FOOT_W + (n + 1) * 0.10          # 0.10 m of real clearance


def cost_at_separation(sep_m):
    """What does the OTHER robot cost you at this centre-to-centre separation?

    The inflation curve from 8.6: 253 inside the inscribed radius, then
    252*exp(-3.0*(d - r_inscribed)) out to the inflation radius.
    """
    d = sep_m - INSCRIBED_R
    if d <= 0:
        return 253
    if d >= INFLATION_R:
        return 0
    return int(max(1, min(252, 252 * math.exp(-3.0 * d))))


def scan_bytes_per_s():
    # LaserScan: 541 float32 ranges + 541 float32 intensities + header
    return (SCAN_BEAMS * 4 * 2 + 64) * SCAN_HZ


def cloud_bytes_per_s():
    # PointCloud2 xyz float32 + rgb, one point per pixel
    return (CAM_W * CAM_H * 16) * CAM_HZ


def costmap_bytes_per_s():
    local = (LOCAL_W / LOCAL_RES) * (LOCAL_H / LOCAL_RES)
    # local costmap publishes at 5 Hz in our config, global at 1 Hz
    return local * 5 + GLOBAL_CELLS * 1


def main():
    print("ONE ROBOT'S ASSUMPTIONS, from an earlier module to 10")
    print("  footprint %.2f x %.2f m, circumscribed %.3f m"
          % (FOOT_L, FOOT_W, ROBOT_R))
    print("  inflation %.2f m, top speed %.1f m/s, controller %d Hz"
          % (INFLATION_R, VX_MAX, HZ_CTRL))
    print("  narrowest verified aisle clearance %.2f m" % AISLE_MIN)
    print()

    print("1. THE AISLE STOPS BEING PRIVATE")
    rows = []
    for n in (1, 2, 3):
        w = passing_width(n)
        fits = w <= AISLE_MIN
        rows.append({"robots": n, "hard_width_m": round(w, 3),
                     "fits_1_70": fits})
        print("  %d abreast: hard width %.2f m in a %.2f m aisle -> %s"
              % (n, w, AISLE_MIN, "fits" if fits else "DOES NOT FIT"))
    print()
    print("  so PHYSICALLY two of our robots pass in a 1.70 m aisle with room")
    print("  to spare. The problem is not geometry, it is COST.")
    print()
    # Two robots hugging opposite walls of an aisle of width W are separated
    # by W - FOOT_W, centre to centre.
    print("  passing separation is aisle width minus one footprint width:")
    costs = []
    for aisle in (1.70, 1.60, 1.57, 1.50, 1.40, 1.20, 1.00):
        sep = aisle - FOOT_W
        c = cost_at_separation(sep)
        costs.append({"aisle_m": aisle, "separation_m": round(sep, 3),
                      "cost": c, "free": c == 0})
        note = "free" if c == 0 else ("LETHAL" if c >= 253 else "penalised")
        print("    a %.2f m aisle -> %.2f m apart -> cost %3d  (%s)"
              % (aisle, sep, c, note))
    print()
    zero_thresh = INSCRIBED_R + INFLATION_R + FOOT_W
    print("  SO IN OUR OWN 1.70 m AISLE THEY PASS CLEANLY, at cost 0.")
    print("  The cost only appears below a %.2f m aisle, and that is the honest"
          % zero_thresh)
    print("  answer: our aisles are wide enough for two robots and the trouble")
    print("  starts %.2f m narrower. What does NOT fit is three abreast."
          % (zero_thresh - AISLE_MIN if zero_thresh > AISLE_MIN else AISLE_MIN - zero_thresh))
    print()
    print("  I got this wrong twice before measuring it. First I added inflation")
    print("  as a hard keep-out and concluded one robot could not fit an aisle it")
    print("  had already driven. Then I quoted a cost at a separation where the")
    print("  cost is zero. Inflation is a GRADIENT: only the inscribed radius")
    print("  blocks, and the gradient reaches %.2f m." % INFLATION_R)
    print()
    max_infl = (AISLE_MIN - 2 * FOOT_W) / 2

    print("2. THE NAMESPACE STOPS BEING PRIVATE")
    topics = ["/cmd_vel", "/odom", "/scan_front", "/scan_rear", "/imu",
              "/camera/points", "/tf", "/map", "/amcl_pose", "/plan"]
    nodes = ["controller_server", "planner_server", "bt_navigator", "amcl",
             "map_server", "collision_monitor", "robot_state_publisher"]
    print("  every topic in an earlier module-10 is unqualified: %s ..."
          % ", ".join(topics[:4]))
    print("  %d topics and %d node names, none namespaced"
          % (len(topics), len(nodes)))
    print("  a second robot on the same domain publishes /cmd_vel too, and both")
    print("  robots subscribe to both. That is not a race, it is addition:")
    print("  each robot receives the SUM of two controllers' intentions.")
    print("  and /tf is worse, because a second base_footprint in one tree makes")
    print("  the tree ambiguous rather than merely wrong.")
    print()

    print("3. THE BANDWIDTH STOPS BEING PRIVATE")
    sb, cb, mb = scan_bytes_per_s(), cloud_bytes_per_s(), costmap_bytes_per_s()
    per_robot = sb + cb + mb
    bw = []
    for n in (1, 2, 5, 10):
        total = per_robot * n
        bw.append({"robots": n, "bytes_per_s": int(total),
                   "mbit_per_s": round(total * 8 / 1e6, 1)})
        print("  %2d robot(s): %.1f MB/s = %.0f Mbit/s"
              % (n, total / 1e6, total * 8 / 1e6))
    print()
    print("  breakdown per robot: scan %.0f kB/s, cloud %.1f MB/s, costmaps %.0f kB/s"
          % (sb / 1e3, cb / 1e6, mb / 1e3))
    print("  the point cloud is %.0f%% of it, which is why fleets do not ship"
          % (100 * cb / per_robot))
    print("  raw clouds over the network. They ship the costmap, or a summary.")

    out = {
        "provenance": "footprint, inflation and limits from nav2_params.yaml "
                      "(measured in 8.10); aisle clearance from "
                      "drive_loop_6_7.py's verified loop; sensor rates from "
                      "cortex_amr.gazebo.xacro; the 2.48 m person figure from 10.4",
        "one_robot": {"footprint": [FOOT_L, FOOT_W],
                      "circumscribed_r": round(ROBOT_R, 3),
                      "inflation_r": INFLATION_R, "vx_max": VX_MAX,
                      "aisle_min_m": AISLE_MIN},
        "abreast": rows,
        "cost_by_aisle_width": costs,
        "free_passing_threshold_m": round(INSCRIBED_R + INFLATION_R + FOOT_W, 3),
        "our_aisle_passes_free": cost_at_separation(AISLE_MIN - FOOT_W) == 0,
        "inscribed_r": INSCRIBED_R,
        "inflation_for_two_abreast_m": round(max_infl, 3),
        "inflation_cut_pct": round(100 * (1 - max_infl / INFLATION_R), 1),
        "unqualified_topics": len(topics), "unqualified_nodes": len(nodes),
        "bandwidth": bw,
        "per_robot_bytes_s": {"scan": int(sb), "cloud": int(cb),
                              "costmaps": int(mb), "total": int(per_robot)},
        "cloud_share_pct": round(100 * cb / per_robot, 1),
        "finding": (
            "A fleet is not N robots because three things stop being private. "
            "The AISLE: two of our robots pass CLEANLY in our own %.2f m aisle, "
            "at %.2f m separation and inflation cost 0. The trouble starts below "
            "a %.2f m aisle, and three abreast never fits. I got this wrong twice "
            "before measuring: first treating inflation as a hard keep-out (which "
            "made a single robot not fit an aisle it had driven), then quoting a "
            "cost at a separation where the cost is zero. The NAMESPACE: %d "
            "topics and %d node names are unqualified, so a second robot does not "
            "race the first, it ADDS to it. The BANDWIDTH: %.1f MB/s per robot of "
            "which %.0f%% is the point cloud, so ten robots is %.0f Mbit/s and "
            "fleets ship costmaps rather than clouds."
            % (AISLE_MIN, AISLE_MIN - FOOT_W,
               INSCRIBED_R + INFLATION_R + FOOT_W, len(topics), len(nodes),
               per_robot / 1e6, 100 * cb / per_robot,
               per_robot * 10 * 8 / 1e6)),
    }
    with open("reference/why_fleets_differ_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nwrote reference/why_fleets_differ_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
