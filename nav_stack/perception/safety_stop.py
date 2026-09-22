#!/usr/bin/env python3
"""The safety stop: the one component allowed to distrust everything else.

Nav2's collision_monitor sits LAST in the velocity chain. Everything upstream
(planner, controller, smoother) proposes; the monitor disposes. 10.6 proved that
empirically the hard way: with its source stale it zeroed every command while
the rest of the stack cheerfully reported it was following a path.

CONFIG IS OURS, from nav2_params.yaml:
  cmd_vel_in_topic  cmd_vel_smoothed
  cmd_vel_out_topic cmd_vel
  source_timeout    1.0 s
  base_shift_correction True
  stop_pub_timeout  2.0 s
  polygons          ["FootprintApproach"]
  FootprintApproach type polygon, action_type "approach",
    time_before_collision 1.2 s, simulation_time_step 0.1 s, min_points 6
  observation_sources ["scan"] -> /scan_front_filtered, min_height 0.15

WHY "approach" RATHER THAN "stop". A stop polygon is a fixed region: anything
inside it halts the robot regardless of where the robot is going. An approach
polygon projects the ROBOT'S OWN FOOTPRINT forward along the commanded velocity
for time_before_collision seconds and asks whether it would hit anything. So the
same obstacle is fine when you are driving away from it and a stop when you are
driving at it, which is the behaviour you actually want in an aisle.

WHAT THIS FILE MEASURES rather than asserts:
  how far forward the projection reaches at each speed, and at what speed the
    projection is shorter than our own braking distance (which is when the
    monitor stops being able to help)
  how many simulation steps the projection actually takes
  what min_points 6 means in beams at each range, given our angular resolution
"""
import json
import math
import sys

# monitor
TIME_BEFORE = 1.2
SIM_STEP = 0.1
MIN_POINTS = 6
SOURCE_TIMEOUT = 1.0
STOP_PUB_TIMEOUT = 2.0
MIN_HEIGHT = 0.15

# robot, from nav2_params.yaml / the urdf
VX_MAX = 0.5
DECEL = 2.5                      # decel_lim_x magnitude
FOOT_X, FOOT_Y = 0.40, 0.29

# scanner, from cortex_amr.gazebo.xacro
BEAMS = 541
FOV = 4.712389
ANG_RES = FOV / (BEAMS - 1)


def projection_distance(v):
    return v * TIME_BEFORE


def braking_distance(v):
    return v * v / (2 * DECEL)


def beams_on_target(width_m, range_m):
    """How many beams fall on an object of this width at this range?"""
    if range_m <= 0:
        return 0
    subtended = 2 * math.atan2(width_m / 2, range_m)
    return int(subtended / ANG_RES)


def main():
    print("THE MONITOR SITS LAST")
    print("  %s -> collision_monitor -> %s"
          % ("cmd_vel_smoothed", "cmd_vel"))
    print("  so whatever it writes IS the command, and 10.6 measured what")
    print("  happens when its source goes stale: every command zeroed.")
    print()

    print("THE PROJECTION, %.1f s ahead in %.1f s steps (%d steps):"
          % (TIME_BEFORE, SIM_STEP, int(TIME_BEFORE / SIM_STEP)))
    rows = []
    for v in (0.1, 0.2, 0.3, 0.4, 0.5):
        pd = projection_distance(v)
        bd = braking_distance(v)
        margin = pd - bd
        rows.append({"v": v, "projection_m": round(pd, 3),
                     "braking_m": round(bd, 3), "margin_m": round(margin, 3),
                     "usable": margin > 0})
        print("  at %.1f m/s: projects %.3f m, brakes in %.3f m, margin %+.3f m %s"
              % (v, pd, bd, margin, "" if margin > 0 else "  <-- TOO LATE"))
    print()
    print("  the projection scales LINEARLY with speed and braking scales with")
    print("  the SQUARE, so there is a speed above which the monitor cannot save")
    print("  you. Solve v*1.2 = v^2/(2*2.5):")
    v_crit = TIME_BEFORE * 2 * DECEL
    print("  v = %.1f m/s. Our maximum is %.1f, so we are %.0fx inside it."
          % (v_crit, VX_MAX, v_crit / VX_MAX))
    print("  That margin is not luck: it is why a 0.5 m/s warehouse AMR is safe")
    print("  with a 1.2 s projection and a 2 m/s one would not be.")
    print()

    print("angular resolution %.3f deg, so min_points %d needs an object to"
          % (math.degrees(ANG_RES), MIN_POINTS))
    print("subtend at least %.2f deg:" % (MIN_POINTS * math.degrees(ANG_RES)))
    pts = []
    for r in (0.5, 1.0, 2.0, 3.0, 5.0, 6.0, 8.0, 12.0):
        n = beams_on_target(0.30, r)
        pts.append({"range_m": r, "beams": n, "triggers": n >= MIN_POINTS})
        print("  a 0.30 m object at %4.1f m: %2d beams %s"
              % (r, n, "-> triggers" if n >= MIN_POINTS else "-> IGNORED"))
    # WHERE min_points ACTUALLY BITES is small objects, not far ones: a 0.30 m
    # object only falls under 6 beams at 6 m, which is past obstacle_max_range
    # anyway. Sweep WIDTH at a range the monitor actually acts on.
    print()
    print("  at the 0.60 m projection distance, by object width:")
    widths = []
    for w in (0.30, 0.15, 0.10, 0.05, 0.02):
        n = beams_on_target(w, 0.60)
        widths.append({"width_m": w, "beams_at_0_6m": n,
                       "triggers": n >= MIN_POINTS})
        print("    %.2f m wide: %2d beams %s"
              % (w, n, "-> triggers" if n >= MIN_POINTS else "-> IGNORED"))
    thin = [x for x in widths if not x["triggers"]]
    if thin:
        print("  so min_points is a filter on THINNESS, not on distance: objects")
        print("  narrower than about %.2f m are invisible to it at the distance"
              % max(x["width_m"] for x in thin))
        print("  where it acts. A trailing cable is exactly that.")
    else:
        print("  every width tested triggers at 0.60 m")
    print()

    print("APPROACH vs STOP, and why ours is approach:")
    print("  a STOP polygon is fixed: anything inside halts the robot")
    print("  an APPROACH polygon projects OUR footprint along the COMMANDED")
    print("  velocity, so the same obstacle is fine when driving away from it")
    print("  footprint %.2f x %.2f m, projected %.3f m at full speed"
          % (FOOT_X * 2, FOOT_Y * 2, projection_distance(VX_MAX)))
    print("  swept area at full speed: %.3f m2"
          % ((FOOT_X * 2 + projection_distance(VX_MAX)) * FOOT_Y * 2))

    out = {
        "provenance": "collision_monitor block of nav2_params.yaml; robot limits "
                      "from the same file (measured); scanner from "
                      "cortex_amr.gazebo.xacro; the stale-source behaviour was "
                      "MEASURED",
        "config": {"time_before_collision": TIME_BEFORE,
                   "simulation_time_step": SIM_STEP,
                   "steps": int(TIME_BEFORE / SIM_STEP),
                   "min_points": MIN_POINTS,
                   "source_timeout": SOURCE_TIMEOUT,
                   "stop_pub_timeout": STOP_PUB_TIMEOUT,
                   "min_height": MIN_HEIGHT,
                   "action_type": "approach"},
        "projection_vs_braking": rows,
        "critical_speed_mps": v_crit,
        "our_max_mps": VX_MAX,
        "safety_factor": round(v_crit / VX_MAX, 1),
        "beams_on_a_300mm_object": pts,
        "beams_by_width_at_projection": widths,
        "angular_resolution_deg": round(math.degrees(ANG_RES), 3),
        "finding": (
            "The monitor projects the footprint v*1.2 m ahead while braking needs "
            "v^2/5, so the projection wins at low speed and loses at high: the "
            "crossover is %.1f m/s and our maximum is %.1f, a %.0fx margin. That "
            "is why a 1.2 s projection is adequate here and would not be on a "
            "faster machine. min_points 6 means a 0.30 m object stops being "
            "a filter on THINNESS rather than distance: a 0.30 m object keeps 6 "
            "beams out to 6 m, past the marking range anyway, but at the 0.60 m "
            "projection distance anything narrower than about %.2f m never "
            "reaches 6 beams. A trailing cable is exactly that. "
            "And because it sits LAST in the chain, a stale source does not "
            "degrade the monitor, it stops the robot:, measured."
            % (v_crit, VX_MAX, v_crit / VX_MAX,
               max([x["width_m"] for x in widths if not x["triggers"]] or [0]))),
    }
    with open("reference/safety_stop_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nwrote reference/safety_stop_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
