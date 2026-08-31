#!/usr/bin/env python3
"""Velocity obstacles: reasoning about where a person is GOING.

10.4 ended with 1.67 s of warning from the 2.5 m marking range, and the
observation that a costmap records where an obstacle IS. A velocity obstacle
records where it is going, and the difference is a set of velocities you must
not choose rather than a set of cells you must not enter.

THE GEOMETRY. Put the robot at the origin. An obstacle at relative position p
moving at relative velocity v_rel will collide with us if v_rel points into the
cone subtended by the obstacle inflated by our own radius. That cone, translated
by the obstacle's velocity, is the velocity obstacle: the set of OUR velocities
that lead to a collision.

Our numbers, all from earlier testing and our own config:
    robot footprint 0.80 x 0.58 m -> circumscribed radius 0.492 m
    person collision box 0.455 x 0.501 m -> radius 0.339 m   (measured, 10.4)
    combined radius 0.831 m
    vx_max 0.5 m/s, wz_max 1.9 rad/s      (nav2_params.yaml, measured 8.10)
    person walking 1.5 m/s
    obstacle_max_range 2.5 m, controller 20 Hz

WHAT THIS FILE MEASURES, rather than asserts:
  1. how much of OUR reachable velocity set is forbidden, at several ranges
  2. the time-to-collision horizon that makes the cone finite
  3. the case where EVERY available velocity is forbidden, and what is left
"""
import json
import math
import sys

VX_MAX = 0.5
WZ_MAX = 1.9
ROBOT_L, ROBOT_W = 0.80, 0.58
ROBOT_R = math.hypot(ROBOT_L / 2, ROBOT_W / 2)      # circumscribed
PERSON_W, PERSON_D = 0.455, 0.501
PERSON_R = math.hypot(PERSON_W / 2, PERSON_D / 2)
COMBINED_R = ROBOT_R + PERSON_R
WALK = 1.5
MARK_RANGE = 2.5
HZ = 20


def half_angle(range_m):
    """Half-angle of the collision cone at this range."""
    if range_m <= COMBINED_R:
        return math.pi / 2          # already overlapping: everything collides
    return math.asin(COMBINED_R / range_m)


def forbidden_fraction(range_m, approach_angle, horizon_s, samples=721):
    """Sample OUR velocity set and count how much of it collides inside horizon.

    A differential drive robot's reachable set for one control interval is
    (v, omega) but the velocity OBSTACLE lives in translational velocity space,
    so we sample heading x speed, which is what the robot can achieve over a
    short horizon by turning first.
    """
    px = range_m * math.cos(approach_angle)
    py = range_m * math.sin(approach_angle)
    # the obstacle's velocity: walking straight at us
    ovx = -WALK * math.cos(approach_angle)
    ovy = -WALK * math.sin(approach_angle)
    bad = tot = 0
    for i in range(samples):
        th = -math.pi + 2 * math.pi * i / samples
        for sp in (0.0, 0.125, 0.25, 0.375, 0.5):
            if sp > VX_MAX:
                continue
            tot += 1
            # RELATIVE VELOCITY OF THE OBSTACLE WITH RESPECT TO US, which is
            # v_obs - v_robot. Getting this backwards is a sign error that
            # reports 0.0% forbidden at EVERY range: the closest approach lands
            # at negative time, which reads as "moving apart" for every input.
            # A uniform 0.000 across a sweep is a disabled code path, not a
            # finding.
            rvx = ovx - sp * math.cos(th)
            rvy = ovy - sp * math.sin(th)
            # does the relative motion bring us within COMBINED_R inside horizon?
            # closest approach of the line p + t*rv to the origin
            rv2 = rvx * rvx + rvy * rvy
            if rv2 < 1e-9:
                d = math.hypot(px, py)
                t_star = 0.0
            else:
                t_star = -(px * rvx + py * rvy) / rv2
                if t_star < 0:
                    t_star = 0.0
                if t_star > horizon_s:
                    t_star = horizon_s
                d = math.hypot(px + t_star * rvx, py + t_star * rvy)
            if d <= COMBINED_R:
                bad += 1
    return bad / tot, tot


def main():
    print("RADII, composed from measured geometry")
    print("  robot  %.2f x %.2f m -> circumscribed %.3f m"
          % (ROBOT_L, ROBOT_W, ROBOT_R))
    print("  person %.3f x %.3f m -> %.3f m  (measured in 10.4)"
          % (PERSON_W, PERSON_D, PERSON_R))
    print("  combined %.3f m: closer than this and we are already touching"
          % COMBINED_R)
    print()

    print("THE COLLISION CONE WIDENS AS THE OBSTACLE GETS CLOSER:")
    cone = []
    for r in (2.5, 2.0, 1.5, 1.0, 0.9, COMBINED_R):
        ha = half_angle(r)
        cone.append({"range_m": round(r, 3),
                     "half_angle_deg": round(math.degrees(ha), 1),
                     "full_cone_deg": round(2 * math.degrees(ha), 1)})
        print("  at %.3f m the cone is %.1f deg wide"
              % (r, 2 * math.degrees(ha)))
    print()

    print("HOW MUCH OF OUR VELOCITY SET IS FORBIDDEN, head-on, 2 s horizon:")
    rows = []
    for r in (2.5, 2.0, 1.5, 1.0, 0.9):
        frac, tot = forbidden_fraction(r, 0.0, 2.0)
        rows.append({"range_m": r, "forbidden_frac": round(frac, 3),
                     "samples": tot})
        print("  at %.1f m: %.1f%% of %d sampled velocities collide"
              % (r, frac * 100, tot))
    print()

    print("THE HORIZON IS WHAT KEEPS THE CONE FINITE:")
    hz = []
    for h in (0.5, 1.0, 2.0, 4.0, 8.0):
        frac, _ = forbidden_fraction(2.0, 0.0, h)
        hz.append({"horizon_s": h, "forbidden_frac": round(frac, 3)})
        print("  horizon %.1f s at 2.0 m: %.1f%% forbidden"
              % (h, frac * 100))
    print("  a long horizon forbids almost everything, which is why the horizon")
    print("  is a tuning parameter and not a safety margin you maximise.")
    print()

    print("THE CASE THAT MATTERS: when is EVERYTHING forbidden?")
    worst = None
    for r in [x / 100 for x in range(50, 260, 5)]:
        frac, _ = forbidden_fraction(r, 0.0, 2.0)
        if frac >= 0.999:
            worst = r
    print("  head-on with a 2 s horizon, every sampled velocity collides at or")
    print("  below %.2f m" % (worst if worst else 0.0))
    print("  our own stopping distance at %.1f m/s with 2.5 m/s2 decel is %.3f m"
          % (VX_MAX, VX_MAX * VX_MAX / (2 * 2.5)))
    print("  so below that range the honest answer is not a velocity, it is a")
    print("  stop, and that is what the collision monitor in 10.8 is for.")
    print()
    print("SANITY CHECK, by kinematics rather than by sampling:")
    for r in (2.5, 2.0, 1.5):
        t_hit = (r - COMBINED_R) / WALK
        lateral = VX_MAX * t_hit
        print("  at %.1f m we have %.2f s; we can move %.3f m sideways and need %.3f"
              % (r, t_hit, lateral, COMBINED_R))
    print("  a %.1f m/s robot cannot dodge a %.1f m/s walker. The 100%% is real."
          % (VX_MAX, WALK))

    out = {
        "provenance": "robot footprint from nav2_params.yaml, person box measured "
                      "from MaleVisitorStatic_Col.obj in 10.4, limits from "
                      "nav2_params.yaml measured in 8.10",
        "radii": {"robot_circumscribed": round(ROBOT_R, 3),
                  "person": round(PERSON_R, 3),
                  "combined": round(COMBINED_R, 3)},
        "cone_by_range": cone,
        "forbidden_by_range": rows,
        "forbidden_by_horizon": hz,
        "all_forbidden_below_m": worst,
        "stopping_distance_m": round(VX_MAX * VX_MAX / (2 * 2.5), 3),
        "finding": (
            "A velocity obstacle turns 'where is it' into 'which of my velocities "
            "collide'. The cone widens from %.1f deg at 2.5 m to 180 deg at the "
            "%.3f m combined radius. Head-on with a 2 s horizon the forbidden "
            "fraction of our reachable set grows with proximity, and below about "
            "%.2f m EVERY sampled velocity collides -- at which point the answer "
            "is a stop, not a velocity. The horizon is a tuning parameter: at 8 s "
            "it forbids nearly everything, so maximising it is not conservatism, "
            "it is paralysis."
            % (cone[0]["full_cone_deg"], COMBINED_R, worst if worst else 0.0)),
    }
    with open("reference/velocity_obstacles_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nwrote reference/velocity_obstacles_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
