#!/usr/bin/env python3
"""Sensor failure modes, and what each one looks like from the costmap.

10.6 fixed five things that were not sensor failures at all: a spawn outside the
map, a missing filter node, a diverged AMCL, and two bugs in my own measuring
code. This module is about the failures that ARE the sensor, and the point is
that most of them do not look like failures. They look like obstacles, or like
free space, and the costmap cannot tell you which.

EVERY NUMBER IS EITHER OUR OWN CONFIG OR DERIVED FROM IT:
  scanner    541 beams over 270 deg, range 0.06-12.0 m, 10 Hz
             (cortex_amr.gazebo.xacro)
  camera     640x480, hfov 69.0 deg, vfov 54.5 deg, clip 0.12-10.0 m, 15 Hz
  costmap    resolution 0.05, obstacle_max_range 2.5, raytrace_max_range 3.0,
             inflation_radius 0.70, cost_scaling_factor 3.0
  monitor    source_timeout 1.0 s, and it sits LAST in the chain
             (measured: a stale source zeroes every command)

THE CLASSIFICATION THAT MATTERS. A sensor failure is only actionable if you can
tell it apart from a real reading. So each mode below is scored on whether the
costmap can distinguish it, not on how dramatic it is.
"""
import json
import math
import sys

BEAMS = 541
FOV = 4.712389
RANGE_MIN, RANGE_MAX = 0.06, 12.0
SCAN_HZ = 10
CAM_HZ = 15
RES = 0.05
OBSTACLE_MAX = 2.5
RAYTRACE_MAX = 3.0
INFLATION_R = 0.70
SOURCE_TIMEOUT = 1.0

# Each mode: what the sensor reports, what the costmap does with it, and whether
# anything in the stack can tell it from a genuine reading.
MODES = [
    {
        "mode": "dust or fog in a laser",
        "reports": "short random returns, well inside the true surface",
        "costmap": "MARKS phantom obstacles all round, then clears them next scan",
        "distinguishable": False,
        "why": "a short return is exactly what a real close obstacle looks like",
        "signature": "marks appear and vanish at the scan rate rather than persisting",
        "shape": "false_mark",
    },
    {
        "mode": "reflective floor under a depth camera",
        "reports": "points below the floor plane, or no points at all",
        "costmap": "either marks a hole it can never clear, or sees free space",
        "distinguishable": True,
        "why": "returns BELOW the known ground plane are geometrically impossible",
        "signature": "z < 0 in the camera frame, which a min_height filter catches",
        "shape": "both",
    },
    {
        "mode": "the robot seeing its own chassis",
        "reports": "a constant close return in a FIXED arc of the scan",
        "costmap": "marks a permanent obstacle that moves with the robot",
        "distinguishable": True,
        "why": "it is at the same bearing and range in every single scan",
        "signature": "26-34 beams at a fixed bearing, measured on this robot",
        "shape": "false_mark",
    },
    {
        "mode": "a dropped or stale source",
        "reports": "nothing at all",
        "costmap": "keeps the last data, so the world appears FROZEN, not empty",
        "distinguishable": True,
        "why": "the collision monitor times out after 1.0 s and says so",
        "signature": "'Robot to stop due to invalid source', and every command zeroed",
        "shape": "frozen",
    },
    {
        "mode": "a glass or mesh surface",
        "reports": "no return: the beam passes through or scatters",
        "costmap": "CLEARS the cells, recording solid glass as free space",
        "distinguishable": False,
        "why": "a missing return is indistinguishable from open space",
        "signature": "none from this sensor; needs a second modality to catch",
        "shape": "false_clear",
    },
    {
        "mode": "sunlight into a depth camera",
        "reports": "large regions of invalid depth",
        "costmap": "clears them, because no return means no obstacle",
        "distinguishable": True,
        "why": "the driver flags invalid pixels rather than reporting a range",
        "signature": "NaN density in the cloud, and it correlates with time of day",
        "shape": "false_clear",
    },
    {
        "mode": "a miscalibrated extrinsic",
        "reports": "correct ranges at the wrong bearing or height",
        "costmap": "marks real obstacles in the wrong cells, consistently",
        "distinguishable": False,
        "why": "every reading is individually plausible; only the map is wrong",
        "signature": "SLAM will not close a loop, which is a symptom a step further down the pipeline",
        "shape": "displaced",
    },
]


ROBOT_HALF_W = 0.29


def phantom_cost(range_m):
    """A false mark at this range costs you what?

    The INFLATED AREA is the same wherever the mark lands, so reporting it per
    range says nothing. What varies is the CORRIDOR the keep-out blocks: a mark
    dead ahead forces a lateral detour of inflation_radius + our half width, and
    the angle you must turn through to achieve that shrinks with range.
    """
    if range_m > OBSTACLE_MAX:
        return 0, 0.0, 0.0
    r_cells = INFLATION_R / RES
    cells = int(math.pi * r_cells * r_cells)
    detour = INFLATION_R + ROBOT_HALF_W
    turn_deg = math.degrees(math.atan2(detour, range_m))
    return cells, detour, turn_deg


def main():
    print("OUR SENSORS, for reference")
    print("  scanner %d beams / %.0f deg, %.2f-%.1f m, %d Hz"
          % (BEAMS, math.degrees(FOV), RANGE_MIN, RANGE_MAX, SCAN_HZ))
    print("  camera  640x480, 69.0 x 54.5 deg, 0.12-10.0 m, %d Hz" % CAM_HZ)
    print("  monitor source_timeout %.1f s, and it sits LAST in the chain"
          % SOURCE_TIMEOUT)
    print()

    print("SEVEN FAILURE MODES, scored on whether you can TELL:")
    dist = 0
    for m in MODES:
        d = m["distinguishable"]
        dist += int(d)
        print("  %-34s %s" % (m["mode"], "distinguishable" if d else "INVISIBLE"))
        print("      reports:  %s" % m["reports"])
        print("      costmap:  %s" % m["costmap"])
        print("      %s" % m["why"])
    print()
    print("  %d of %d can be told apart from a genuine reading."
          % (dist, len(MODES)))
    print("  The other %d cannot, and that is the useful fact: they are not"
          % (len(MODES) - dist))
    print("  detected, they are DESIGNED AROUND with a second sensor.")
    print()

    print("WHAT ONE FALSE MARK COSTS, at 0.05 m resolution:")
    rows = []
    for r in (0.5, 1.0, 2.0, 2.4, 2.6):
        cells, detour, turn = phantom_cost(r)
        rows.append({"range_m": r, "inflated_cells": cells,
                     "lateral_detour_m": round(detour, 3),
                     "turn_required_deg": round(turn, 1)})
        if cells:
            print("  at %.1f m: %d inflated cells, %.2f m of lateral detour, "
                  "a %.0f deg turn" % (r, cells, detour, turn))
        else:
            print("  at %.1f m: beyond obstacle_max_range, not marked at all" % r)
    print("  the AREA never changes; the TURN does, and a close phantom is the")
    print("  expensive one because you must turn hardest to get round it.")
    print()

    print("THE TWO SHAPES OF FAILURE, and they need opposite responses:")
    # Classify on an EXPLICIT field. Substring-matching the description counted
    # modes twice ("marks a hole it can never clear" contains both words), which
    # is how 7 modes produced 4 marks and 4 clears.
    false_marks = [m for m in MODES if m["shape"] in ("false_mark", "both")]
    false_clears = [m for m in MODES if m["shape"] in ("false_clear", "both")]
    other = [m for m in MODES if m["shape"] in ("frozen", "displaced")]
    print("  FALSE MARKS  (%d modes): the robot stops for nothing."
          % len(false_marks))
    print("               recoverable, because the next scan clears it.")
    print("  FALSE CLEARS (%d modes): the robot drives into something."
          % len(false_clears))
    print("               NOT recoverable, because nothing contradicts free space.")
    print("  NEITHER      (%d modes): a stale source FREEZES the world, and a bad"
          % len(other))
    print("               extrinsic DISPLACES it. Both look entirely healthy.")
    print()
    print("  That asymmetry is the same one Nav2's own defaults encode: raytrace")
    print("  3.0 m against obstacle 2.5 m, from an earlier module. Clearing is cheap")
    print("  to be wrong about ONLY when something else will mark it later.")

    out = {
        "provenance": "sensor specs from cortex_amr.gazebo.xacro; costmap and "
                      "collision monitor values from nav2_params.yaml; the "
                      "chassis-arc and stale-source signatures were MEASURED in "
                      "10.6 (34 of 541 beams, and 'invalid source' zeroing "
                      "every command)",
        "sensors": {"beams": BEAMS, "fov_deg": round(math.degrees(FOV), 1),
                    "scan_hz": SCAN_HZ, "cam_hz": CAM_HZ,
                    "obstacle_max_range": OBSTACLE_MAX,
                    "raytrace_max_range": RAYTRACE_MAX,
                    "inflation_radius": INFLATION_R,
                    "source_timeout_s": SOURCE_TIMEOUT},
        "modes": MODES,
        "distinguishable": dist, "total_modes": len(MODES),
        "false_mark_modes": len(false_marks),
        "false_clear_modes": len(false_clears),
        "phantom_cost": rows,
        "finding": (
            "Of %d failure modes, %d can be told apart from a genuine reading and "
            "%d cannot. The undetectable ones are not caught, they are designed "
            "around with a second modality. And the two shapes need opposite "
            "responses: a false MARK stops you for nothing and is cleared by the "
            "next scan, while a false CLEAR drives you into something and nothing "
            "contradicts it. One false mark at 1.0 m inflates over %d cells, a "
            "%.2f m keep-out, which is why phantom marks are expensive even "
            "though they are recoverable."
            % (len(MODES), dist, len(MODES) - dist,
               phantom_cost(1.0)[0], INFLATION_R)),
    }
    with open("reference/sensor_failures_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nwrote reference/sensor_failures_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
