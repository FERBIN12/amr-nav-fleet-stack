#!/usr/bin/env python3
"""The voxel layer: what a 3D costmap layer does, and what ours actually gets.

10.1 ended on what a flat scan misses. The voxel layer is Nav2's answer: keep a
COLUMN of occupancy per cell instead of one bit, so an obstacle at ankle height
and an overhanging shelf are different facts.

EVERY NUMBER BELOW IS READ OUT OF OUR OWN FILES, not remembered:

local_costmap voxel_layer, from
ros2_ws/src/cortex_amr_description/nav2_params.yaml:
    z_resolution 0.05      z_voxels 16        origin_z 0.0
    max_obstacle_height 2.0                   mark_threshold 0
    publish_voxel_map True
    observation_sources: scan   ->   topic /scan_front

robot geometry, from cortex_amr.urdf.xacro:
    clear 0.105 + skirt_h 0.040 + body_h 0.220
    body centre z = 0.105 + 0.040 + 0.110 = 0.255
    scanner  z    = body centre + (body_h/2 - 0.010) = 0.355 m
    robot top     = 0.105 + 0.040 + 0.220 = 0.365 m

TWO THINGS OUR CONFIG SAYS THAT DO NOT AGREE, and both are the module:

1. z_voxels 16 at z_resolution 0.05 is a column 0.80 m tall. But
   max_obstacle_height is 2.0 m. The column cannot represent anything above
   0.80 m, so the extra 1.2 m of declared height does not exist. Nav2 does not
   complain: the parameter is legal, it is just unreachable.

2. The only observation source is `scan`, a LaserScan on /scan_front. A
   LaserScan is a single plane. Our scanner sits at 0.355 m, which is voxel
   index 7 of 16. So a 16-layer column is being filled from ONE layer, and the
   other fifteen are permanently empty. The layer is doing 3D bookkeeping over
   2D data.

That is not a bug I introduced to make a point. It is what the config in an earlier module actually says, and it is the normal state of a Nav2 stack that has a laser and
no depth camera wired into the costmap. Our robot HAS an RGBD camera
(/camera/points is bridged), which is what 10.3 is about.
"""
import json
import math
import sys

Z_RES = 0.05
Z_VOXELS = 16
ORIGIN_Z = 0.0
MAX_OBSTACLE_HEIGHT = 2.0
MARK_THRESHOLD = 0
RES = 0.05

CLEAR = 0.105
SKIRT = 0.040
BODY_H = 0.220
BODY_Z = CLEAR + SKIRT + BODY_H / 2
SCAN_Z = BODY_Z + (BODY_H / 2 - 0.010)
ROBOT_TOP = CLEAR + SKIRT + BODY_H

# Real warehouse hazards, with heights a 2D scan at 0.355 m either sees or does
# not. Heights are the span the object occupies, floor-relative.
HAZARDS = [
    ("forklift tine", 0.02, 0.09),
    ("pallet, empty", 0.00, 0.14),
    ("pallet, loaded", 0.00, 1.20),
    ("kerb / floor lip", 0.00, 0.06),
    ("trailing cable", 0.00, 0.03),
    ("person, standing", 0.00, 1.75),
    ("shelf underside", 1.90, 2.40),
    ("overhanging load", 1.40, 1.95),
    ("dock leveller lip", 0.00, 0.11),
]


def voxel_index(z):
    """Which voxel does height z fall into? None if outside the column."""
    if z < ORIGIN_Z:
        return None
    i = int((z - ORIGIN_Z) / Z_RES)
    return i if i < Z_VOXELS else None


def seen_by_plane(lo, hi, plane_z=SCAN_Z):
    return lo <= plane_z <= hi


def main():
    col_top = ORIGIN_Z + Z_VOXELS * Z_RES
    print("VOXEL COLUMN, from our nav2_params.yaml")
    print("  z_voxels %d x z_resolution %.2f = %.2f m tall, origin_z %.1f"
          % (Z_VOXELS, Z_RES, Z_VOXELS * Z_RES, ORIGIN_Z))
    print("  column spans %.2f to %.2f m" % (ORIGIN_Z, col_top))
    print("  max_obstacle_height declared %.1f m -> UNREACHABLE above %.2f m"
          % (MAX_OBSTACLE_HEIGHT, col_top))
    print("  unusable declared height: %.2f m" % (MAX_OBSTACLE_HEIGHT - col_top))
    print()
    print("ROBOT, from cortex_amr.urdf.xacro")
    print("  body centre %.3f m, scanner %.3f m, top %.3f m"
          % (BODY_Z, SCAN_Z, ROBOT_TOP))
    print("  the scan plane fills voxel index %d of %d"
          % (voxel_index(SCAN_Z), Z_VOXELS))
    print("  so %d of %d layers are permanently empty"
          % (Z_VOXELS - 1, Z_VOXELS))
    print()

    print("WHAT A PLANE AT %.3f m SEES:" % SCAN_Z)
    print("  %-20s %-14s %-8s %s" % ("hazard", "span (m)", "in col?", "seen?"))
    rows = []
    seen = miss = 0
    for name, lo, hi in HAZARDS:
        s = seen_by_plane(lo, hi)
        # is any part of it inside the voxel column at all?
        in_col = (lo < col_top)
        rows.append({"hazard": name, "lo_m": lo, "hi_m": hi,
                     "in_column": in_col, "seen_by_plane": s})
        seen += int(s)
        miss += int(not s)
        print("  %-20s %.2f - %-8.2f %-8s %s"
              % (name, lo, hi, "yes" if in_col else "NO",
                 "yes" if s else "MISSED"))
    print()
    print("  seen %d of %d, MISSED %d" % (seen, len(HAZARDS), miss))
    print()

    # Which voxel would each hazard occupy, if the data existed?
    print("IF A DEPTH CAMERA FILLED THE COLUMN, voxels each hazard would mark:")
    for name, lo, hi in HAZARDS:
        a, b = voxel_index(lo), voxel_index(min(hi, col_top - 1e-9))
        if a is None:
            span = "outside the column"
        elif b is None:
            span = "voxel %d upward, clipped at the top" % a
        else:
            span = "voxels %d-%d (%d of %d)" % (a, b, b - a + 1, Z_VOXELS)
        print("  %-20s %s" % (name, span))
    print()
    print("MARK_THRESHOLD is %d, so a SINGLE observation in a column marks the"
          % MARK_THRESHOLD)
    print("cell as an obstacle. With one plane of data that is the only setting")
    print("that can work: requiring 2 hits in a column no source ever fills")
    print("twice would mark nothing at all.")

    out = {
        "provenance": "voxel_layer block of local_costmap in nav2_params.yaml; "
                      "robot geometry from cortex_amr.urdf.xacro",
        "column": {"z_resolution": Z_RES, "z_voxels": Z_VOXELS,
                   "origin_z": ORIGIN_Z, "height_m": round(Z_VOXELS * Z_RES, 2),
                   "max_obstacle_height_declared": MAX_OBSTACLE_HEIGHT,
                   "unreachable_m": round(MAX_OBSTACLE_HEIGHT
                                          - Z_VOXELS * Z_RES, 2),
                   "mark_threshold": MARK_THRESHOLD},
        "robot": {"body_centre_z": round(BODY_Z, 3),
                  "scanner_z": round(SCAN_Z, 3),
                  "top_z": round(ROBOT_TOP, 3),
                  "scan_voxel_index": voxel_index(SCAN_Z),
                  "layers_empty": Z_VOXELS - 1},
        "hazards": rows,
        "seen": seen, "missed": miss, "total": len(HAZARDS),
        "finding": (
            "Our voxel_layer's only observation source is a LaserScan on "
            "/scan_front, a single plane at 0.355 m. That fills voxel 7 of 16 "
            "and leaves 15 layers permanently empty: 3D bookkeeping over 2D "
            "data. Separately, z_voxels 16 x z_resolution 0.05 is a 0.80 m "
            "column, so the declared max_obstacle_height of 2.0 m is "
            "unreachable by 1.20 m and Nav2 never says so. Of 9 real warehouse "
            "hazards, the plane sees %d and misses %d." % (seen, miss)),
    }
    with open("reference/voxel_layer_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nwrote reference/voxel_layer_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
