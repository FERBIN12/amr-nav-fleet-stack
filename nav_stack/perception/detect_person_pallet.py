#!/usr/bin/env python3
"""Detecting a person and a pallet: what each sensor actually returns.

10.3 compared a laser plane against a depth frustum on invented hazard heights.
This module uses the REAL objects in our warehouse world, with their REAL
collision geometry, and asks what each sensor reports.

THE OBJECTS ARE IN THE WORLD FILE, not invented:
  /opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/warehouse.sdf
    MaleVisitorOnPhone  "Person 1 - Standing"  pose  1  -1 0    yaw 1.57
    Casual female       "Person 2 - Walking"   pose -12  15 0
    pallet_box_mobile   "pallet_box_0"         pose -4   12 0.01

THE PERSON'S COLLISION GEOMETRY IS ONE BOX, measured from the cached mesh
~/.gz/fuel/.../malevisitoronphone/2/meshes/MaleVisitorStatic_Col.obj:
    8 vertices. x 0.455 m, y 0.501 m, z 0.000 to 1.745 m.
That matters twice over. First, a 1.745 m person is taller than our 0.80 m voxel
column, so most of them cannot be stored. Second, the COLLISION mesh is a box
while the VISUAL mesh is a detailed human: a depth camera renders the visual
mesh and a raycasting lidar hits the collision mesh, so the two sensors do not
even agree on the shape of the obstacle.

SENSORS, both measured in earlier testing:
    laser   plane at 0.355 m, 270 deg, 541 beams, 10 Hz, range 0.06-12.0
    camera  0.580 m, hfov 69.0 deg, vfov 54.5 deg, clip 0.12-10.0, 15 Hz
"""
import json
import math
import sys

LASER_Z = 0.355
LASER_MIN, LASER_MAX = 0.06, 12.0
CAM_Z = 0.580
VFOV = 0.952
HFOV = 1.204
CAM_NEAR, CAM_FAR = 0.12, 10.0
Z_RES, Z_VOXELS = 0.05, 16
COL_TOP = Z_VOXELS * Z_RES

# measured from the collision meshes / world file
PERSON = {"name": "standing person", "w": 0.455, "d": 0.501, "h": 1.745,
          "pose": (1.0, -1.0)}
# pallet_box_mobile: a euro-pallet box. Height from the model's own collision
# box; the world places it at z=0.01.
PALLET = {"name": "pallet box", "w": 1.20, "d": 0.80, "h": 0.97,
          "pose": (-4.0, 12.0)}
SPAWN = (-2.0, 0.0)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def laser_sees(obj):
    """A plane at 0.355 m: does the object span that height?"""
    return 0.0 <= LASER_Z <= obj["h"]


def camera_band(range_m):
    half = range_m * math.tan(VFOV / 2)
    return max(0.0, CAM_Z - half), CAM_Z + half


def camera_sees(obj, range_m):
    if not (CAM_NEAR <= range_m <= CAM_FAR):
        return False
    lo, hi = camera_band(range_m)
    return not (obj["h"] < lo or 0.0 > hi)


def stored_fraction(obj):
    """How much of the object's height fits the voxel column?"""
    return min(obj["h"], COL_TOP) / obj["h"]


def main():
    print("OBJECTS, from warehouse.sdf and their collision meshes")
    for o in (PERSON, PALLET):
        print("  %-16s %.3f x %.3f x %.3f m at (%.1f, %.1f)"
              % (o["name"], o["w"], o["d"], o["h"], o["pose"][0], o["pose"][1]))
    print("  robot spawns at (%.1f, %.1f)" % SPAWN)
    print()

    rows = []
    for o in (PERSON, PALLET):
        r = dist(SPAWN, o["pose"])
        ls = laser_sees(o)
        cs = camera_sees(o, min(r, 2.0))       # evaluate at 2 m, a realistic approach
        frac = stored_fraction(o)
        lo, hi = camera_band(2.0)
        rows.append({"object": o["name"], "h_m": o["h"],
                     "range_from_spawn_m": round(r, 2),
                     "laser_sees": ls, "camera_sees_at_2m": cs,
                     "camera_band_at_2m": [round(lo, 3), round(hi, 3)],
                     "fraction_storable": round(frac, 3),
                     "voxels_of_16": min(Z_VOXELS,
                                         int(min(o["h"], COL_TOP) / Z_RES))})
        print("%s" % o["name"].upper())
        print("  height %.3f m, %.2f m from spawn" % (o["h"], r))
        print("  laser plane at %.3f m intersects it:  %s"
              % (LASER_Z, "yes" if ls else "NO"))
        print("  camera band at 2 m is %.3f to %.3f m:  %s"
              % (lo, hi, "yes" if cs else "NO"))
        # THE COLUMN IS FULL AND STILL TOO SHORT. Both facts are true and they
        # sound contradictory side by side, so say which is which.
        nv = min(Z_VOXELS, int(min(o["h"], COL_TOP) / Z_RES))
        print("  the column is %d of %d voxels FULL, and that holds only %.0f%%"
              % (nv, Z_VOXELS, frac * 100))
        print("  of its %.3f m height. Saturated is not the same as sufficient."
              % o["h"])
        print()

    # --- THE TWO SENSORS DISAGREE ABOUT THE SHAPE -------------------------
    print("THE SENSORS DISAGREE ABOUT THE SHAPE OF A PERSON:")
    print("  collision mesh: 8 vertices, one box, %.3f x %.3f m"
          % (PERSON["w"], PERSON["d"]))
    print("  visual mesh:    a detailed human")
    print("  a raycasting lidar hits the COLLISION mesh -> a rectangle")
    print("  a depth camera renders the VISUAL mesh     -> a human silhouette")
    print("  so the costmap footprint of a person depends on which sensor")
    print("  marked it, and neither is wrong.")
    print()

    # --- WHY A PERSON IS THE HARD CASE -----------------------------------
    print("WHY A PERSON IS THE HARD CASE, and it is not height:")
    walk = 1.5
    for r in (3.0, 2.0, 1.0, 0.5):
        t = r / walk
        print("  closing at %.1f m/s: %.2f m away is %.2f s of warning"
              % (walk, r, t))
    commit = 2.5      # obstacle_max_range
    print("  marking starts at %.1f m -> %.2f s before contact"
          % (commit, commit / walk))
    print("  our controller runs at 20 Hz, so that is %d control cycles"
          % int(commit / walk * 20))
    print()
    print("  the pallet does not move, so its %.2f s is a floor, not a budget."
          % (commit / walk))

    out = {
        "provenance": "objects and poses from nav2_minimal_tb4_sim warehouse.sdf; "
                      "person collision box measured from the cached "
                      "MaleVisitorStatic_Col.obj (8 vertices); sensor geometry "
                      "from cortex_amr.gazebo.xacro and earlier testing",
        "objects": rows,
        "person_collision_box": {"vertices": 8, "w": PERSON["w"],
                                 "d": PERSON["d"], "h": PERSON["h"]},
        "shape_disagreement": (
            "a raycasting lidar hits the 8-vertex COLLISION box and reports a "
            "rectangle; a depth camera renders the detailed VISUAL mesh and "
            "reports a human silhouette. The costmap footprint of a person "
            "depends on which sensor marked it."),
        "warning_time": {"walk_speed_mps": walk,
                         "mark_range_m": commit,
                         "seconds": round(commit / walk, 2),
                         "control_cycles_at_20hz": int(commit / walk * 20)},
        "finding": (
            "The person is 1.745 m tall, so the laser plane at 0.355 m sees it and "
            "the camera sees it, but only %.0f%% of its height fits the 0.80 m "
            "column -- the column saturates at 16 of 16 voxels and still holds "
            "under half the person, which is not the same thing. The pallet at "
            "0.97 m is the same story. Detection is not the "
            "hard part for either object: REPRESENTATION is. And a walking person "
            "at 1.5 m/s gives %.2f s from the 2.5 m marking range, which is %d "
            "control cycles at 20 Hz."
            % (stored_fraction(PERSON) * 100, commit / walk,
               int(commit / walk * 20))),
    }
    with open("reference/detect_person_pallet_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("wrote reference/detect_person_pallet_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
