#!/usr/bin/env python3
"""A costmap from depth: what the RGBD camera adds, and what it costs.

10.2 ended on a voxel layer fed by one plane, with /camera/points bridged and
unused. This is what happens when you wire it in.

EVERY NUMBER READ OUT OF OUR OWN FILES, not remembered:

camera, from cortex_amr.gazebo.xacro:
    640 x 480 at 15 Hz, horizontal_fov 1.204 rad, clip 0.12 to 10.0 m
    vertical fov = 2*atan(tan(hfov/2) * 480/640) = 0.952 rad = 54.5 deg

camera height, composed from cortex_amr.urdf.xacro rather than guessed:
    base_link  0.105 clear + 0.040 skirt + 0.110 body_h/2 = 0.255
    deck_link  + 0.110 body_h/2 + 0.030 deck_h/2          = 0.395
    mast_link  + 0.030 deck_h/2 + 0.095                   = 0.520
    camera     + 0.060 (camera_joint)                     = 0.580 m

laser, for comparison (10.2): a single plane at 0.355 m.

voxel column, from local_costmap in nav2_params.yaml:
    z_resolution 0.05, z_voxels 16, origin_z 0.0  ->  a 0.80 m column
    mark_threshold 0

THE POINT OF THE module: the camera does not merely see MORE, it changes the
shape of the problem. A LaserScan contributes one voxel layer per cell. A
PointCloud2 contributes a contiguous RUN of layers, which is what finally makes
mark_threshold above zero mean something. But the camera has a narrow horizontal
field and a near clip, so it is worse than the laser in two ways that matter.
"""
import json
import math
import sys

# camera, measured
HFOV = 1.204
IMG_W, IMG_H = 640, 480
NEAR, FAR = 0.12, 10.0
RATE = 15
VFOV = 2 * math.atan(math.tan(HFOV / 2) * IMG_H / IMG_W)
CAM_Z = 0.580

# laser, from 10.2
LASER_Z = 0.355
LASER_FOV = 4.712389          # 270 deg
LASER_BEAMS = 541
LASER_RATE = 10

# voxel column
Z_RES, Z_VOXELS = 0.05, 16
COL_TOP = Z_VOXELS * Z_RES

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


def vertical_span(range_m):
    """What height band does the camera cover at this range?"""
    half = range_m * math.tan(VFOV / 2)
    return CAM_Z - half, CAM_Z + half


def seen_by_camera(lo, hi, range_m):
    """Does the camera's frustum intersect this object at this range?"""
    if range_m < NEAR or range_m > FAR:
        return False
    blo, bhi = vertical_span(range_m)
    # the object must overlap the band AND be inside the voxel column to be
    # representable at all
    return not (hi < max(blo, 0.0) or lo > bhi)


def voxels_marked(lo, hi):
    """How many of the 16 layers would a full-height observation fill?"""
    a = max(0.0, lo)
    b = min(hi, COL_TOP - 1e-9)
    if b < a:
        return 0
    return int(b / Z_RES) - int(a / Z_RES) + 1


def main():
    print("CAMERA, from cortex_amr.gazebo.xacro")
    print("  %dx%d at %d Hz, hfov %.3f rad (%.1f deg)"
          % (IMG_W, IMG_H, RATE, HFOV, math.degrees(HFOV)))
    print("  vfov = 2*atan(tan(hfov/2)*%d/%d) = %.3f rad (%.1f deg)"
          % (IMG_H, IMG_W, VFOV, math.degrees(VFOV)))
    print("  clip %.2f to %.1f m, mounted at %.3f m" % (NEAR, FAR, CAM_Z))
    print()
    print("LASER, for comparison: one plane at %.3f m, %.0f deg, %d beams, %d Hz"
          % (LASER_Z, math.degrees(LASER_FOV), LASER_BEAMS, LASER_RATE))
    print()

    print("THE HEIGHT BAND THE CAMERA COVERS, by range:")
    band = []
    for r in (0.5, 1.0, 1.5, 2.0, 3.0, 5.0):
        lo, hi = vertical_span(r)
        lo_c = max(lo, 0.0)
        band.append({"range_m": r, "low_m": round(lo_c, 3),
                     "high_m": round(hi, 3),
                     "covers_floor": lo <= 0.0})
        print("  at %4.1f m: %.3f to %.3f m %s"
              % (r, lo_c, hi, "(reaches the floor)" if lo <= 0.0 else ""))
    print()

    print("HAZARDS, laser plane vs camera frustum at 2.0 m:")
    print("  %-20s %-12s %-8s %-8s %-8s %s"
          % ("hazard", "span (m)", "laser", "camera", "voxels", "stored?"))
    rows = []
    l_seen = c_seen = 0
    for name, lo, hi in HAZARDS:
        ls = lo <= LASER_Z <= hi
        cs = seen_by_camera(lo, hi, 2.0)
        nv = voxels_marked(lo, hi) if cs else 0
        l_seen += int(ls)
        c_seen += int(cs)
        # OBSERVED and REPRESENTABLE are different facts. The overhanging load
        # at 1.40-1.95 m is inside the camera's 1.611 m band at 2 m, so the
        # camera genuinely sees it -- and the 0.80 m voxel column cannot store
        # it. Printing those as one column read as a contradiction.
        stored = cs and nv > 0
        rows.append({"hazard": name, "lo_m": lo, "hi_m": hi,
                     "laser": ls, "camera": cs, "voxels": nv,
                     "stored_in_column": stored})
        print("  %-20s %.2f-%-6.2f %-8s %-8s %-8s %s"
              % (name, lo, hi, "yes" if ls else "MISS",
                 "yes" if cs else "MISS", nv if nv else "-",
                 "yes" if stored else "NO (column is 0.80 m)"))
    print()
    stored_n = sum(1 for r in rows if r["stored_in_column"])
    print("  laser sees %d of %d, camera sees %d of %d, and only %d of those"
          % (l_seen, len(HAZARDS), c_seen, len(HAZARDS), stored_n))
    print("  can actually be STORED in a 0.80 m column. Seeing an obstacle and")
    print("  being able to represent it are two separate problems.")
    print()

    # --- WHAT THE CAMERA IS WORSE AT --------------------------------------
    print("WHERE THE CAMERA IS WORSE, and both matter:")
    print("  horizontal field  %.1f deg against the laser's %.0f deg"
          % (math.degrees(HFOV), math.degrees(LASER_FOV)))
    print("                    -> a %.0f deg blind wedge the laser covers"
          % (math.degrees(LASER_FOV) - math.degrees(HFOV)))
    near_lo, near_hi = vertical_span(NEAR)
    print("  near clip %.2f m   nothing closer than %.2f m exists at all" % (NEAR, NEAR))
    print("                    at that range it sees only %.3f to %.3f m"
          % (max(near_lo, 0.0), near_hi))
    print("  rate %d Hz        against the laser's %d Hz" % (RATE, LASER_RATE))
    print()

    # --- THE MARK_THRESHOLD CONSEQUENCE ----------------------------------
    print("WHY mark_threshold CAN NOW BE RAISED:")
    laser_layers = 1
    print("  a LaserScan fills   %d layer  per cell" % laser_layers)
    tall = [r for r in rows if r["camera"] and r["voxels"] >= 3]
    if tall:
        avg = sum(r["voxels"] for r in tall) / len(tall)
        print("  a PointCloud fills  %.1f layers per cell on average, for the %d"
              % (avg, len(tall)))
        print("                      hazards it can see at 3+ layers")
    print("  so a threshold of 2 rejects single-voxel noise and still marks a")
    print("  real object. With one plane it would have marked nothing.")

    out = {
        "provenance": "camera from cortex_amr.gazebo.xacro; mount height composed "
                      "from cortex_amr.urdf.xacro joint chain; voxel column from "
                      "local_costmap in nav2_params.yaml",
        "camera": {"width": IMG_W, "height": IMG_H, "rate_hz": RATE,
                   "hfov_rad": HFOV, "hfov_deg": round(math.degrees(HFOV), 1),
                   "vfov_rad": round(VFOV, 3),
                   "vfov_deg": round(math.degrees(VFOV), 1),
                   "near_m": NEAR, "far_m": FAR, "mount_z_m": CAM_Z},
        "laser": {"plane_z_m": LASER_Z, "fov_deg": 270, "beams": LASER_BEAMS,
                  "rate_hz": LASER_RATE},
        "band_by_range": band,
        "hazards": rows,
        "laser_seen": l_seen, "camera_seen": c_seen, "total": len(HAZARDS),
        "camera_worse": {
            "blind_wedge_deg": round(math.degrees(LASER_FOV)
                                     - math.degrees(HFOV), 1),
            "near_clip_m": NEAR,
            "rate_penalty_hz": LASER_RATE - RATE},
        "finding": (
            "The camera at 0.580 m with a 54.5 deg vertical field covers the floor "
            "to 1.61 m at 2 m range, so it sees %d of 9 hazards against the laser "
            "plane's %d. It also fills a RUN of voxel layers per cell instead of "
            "one, which is what lets mark_threshold rise above 0. But its "
            "horizontal field is 69.0 deg against the laser's 270, leaving a "
            "201.0 deg blind wedge, it is blind inside 0.12 m, and it runs at "
            "15 Hz against 10 Hz. Depth is an addition, not a replacement. And "
            "of the 8 it sees, only 7 fit the 0.80 m column: the overhanging load "
            "at 1.40-1.95 m is OBSERVED and not REPRESENTABLE, which is a "
            "different bug from not seeing it."
            % (c_seen, l_seen)),
    }
    with open("reference/costmap_from_depth_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nwrote reference/costmap_from_depth_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
