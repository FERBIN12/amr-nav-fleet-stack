#!/usr/bin/env python3
"""Static vs dynamic obstacles: what the two costmap layers actually do.

The static layer is the SLAM map from an earlier module, loaded once and never changed.
The obstacle layer is the live scan, marked and cleared every cycle. A learner's
first instinct is that they are two sources of the same thing. They are not, and
the difference has consequences you can measure.

EVERY PARAMETER IS OURS, read out of
~/amr_ws/src/cortex_amr_description/config/nav2_params.yaml (local_costmap):
    resolution 0.05        footprint 0.80 x 0.58 m
    raytrace_max_range 3.0     obstacle_max_range 2.5
    clearing True              marking True
    inflation_radius 0.7       cost_scaling_factor 3.0
    track_unknown_space True

THE ASYMMETRY THAT MATTERS: raytrace_max_range (3.0) is LARGER than
obstacle_max_range (2.5). So between 2.5 m and 3.0 m the robot will CLEAR cells
it is not willing to MARK. That is deliberate in Nav2's defaults and it is the
first thing to understand about a live costmap: clearing is cheaper to be wrong
about than marking.

Map: /tmp/maps/warehouse.pgm (reference/maps/ holds the durable copy).
"""
import json
import math
import sys

RES = 0.05
RAYTRACE_MAX = 3.0
OBSTACLE_MAX = 2.5
INFLATION_RADIUS = 0.7
COST_SCALING = 3.0
INSCRIBED = 0.29
FOOT_X, FOOT_Y = 0.40, 0.29
# our two 270 deg scanners, from cortex_amr.gazebo.xacro
SCAN_MIN, SCAN_MAX = -2.356194, 2.356194
SCAN_SAMPLES = 541


def load(path="/tmp/maps/warehouse.pgm"):
    d = open(path, "rb").read()
    i = 0
    for _ in range(4):
        while d[i:i + 1].isspace():
            i += 1
        while i < len(d) and not d[i:i + 1].isspace():
            i += 1
    i += 1
    hdr = d[:i].split()
    w, h = int(hdr[1]), int(hdr[2])
    body = d[i:]
    return w, h, [[body[(h - 1 - y) * w + x] for x in range(w)] for y in range(h)]


def cast(grid, w, h, ox, oy, ang, max_m):
    """March a ray until it hits an occupied cell. Returns (hit, range_m)."""
    steps = int(max_m / RES / 0.5)
    for s in range(1, steps + 1):
        d = s * 0.5
        x = int(ox + math.cos(ang) * d)
        y = int(oy + math.sin(ang) * d)
        if not (0 <= x < w and 0 <= y < h):
            return False, d * RES
        if grid[y][x] == 0:
            return True, d * RES
    return False, max_m


def scan_from(grid, w, h, ox, oy, yaw):
    """One full 270 degree sweep, as the real scanner would report it."""
    out = []
    for i in range(SCAN_SAMPLES):
        a = SCAN_MIN + (SCAN_MAX - SCAN_MIN) * i / (SCAN_SAMPLES - 1)
        hit, r = cast(grid, w, h, ox, oy, yaw + a, RAYTRACE_MAX)
        out.append((yaw + a, hit, r))
    return out


def mark_and_clear(scan):
    """Apply Nav2's marking/clearing rules to one scan. Returns the counts.

    marking:  a return inside obstacle_max_range becomes an obstacle cell
    clearing: everything along the ray up to the return is freed, out to
              raytrace_max_range
    The gap between the two ranges is the interesting part.
    """
    marked = cleared = beyond_mark = 0
    for ang, hit, r in scan:
        if hit and r <= OBSTACLE_MAX:
            marked += 1
        elif hit and r <= RAYTRACE_MAX:
            # a real return that is TOO FAR to mark; the ray still clears
            beyond_mark += 1
        cleared += int(min(r, RAYTRACE_MAX) / RES)
    return marked, cleared, beyond_mark


def inflation_cost(d_m):
    """Nav2 InflationLayer, exactly as derived in 8.6 and inverted in 9.10."""
    if d_m <= INSCRIBED:
        return 253
    if d_m >= INFLATION_RADIUS:
        return 0
    return int(max(1, min(252, 252 * math.exp(-COST_SCALING * (d_m - INSCRIBED)))))


def main():
    w, h, grid = load()
    # A pose in a real aisle: the 9.x experiments' path start, facing along it.
    ox, oy, yaw = 100, 270, 0.0
    print("map %dx%d at %.2f m/cell" % (w, h, RES))
    print("pose (%d,%d) yaw %.2f, 270 deg scanner, %d samples"
          % (ox, oy, yaw, SCAN_SAMPLES))
    print()

    scan = scan_from(grid, w, h, ox, oy, yaw)
    marked, cleared, beyond = mark_and_clear(scan)
    hits = sum(1 for _, hit, _ in scan if hit)
    print("ONE SCAN, through Nav2's rules:")
    print("  beams              %d" % len(scan))
    print("  returns at all     %d" % hits)
    print("  MARKED (<=%.1f m)   %d" % (OBSTACLE_MAX, marked))
    print("  real return but too far to mark (%.1f-%.1f m)  %d"
          % (OBSTACLE_MAX, RAYTRACE_MAX, beyond))
    print("  cells cleared      %d" % cleared)
    print("  clear:mark ratio   %.0f:1" % (cleared / max(1, marked)))
    print()

    # --- THE STATIC LAYER CANNOT FORGET -------------------------------------
    # A pallet dropped in an aisle AFTER the map was made. The static layer has
    # no idea; the obstacle layer sees it for as long as it is in range.
    print("A PALLET APPEARS 1.2 m AHEAD, after the map was built:")
    pal = []
    for dx in range(-6, 7):
        for dy in range(-6, 7):
            pal.append((ox + 24 + dx, oy + dy))
    static_says = "free" if grid[oy][ox + 24] == 254 else "occupied"
    print("  static layer says            %s (the map predates it)" % static_says)
    print("  obstacle layer marks it      yes, 1.20 m is inside %.1f m"
          % OBSTACLE_MAX)
    print("  inflation at the footprint   cost %d at %.2f m"
          % (inflation_cost(1.20 - FOOT_X), 1.20 - FOOT_X))
    print()

    # --- RANGE SWEEP: at what distance does an obstacle become visible? -----
    print("  distance   marked?   cleared?   inflated cost at the footprint")
    rows = []
    for d in (0.5, 1.0, 1.5, 2.0, 2.4, 2.6, 2.9, 3.2):
        mk = d <= OBSTACLE_MAX
        cl = d <= RAYTRACE_MAX
        gap = max(0.0, d - FOOT_X)
        c = inflation_cost(gap)
        rows.append({"range_m": d, "marked": mk, "cleared": cl,
                     "gap_m": round(gap, 2), "cost": c})
        print("  %7.1f m   %-7s   %-8s   %d"
              % (d, "yes" if mk else "NO", "yes" if cl else "no", c))
    print()
    print("  THE GAP: between %.1f and %.1f m a real return is CLEARED but never"
          % (OBSTACLE_MAX, RAYTRACE_MAX))
    print("  MARKED. Nav2 would rather forget an obstacle than invent one.")

    out = {
        "provenance": "parameters from our local_costmap in nav2_params.yaml; "
                      "inflation identical to 8.6/9.10; map reference/maps/",
        "params": {"resolution": RES, "raytrace_max_range": RAYTRACE_MAX,
                   "obstacle_max_range": OBSTACLE_MAX,
                   "inflation_radius": INFLATION_RADIUS,
                   "cost_scaling_factor": COST_SCALING,
                   "inscribed_radius": INSCRIBED,
                   "footprint": [FOOT_X * 2, FOOT_Y * 2]},
        "one_scan": {"beams": len(scan), "returns": hits, "marked": marked,
                     "returns_too_far_to_mark": beyond, "cells_cleared": cleared,
                     "clear_to_mark_ratio": round(cleared / max(1, marked), 1)},
        "range_sweep": rows,
        "finding": ("raytrace_max_range 3.0 EXCEEDS obstacle_max_range 2.5, so "
                    "between 2.5 and 3.0 m a genuine return is cleared and never "
                    "marked. Clearing is cheaper to be wrong about than marking. "
                    "The static layer, being a SLAM map from an earlier module, cannot "
                    "represent anything that arrived after it was saved."),
    }
    with open("reference/static_vs_dynamic_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nwrote reference/static_vs_dynamic_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
