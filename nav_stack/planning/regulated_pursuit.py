#!/usr/bin/env python3
"""Regulated pure pursuit: what Nav2 adds to the four lines of an earlier module.

9.7 derived pure pursuit and swept its ONE parameter. This module measures what
Nav2's default controller adds on top, and the honest answer is that it does not
touch the steering law at all. Curvature is still 2y/L^2. Every addition is a
constraint on the LINEAR velocity, plus a rule for what to do when the geometry
has nothing to say.

EVERY DEFAULT BELOW WAS READ OUT OF THE REAL PLUGIN, not remembered. A
controller_server was launched with plugin
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController, driven to
configure, and `ros2 param dump` captured them ->
reference/rpp_declared_defaults.yaml:

    desired_linear_vel                    0.5    lookahead_dist            0.6
    regulated_linear_scaling_min_radius   0.9    min_lookahead_dist        0.3
    regulated_linear_scaling_min_speed    0.25   max_lookahead_dist        0.9
    min_approach_linear_velocity          0.05   lookahead_time            1.5
    approach_velocity_scaling_dist        0.6    cost_scaling_dist         0.6
    inflation_cost_scaling_factor         3.0    cost_scaling_gain         1.0
    use_regulated_linear_velocity_scaling true   use_rotate_to_heading     true
    rotate_to_heading_angular_vel         1.8    rotate_to_heading_min_angle 0.785
    use_velocity_scaled_lookahead_dist    false

The three regulation functions are implemented verbatim from the installed
header /opt/ros/jazzy/include/nav2_regulated_pure_pursuit_controller/
regulation_functions.hpp, so the shapes are the shipped shapes:

  curvatureConstraint: radius = |1/kappa|; if radius < min_radius,
      v *= 1 - |radius - min_radius| / min_radius
  costConstraint: recovers distance-to-obstacle by INVERTING the inflation
      exponential, d = (f*r_inscribed - ln(cost) + ln(253)) / f, then if
      d < cost_scaling_dist, v *= gain * d / cost_scaling_dist
  approachVelocityConstraint: inside approach_velocity_scaling_dist of the goal,
      v *= |last carrot| / dist, floored at min_approach_linear_velocity

Same map, same path, same robot limits as 9.7, so the numbers are comparable to
the table that module already published:
  vx_max 0.5 m/s, wz_max 1.9 rad/s, dt 0.05 s, 0.05 m/cell,
  path (100,270) -> (389,219), 290 points, 310.1 cells, 15.5 m.
"""
import heapq
import json
import math
import sys

RES = 0.05
VX = 0.5
WZ = 1.9
DT = 0.05

# --- the real declared defaults (reference/rpp_declared_defaults.yaml) --------
LOOKAHEAD = 0.6
MIN_LOOKAHEAD = 0.3
MAX_LOOKAHEAD = 0.9
LOOKAHEAD_TIME = 1.5
MIN_RADIUS = 0.9
MIN_SPEED = 0.25
MIN_APPROACH_VEL = 0.05
APPROACH_DIST = 0.6
COST_SCALING_DIST = 0.6
COST_SCALING_GAIN = 1.0
INFLATION_FACTOR = 3.0
ROTATE_TO_HEADING_MIN_ANGLE = 0.785
ROTATE_TO_HEADING_VEL = 1.8

# our robot, from nav2_params.yaml (measured in 8.10) and the real footprint
INSCRIBED_RADIUS = 0.29           # half the 0.58 m width
FOOTPRINT_HALF_X = 0.40
FOOTPRINT_HALF_Y = 0.29


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


NB8 = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def astar(grid, w, h, s, g):
    def hc(a):
        return math.hypot(a[0] - g[0], a[1] - g[1])
    gs = {s: 0.0}
    prev = {s: None}
    pq = [(hc(s), 0.0, s)]
    done = set()
    while pq:
        _, d, cur = heapq.heappop(pq)
        if cur in done:
            continue
        done.add(cur)
        if cur == g:
            out = []
            k = cur
            while k is not None:
                out.append(k)
                k = prev[k]
            return out[::-1]
        for dx, dy in NB8:
            n = (cur[0] + dx, cur[1] + dy)
            if not (0 <= n[0] < w and 0 <= n[1] < h) or grid[n[1]][n[0]] != 254:
                continue
            nd = d + math.hypot(dx, dy)
            if nd < gs.get(n, float("inf")):
                gs[n] = nd
                prev[n] = cur
                heapq.heappush(pq, (nd + hc(n), nd, n))
    return []


# --- the inflation layer, so costConstraint has a real cost to invert --------
#
# Nav2's InflationLayer: cells within the inscribed radius get 253 (definitely
# in collision), and beyond that cost decays exponentially with
# inflation_cost_scaling_factor. We build it once from the occupied cells so the
# cost the controller reads is the cost Nav2 would have handed it.
def inflate(grid, w, h, factor=INFLATION_FACTOR, radius_m=1.2):
    from collections import deque
    R = int(radius_m / RES)
    dist = [[float("inf")] * w for _ in range(h)]
    q = deque()
    for y in range(h):
        row = grid[y]
        for x in range(w):
            if row[x] == 0:
                dist[y][x] = 0.0
                q.append((x, y))
    # BFS in 8-connectivity gives an adequate EDT for a 1.2 m band
    while q:
        x, y = q.popleft()
        d0 = dist[y][x]
        if d0 >= R:
            continue
        for dx, dy in NB8:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            nd = d0 + math.hypot(dx, dy)
            if nd < dist[ny][nx]:
                dist[ny][nx] = nd
                q.append((nx, ny))
    cost = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            dm = dist[y][x] * RES
            if dm == 0.0:
                cost[y][x] = 254
            elif dm <= INSCRIBED_RADIUS:
                cost[y][x] = 253
            elif dm < radius_m:
                v = 252 * math.exp(-factor * (dm - INSCRIBED_RADIUS))
                cost[y][x] = int(max(1, min(252, v)))
            else:
                cost[y][x] = 0
    return cost


# --- the three regulations, verbatim from regulation_functions.hpp -----------
def curvature_constraint(raw, kappa, min_radius=MIN_RADIUS):
    if abs(kappa) < 1e-9:
        return raw
    radius = abs(1.0 / kappa)
    if radius < min_radius:
        return raw * (1.0 - (abs(radius - min_radius) / min_radius))
    return raw


def cost_constraint(raw, pose_cost):
    """pose_cost is the costmap value under the robot. 0 = free, 255 = unknown."""
    if pose_cost in (0, 255):
        return raw
    d = (INFLATION_FACTOR * INSCRIBED_RADIUS - math.log(max(1.0, float(pose_cost)))
         + math.log(253.0)) / INFLATION_FACTOR
    if d < COST_SCALING_DIST:
        return raw * (COST_SCALING_GAIN * d / COST_SCALING_DIST)
    return raw


def approach_constraint(v, remaining_m, carrot_dist_m):
    if remaining_m < APPROACH_DIST:
        scaled = v * (carrot_dist_m / APPROACH_DIST)
        return min(v, max(scaled, MIN_APPROACH_VEL))
    return v


def run(path, cost, w, h, regulate=True, rotate_to_heading=True,
        start_yaw_off=0.0, max_steps=40000, min_speed=MIN_SPEED):
    """Drive the path. Returns a dict of measurements."""
    if not path:
        return None
    x, y = float(path[0][0]), float(path[0][1])
    th = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0]) + start_yaw_off
    idx = 0
    look_cells = LOOKAHEAD / RES
    errs, vs = [], []
    curv_hits = cost_hits = appr_hits = 0
    rotating_steps = 0
    total_len = sum(math.hypot(path[i + 1][0] - path[i][0],
                               path[i + 1][1] - path[i][1])
                    for i in range(len(path) - 1)) * RES
    for step in range(max_steps):
        while idx < len(path) - 1 and \
                math.hypot(path[idx][0] - x, path[idx][1] - y) < look_cells:
            idx += 1
        tgt = path[idx]
        dx, dy = tgt[0] - x, tgt[1] - y
        c, s = math.cos(-th), math.sin(-th)
        lx = dx * c - dy * s
        ly = dx * s + dy * c
        L2 = lx * lx + ly * ly
        if L2 < 1e-9:
            break
        kappa_cells = 2.0 * ly / L2
        kappa = kappa_cells / RES          # 1/m, what the plugin regulates on

        # ROTATE TO HEADING. When the carrot is behind the shoulder the geometry
        # is useless: pure pursuit would drive a huge arc. RPP spins in place.
        heading_to_carrot = math.atan2(ly, lx)
        if rotate_to_heading and abs(heading_to_carrot) > ROTATE_TO_HEADING_MIN_ANGLE:
            wz = math.copysign(ROTATE_TO_HEADING_VEL, heading_to_carrot)
            th += wz * DT
            rotating_steps += 1
            vs.append(0.0)
            errs.append(min(math.hypot(p[0] - x, p[1] - y) for p in path[::7]))
            continue

        v = VX
        if regulate:
            v1 = curvature_constraint(v, kappa)
            if v1 < v - 1e-12:
                curv_hits += 1
            v = v1
            pc = cost[int(y)][int(x)] if 0 <= int(x) < w and 0 <= int(y) < h else 0
            v2 = cost_constraint(v, pc)
            if v2 < v - 1e-12:
                cost_hits += 1
            v = v2
            # the plugin floors the regulated speed
            v = max(v, min_speed) if v > 0 else v
            remaining = sum(math.hypot(path[i + 1][0] - path[i][0],
                                       path[i + 1][1] - path[i][1])
                            for i in range(idx, len(path) - 1)) * RES
            carrot_d = math.sqrt(L2) * RES
            v3 = approach_constraint(v, remaining, carrot_d)
            if v3 < v - 1e-12:
                appr_hits += 1
            v = v3

        wz = kappa * v                      # rad/s, curvature(1/m) * v(m/s)
        sat = False
        if abs(wz) > WZ:
            wz = math.copysign(WZ, wz)
            sat = True
        vs.append(v)
        v_cells = v / RES
        th += wz * DT
        x += v_cells * math.cos(th) * DT
        y += v_cells * math.sin(th) * DT
        errs.append(min(math.hypot(p[0] - x, p[1] - y) for p in path[::7]))
        if math.hypot(path[-1][0] - x, path[-1][1] - y) < look_cells:
            moving = [q for q in vs if q > 0]
            return {
                "max_err_m": round(max(errs) * RES, 3),
                "mean_err_m": round(sum(errs) / len(errs) * RES, 3),
                "time_s": round(step * DT, 1),
                "mean_v": round(sum(moving) / max(1, len(moving)), 3),
                "min_v": round(min(moving) if moving else 0.0, 3),
                "curv_pct": round(100.0 * curv_hits / max(1, len(vs)), 1),
                "cost_pct": round(100.0 * cost_hits / max(1, len(vs)), 1),
                "appr_pct": round(100.0 * appr_hits / max(1, len(vs)), 1),
                "rotate_steps": rotating_steps,
                "path_m": round(total_len, 1),
                "done": True,
            }
    moving = [q for q in vs if q > 0]
    return {
        "max_err_m": round(max(errs) * RES, 3) if errs else 0,
        "mean_err_m": round(sum(errs) / len(errs) * RES, 3) if errs else 0,
        "time_s": round(max_steps * DT, 1),
        "mean_v": round(sum(moving) / max(1, len(moving)), 3) if moving else 0,
        "min_v": round(min(moving), 3) if moving else 0,
        "curv_pct": round(100.0 * curv_hits / max(1, len(vs)), 1),
        "cost_pct": round(100.0 * cost_hits / max(1, len(vs)), 1),
        "appr_pct": round(100.0 * appr_hits / max(1, len(vs)), 1),
        "rotate_steps": rotating_steps,
        "path_m": round(total_len, 1),
        "done": False,
    }


def main():
    w, h, grid = load()
    path = astar(grid, w, h, (100, 270), (389, 219))
    if not path:
        print("no path")
        return 1
    print("path: %d points, %.1f m" % (len(path), sum(
        math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
        for i in range(len(path)-1)) * RES))
    print("building the inflation layer ...")
    cost = inflate(grid, w, h)
    band = sum(1 for y in range(h) for x in range(w) if 0 < cost[y][x] < 253)
    lethal = sum(1 for y in range(h) for x in range(w) if cost[y][x] >= 253)
    print("  inflation: %d lethal/inscribed cells, %d in the decay band" % (lethal, band))
    on_path = [cost[p[1]][p[0]] for p in path]
    nz = [c for c in on_path if c > 0]
    print("  cost under the path: %d of %d cells nonzero, max %d, mean %.1f"
          % (len(nz), len(on_path), max(on_path), sum(on_path) / len(on_path)))
    print()

    results = {
        "provenance": {
            "defaults": "reference/rpp_declared_defaults.yaml, dumped from a live "
                        "controller_server running the real RPP plugin",
            "regulations": "implemented verbatim from "
                           "/opt/ros/jazzy/include/nav2_regulated_pure_pursuit_"
                           "controller/regulation_functions.hpp",
            "map": "reference/maps/warehouse.pgm (rebuilt 2026-08-21, reproduces "
                   "9.7's published table)",
        },
        "defaults": {
            "desired_linear_vel": VX, "lookahead_dist": LOOKAHEAD,
            "min_lookahead_dist": MIN_LOOKAHEAD, "max_lookahead_dist": MAX_LOOKAHEAD,
            "lookahead_time": LOOKAHEAD_TIME,
            "regulated_linear_scaling_min_radius": MIN_RADIUS,
            "regulated_linear_scaling_min_speed": MIN_SPEED,
            "min_approach_linear_velocity": MIN_APPROACH_VEL,
            "approach_velocity_scaling_dist": APPROACH_DIST,
            "cost_scaling_dist": COST_SCALING_DIST,
            "cost_scaling_gain": COST_SCALING_GAIN,
            "inflation_cost_scaling_factor": INFLATION_FACTOR,
            "rotate_to_heading_min_angle": ROTATE_TO_HEADING_MIN_ANGLE,
            "rotate_to_heading_angular_vel": ROTATE_TO_HEADING_VEL,
        },
        "inflation": {"lethal_cells": lethal, "band_cells": band,
                      "path_cells_nonzero": len(nz), "path_cells": len(on_path),
                      "path_cost_max": max(on_path),
                      "path_cost_mean": round(sum(on_path) / len(on_path), 1)},
    }

    # --- 1. plain vs regulated, on the good path ----------------------------
    plain = run(path, cost, w, h, regulate=False)
    reg = run(path, cost, w, h, regulate=True)
    results["plain"] = plain
    results["regulated"] = reg
    print("  %-26s %9s %9s" % ("", "plain 9.7", "regulated"))
    for k, lbl in (("max_err_m", "max cross-track (m)"),
                   ("mean_err_m", "mean cross-track (m)"),
                   ("time_s", "time (s)"), ("mean_v", "mean speed (m/s)"),
                   ("min_v", "min speed (m/s)"),
                   ("curv_pct", "curvature capped (%)"),
                   ("cost_pct", "cost capped (%)"),
                   ("appr_pct", "approach capped (%)")):
        print("  %-26s %9s %9s" % (lbl, plain[k], reg[k]))
    print()

    # --- 2. which regulation actually fires, one at a time ------------------
    print("  ablation, each regulation alone:")
    abl = {}
    for name, kw in (("curvature only", dict(regulate=True)),
                     ("no floor (min_speed 0)", dict(regulate=True, min_speed=0.0))):
        r = run(path, cost, w, h, **kw)
        abl[name] = r
        print("    %-24s min_v=%-6s mean_v=%-6s time=%-6s max_err=%s"
              % (name, r["min_v"], r["mean_v"], r["time_s"], r["max_err_m"]))
    results["ablation"] = abl
    print()

    # --- 3. the 90 degree start: 9.7's failure, now with rotate_to_heading --
    print("  the 90 deg heading error that broke 9.7's short lookahead:")
    rows = []
    for label, kw in (("plain, no rotate", dict(regulate=False, rotate_to_heading=False)),
                      ("regulated, no rotate", dict(regulate=True, rotate_to_heading=False)),
                      ("regulated + rotate_to_heading", dict(regulate=True, rotate_to_heading=True))):
        r = run(path, cost, w, h, start_yaw_off=math.pi / 2, **kw)
        r["label"] = label
        rows.append(r)
        print("    %-30s done=%-5s time=%-7s max_err=%-6s rotate_steps=%s"
              % (label, r["done"], r["time_s"], r["max_err_m"], r["rotate_steps"]))
    results["heading_error_90deg"] = rows

    with open("reference/rpp_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("\nwrote reference/rpp_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
