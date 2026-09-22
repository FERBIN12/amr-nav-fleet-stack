#!/usr/bin/env python3
"""DWB: sample the velocity space, roll each candidate out, score them.

Every parameter here was read out of the REAL plugin rather than remembered.
A controller_server was launched with plugin dwb_core::DWBLocalPlanner, driven to
the configure state, and `ros2 param dump` captured the declared defaults ->
reference/dwb_declared_defaults.yaml:

    sim_time              1.7      vx_samples        20
    linear_granularity    0.5      vtheta_samples    20
    angular_granularity   0.025    vy_samples         5
    discretize_by_time    false    forward_prune_distance 2.0
    every critic scale    1.0      forward_point_distance 0.325

Robot limits are OUR robot's, from nav2_params.yaml (measured):
vx_max 0.5 m/s, wz_max 1.9 rad/s, and the real footprint 0.80 x 0.58 m.

The critics implemented here are the four that decide the outcome in a corridor:
BaseObstacle, PathDist, GoalDist and PathAlign. Each returns a raw cost; the
score is the weighted sum, and LOWER is better.
"""
import heapq
import json
import math

RES = 0.05
VX_MAX = 0.5
WZ_MAX = 1.9
SIM_TIME = 1.7            # DWB default, measured
LIN_GRAN = 0.5            # DWB default: 0.5 m between rollout samples
VX_SAMPLES = 20           # DWB default
VTH_SAMPLES = 20          # DWB default
HALF_X = 0.40 / RES       # 8.0 cells
HALF_Y = 0.29 / RES       # 5.8 cells


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


def free(grid, w, h, x, y):
    xi, yi = int(round(x)), int(round(y))
    return 0 <= xi < w and 0 <= yi < h and grid[yi][xi] == 254


def footprint_free(grid, w, h, x, y, th):
    """The real 0.80 x 0.58 m footprint, 9 sample points like 9.4."""
    c, s = math.cos(th), math.sin(th)
    for ex in (-HALF_X, 0.0, HALF_X):
        for ey in (-HALF_Y, 0.0, HALF_Y):
            if not free(grid, w, h, x + ex * c - ey * s, y + ex * s + ey * c):
                return False
    return True


def rollout(x, y, th, v, wz):
    """DWB rolls out by DISTANCE, not by time: discretize_by_time is false and
    linear_granularity is 0.5 m. That is the detail people get wrong -- a slow
    candidate gets FEWER samples over the same sim_time, not more."""
    dist = abs(v) * SIM_TIME
    n = max(1, int(dist / LIN_GRAN))
    dt = SIM_TIME / n
    pts = [(x, y, th)]
    for _ in range(n):
        th += wz * dt
        x += (v / RES) * math.cos(th) * dt
        y += (v / RES) * math.sin(th) * dt
        pts.append((x, y, th))
    return pts


def nearest(path, x, y, stride=5):
    return min(math.hypot(p[0] - x, p[1] - y) for p in path[::stride])


def score(traj, grid, w, h, path, goal, weights):
    """The four critics that matter in a corridor. LOWER is better; a critic that
    cannot be satisfied returns None, which is how DWB rejects a candidate."""
    obs = 0.0
    for (x, y, th) in traj:
        if not footprint_free(grid, w, h, x, y, th):
            return None, None            # BaseObstacle VETO, not a penalty
        obs += 0.0
    end = traj[-1]
    pathd = nearest(path, end[0], end[1])
    goald = math.hypot(goal[0] - end[0], goal[1] - end[1])
    # PathAlign uses a point FORWARD of the robot, 0.325 m ahead (measured).
    fp = 0.325 / RES
    ax = end[0] + fp * math.cos(end[2])
    ay = end[1] + fp * math.sin(end[2])
    align = nearest(path, ax, ay)
    parts = {"obstacle": obs, "path_dist": pathd, "goal_dist": goald,
             "path_align": align}
    total = sum(weights[k] * v for k, v in parts.items())
    return total, parts


def sample_and_score(grid, w, h, path, goal, pose, weights):
    x, y, th = pose
    best = None
    kept = 0
    vetoed = 0
    rows = []
    for i in range(VX_SAMPLES):
        v = -0.35 + (VX_MAX - (-0.35)) * i / (VX_SAMPLES - 1)
        for j in range(VTH_SAMPLES):
            wz = -WZ_MAX + 2 * WZ_MAX * j / (VTH_SAMPLES - 1)
            if abs(v) < 1e-6:
                continue
            traj = rollout(x, y, th, v, wz)
            s, parts = score(traj, grid, w, h, path, goal, weights)
            if s is None:
                vetoed += 1
                continue
            kept += 1
            rows.append((s, v, wz, len(traj), parts))
            if best is None or s < best[0]:
                best = (s, v, wz, len(traj), parts)
    rows.sort(key=lambda r: r[0])
    return best, kept, vetoed, rows


def tunnelling_audit(grid, w, h, path):
    """THE HEADLINE MEASUREMENT.

    With the declared defaults and our robot, vx_max 0.5 m/s over sim_time 1.7 s
    covers 0.85 m, and linear_granularity 0.5 m divides that into exactly ONE
    interval. So the collision check examines the start pose and the end pose and
    nothing between -- a step of 0.850 m, which is LONGER than the robot's own
    0.80 m body. An obstacle in the middle of the arc is invisible.

    This counts how often that actually happens on a real path in our warehouse.
    """
    coarse_ok_fine_bad = 0
    tested = 0
    for pi in range(0, len(path) - 10, 7):
        px, py = path[pi]
        th = math.atan2(path[pi + 6][1] - py, path[pi + 6][0] - px)
        for j in range(VTH_SAMPLES):
            wz = -WZ_MAX + 2 * WZ_MAX * j / (VTH_SAMPLES - 1)
            v = VX_MAX
            tested += 1
            coarse = rollout(float(px), float(py), th, v, wz)
            n = 17                          # granularity 0.05 m
            dt = SIM_TIME / n
            x, y, t2 = float(px), float(py), th
            fine = [(x, y, t2)]
            for _ in range(n):
                t2 += wz * dt
                x += (v / RES) * math.cos(t2) * dt
                y += (v / RES) * math.sin(t2) * dt
                fine.append((x, y, t2))
            ok_c = all(footprint_free(grid, w, h, *q) for q in coarse)
            ok_f = all(footprint_free(grid, w, h, *q) for q in fine)
            if ok_c and not ok_f:
                coarse_ok_fine_bad += 1
    return coarse_ok_fine_bad, tested


def main():
    w, h, grid = load()
    start, goal = (100, 270), (389, 219)
    path = astar(grid, w, h, start, goal)
    print(f"path {len(path)} points")

    # the pose DWB is actually asked about: on the path, heading along it
    px, py = path[0]
    th = math.atan2(path[6][1] - py, path[6][0] - px)
    pose = (float(px), float(py), th)

    W_DEFAULT = {"obstacle": 1.0, "path_dist": 1.0, "goal_dist": 1.0,
                 "path_align": 1.0}
    out = {"declared_defaults": {
               "sim_time": SIM_TIME, "linear_granularity": LIN_GRAN,
               "vx_samples": VX_SAMPLES, "vtheta_samples": VTH_SAMPLES,
               "discretize_by_time": False, "forward_point_distance": 0.325,
               "source": "ros2 param dump of a configured controller_server"},
           "robot": {"vx_max": VX_MAX, "wz_max": WZ_MAX,
                     "footprint_m": [0.80, 0.58]},
           "path": {"points": len(path), "query": [list(start), list(goal)]}}

    best, kept, vetoed, rows = sample_and_score(grid, w, h, path, goal, pose,
                                                W_DEFAULT)
    print(f"\nGRID: {VX_SAMPLES} x {VTH_SAMPLES} = {VX_SAMPLES*VTH_SAMPLES} "
          f"candidates; kept {kept}, vetoed {vetoed}")
    print(f"best: v={best[1]:.3f} wz={best[2]:+.3f} score={best[0]:.2f} "
          f"samples={best[3]}")
    out["default_weights"] = {
        "candidates": VX_SAMPLES * VTH_SAMPLES, "kept": kept, "vetoed": vetoed,
        "best": {"v": round(best[1], 3), "wz": round(best[2], 3),
                 "score": round(best[0], 2), "rollout_samples": best[3],
                 "parts": {k: round(v, 2) for k, v in best[4].items()}},
        "top5": [{"v": round(r[1], 3), "wz": round(r[2], 3),
                  "score": round(r[0], 2)} for r in rows[:5]],
    }

    # HOW MANY SAMPLES DOES EACH CANDIDATE ACTUALLY GET? This is the
    # discretize_by_time trap, measured rather than asserted.
    print("\nrollout resolution vs commanded speed (sim_time 1.7 s):")
    res = []
    for v in (0.05, 0.10, 0.25, 0.50):
        n = len(rollout(*pose, v, 0.0))
        d = v * SIM_TIME
        print(f"  v={v:.2f} m/s -> {d:.2f} m covered, {n} samples")
        res.append({"v": v, "metres": round(d, 3), "samples": n})
    out["rollout_resolution"] = res

    # WEIGHT SWEEP: does the winner change, and by how much?
    print("\nweight sweep (which candidate wins):")
    sweep = []
    for name, wt in [
            ("all 1.0 (declared default)", W_DEFAULT),
            ("path_dist x32", {**W_DEFAULT, "path_dist": 32.0}),
            ("goal_dist x24", {**W_DEFAULT, "goal_dist": 24.0}),
            ("path_align x32", {**W_DEFAULT, "path_align": 32.0}),
            ("goal_dist only", {"obstacle": 1.0, "path_dist": 0.0,
                                "goal_dist": 24.0, "path_align": 0.0})]:
        b, k, vv, _ = sample_and_score(grid, w, h, path, goal, pose, wt)
        print(f"  {name:<28} v={b[1]:+.3f} wz={b[2]:+.3f} score={b[0]:.2f}")
        sweep.append({"weights": name, "v": round(b[1], 3),
                      "wz": round(b[2], 3), "score": round(b[0], 2)})
    out["weight_sweep"] = sweep

    bad, tested = tunnelling_audit(grid, w, h, path)
    print(f"\nTUNNELLING AUDIT at the declared granularity:")
    print(f"  step at vx_max = {VX_MAX*SIM_TIME:.3f} m, robot body = 0.80 m")
    print(f"  arcs accepted by the coarse check that actually collide: "
          f"{bad} of {tested} = {100.0*bad/tested:.1f}%")
    out["tunnelling"] = {
        "step_m": round(VX_MAX * SIM_TIME, 3), "body_m": 0.80,
        "false_accepts": bad, "tested": tested,
        "pct": round(100.0 * bad / tested, 1),
        "note": "coarse = declared linear_granularity 0.5 m (1 interval); "
                "fine = 0.05 m (17 intervals)"}
    out["granularity_table"] = [
        {"granularity_m": g,
         "intervals": max(1, int(VX_MAX * SIM_TIME / g)),
         "step_m": round(VX_MAX * SIM_TIME / max(1, int(VX_MAX * SIM_TIME / g)), 3)}
        for g in (0.5, 0.25, 0.10, 0.05)]

    json.dump(out, open("reference/dwb_results.json", "w"), indent=2)
    print("\n-> reference/dwb_results.json")


if __name__ == "__main__":
    main()
