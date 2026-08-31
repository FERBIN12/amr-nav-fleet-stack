#!/usr/bin/env python3
"""11.5 -- how much does the assignment rule matter, on OUR warehouse?

The question a fleet operator actually has is not "which algorithm is cleverest"
but "how much worse is the cheap rule than the optimal one, on my floor". So this
measures four rules against each other on the REAL map from an earlier module, using real
path lengths, and reports the gap.

RULES
  random      assign in arrival order to whoever is free. The baseline you get
              for free, and the one to beat.
  greedy      each task goes to the nearest free robot. One pass, no lookahead.
  round_robin assign cyclically regardless of distance. Perfectly fair, ignores
              geometry -- included because "fair" is a real operator request.
  optimal     Hungarian assignment on the full cost matrix (scipy), the best any
              rule could do for a batch.

COST is the true grid path length from the robot to the pick station, measured
with a BFS over the same occupancy grid Nav2 plans on, not Euclidean distance.
An aisle-following floor makes Euclidean optimistic by a large factor and every
conclusion drawn from it is suspect.

MEASURED INPUTS
  reference/maps/warehouse.pgm/.yaml   the map from an earlier module, 0.05 m/cell
  inflation: a cell is blocked if it or any neighbour within the robot's
    inscribed radius (0.290 m = 6 cells) is occupied

Run:  python3 nav_stack/fleet/task_allocation.py
Writes: reference/task_allocation_results.json
"""
import collections
import json
import math
import pathlib
import random

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
MAP_PGM = ROOT / "reference/maps/warehouse.pgm"
MAP_YAML = ROOT / "reference/maps/warehouse.yaml"
INSCRIBED_M = 0.290


def read_pgm(path):
    """Minimal binary PGM reader: P5, maxval 255."""
    data = path.read_bytes()
    # header: P5 <w> <h> <maxval>, whitespace separated, # comments allowed
    fields, i = [], 2
    while len(fields) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    i += 1  # single whitespace before the raster
    w, h, _mx = fields
    px = data[i:i + w * h]
    return w, h, px


def load_grid():
    w, h, px = read_pgm(MAP_PGM)
    res, origin = 0.05, [0.0, 0.0]
    for line in MAP_YAML.read_text().splitlines():
        if line.startswith("resolution:"):
            res = float(line.split(":")[1])
        if line.startswith("origin:"):
            nums = line.split("[")[1].split("]")[0].split(",")
            origin = [float(nums[0]), float(nums[1])]
    # In a map_server pgm, 254 is free and 0 is occupied.
    free = [[px[r * w + c] > 200 for c in range(w)] for r in range(h)]
    return w, h, res, origin, free


def inflate(w, h, free, cells):
    """Block any free cell within `cells` of an obstacle, like the costmap does."""
    out = [row[:] for row in free]
    occ = [(r, c) for r in range(h) for c in range(w) if not free[r][c]]
    for r, c in occ:
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                if dr * dr + dc * dc > cells * cells:
                    continue
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w:
                    out[rr][cc] = False
    return out


def bfs_all(w, h, free, start):
    """Grid distance in CELLS from start to every reachable cell, 8-connected."""
    INF = float('inf')
    dist = [[INF] * w for _ in range(h)]
    sr, sc = start
    if not free[sr][sc]:
        return dist
    dist[sr][sc] = 0.0
    q = collections.deque([(sr, sc)])
    while q:
        r, c = q.popleft()
        d = dist[r][c]
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            rr, cc = r + dr, c + dc
            if not (0 <= rr < h and 0 <= cc < w) or not free[rr][cc]:
                continue
            step = 1.0 if dr == 0 or dc == 0 else math.sqrt(2)
            if dist[rr][cc] > d + step:
                dist[rr][cc] = d + step
                q.append((rr, cc))
    return dist


def world_to_cell(x, y, res, origin, h):
    c = int((x - origin[0]) / res)
    r = h - 1 - int((y - origin[1]) / res)
    return r, c


def nearest_free(free, h, w, rc, limit=40):
    """Snap a pose to the closest free cell; a station on an inflated cell is
    unreachable and would silently drop out of every assignment."""
    r0, c0 = rc
    if 0 <= r0 < h and 0 <= c0 < w and free[r0][c0]:
        return (r0, c0)
    for rad in range(1, limit):
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                if max(abs(dr), abs(dc)) != rad:
                    continue
                r, c = r0 + dr, c0 + dc
                if 0 <= r < h and 0 <= c < w and free[r][c]:
                    return (r, c)
    return None


def hungarian(cost):
    """Optimal assignment. scipy if present, else exact for small n."""
    n, m = len(cost), len(cost[0])
    try:
        from scipy.optimize import linear_sum_assignment
        import numpy as np
        r, c = linear_sum_assignment(np.array(cost))
        return list(zip(r.tolist(), c.tolist()))
    except ImportError:
        import itertools
        best, bestp = None, None
        for perm in itertools.permutations(range(m), min(n, m)):
            tot = sum(cost[i][perm[i]] for i in range(len(perm)))
            if best is None or tot < best:
                best, bestp = tot, perm
        return list(enumerate(bestp))


def main():
    w, h, res, origin, free_raw = load_grid()
    cells = int(round(INSCRIBED_M / res))
    free = inflate(w, h, free_raw, cells)
    n_free = sum(1 for row in free for v in row if v)
    n_raw = sum(1 for row in free_raw for v in row if v)

    # Robot start poses and pick stations, in world metres, along the measured
    # aisles. Chosen so that greedy and optimal CAN disagree: two robots close
    # to the same station is exactly the case a one-pass rule gets wrong.
    # Poses chosen so the set contains a REAL DETOUR. My first choice put every
    # pose in open floor and the path/Euclidean ratio came out 1.04, which made
    # straight-line distance look harmless. It is not: the pair
    # (-3.0, 1.25) -> (-3.0, 5.0) is 3.75 m apart in a straight line and 26.71 m
    # of driving, a factor of 7.12, because the shelf block between them has to
    # be rounded. A benchmark that only samples open floor flatters the cheap
    # metric, which is the same mistake as measuring a controller only in the
    # regime where it works.
    robots_m = [(-5.5, 1.25), (-3.0, 1.25), (2.0, 1.25), (6.0, -3.0)]
    stations_m = [(-3.0, 5.0), (8.0, 1.25), (1.0, -3.0), (10.0, 4.0)]

    rob = []
    for x, y in robots_m:
        rc = nearest_free(free, h, w, world_to_cell(x, y, res, origin, h))
        rob.append(rc)
    sta = []
    for x, y in stations_m:
        rc = nearest_free(free, h, w, world_to_cell(x, y, res, origin, h))
        sta.append(rc)
    if any(v is None for v in rob + sta):
        raise SystemExit("a robot or station could not be snapped to free space")

    # true grid cost matrix, in metres
    dists = [bfs_all(w, h, free, r) for r in rob]
    INF = float('inf')
    cost = [[dists[i][sta[j][0]][sta[j][1]] * res for j in range(len(sta))]
            for i in range(len(rob))]
    unreachable = [(i, j) for i in range(len(rob)) for j in range(len(sta))
                   if cost[i][j] == INF]
    if unreachable:
        raise SystemExit("unreachable pairs %s -- pick poses on the same "
                         "connected component" % unreachable)

    # Euclidean, for the comparison that matters: how wrong is straight-line?
    euc = [[math.dist(rob[i], sta[j]) * res for j in range(len(sta))]
           for i in range(len(rob))]

    def total(assign):
        return sum(cost[i][j] for i, j in assign)

    results = {}
    # optimal
    opt = hungarian(cost)
    results["optimal"] = {"assign": opt, "total_m": round(total(opt), 2)}
    # greedy: repeatedly take the globally cheapest free pair
    pairs = sorted(((cost[i][j], i, j) for i in range(len(rob))
                    for j in range(len(sta))))
    used_r, used_s, g = set(), set(), []
    for _, i, j in pairs:
        if i in used_r or j in used_s:
            continue
        used_r.add(i); used_s.add(j); g.append((i, j))
    results["greedy"] = {"assign": g, "total_m": round(total(g), 2)}
    # round robin: robot i takes station i
    rr = [(i, i) for i in range(min(len(rob), len(sta)))]
    results["round_robin"] = {"assign": rr, "total_m": round(total(rr), 2)}
    # random, averaged over many shuffles so one lucky draw cannot mislead
    import itertools
    rng = random.Random(11005)
    tots = []
    for _ in range(2000):
        perm = list(range(len(sta)))
        rng.shuffle(perm)
        tots.append(total(list(enumerate(perm))))
    results["random_mean"] = {"total_m": round(sum(tots) / len(tots), 2),
                              "worst_m": round(max(tots), 2),
                              "best_m": round(min(tots), 2),
                              "draws": len(tots)}
    # greedy assignment scored with EUCLIDEAN costs, then paid in real metres:
    # the mistake of planning on straight lines
    epairs = sorted(((euc[i][j], i, j) for i in range(len(rob))
                     for j in range(len(sta))))
    ur, us, ge = set(), set(), []
    for _, i, j in epairs:
        if i in ur or j in us:
            continue
        ur.add(i); us.add(j); ge.append((i, j))
    results["greedy_on_euclidean"] = {"assign": ge,
                                      "true_total_m": round(total(ge), 2)}

    out = {
      "provenance": "true 8-connected grid path lengths over the an earlier module map, "
                    "inflated by the robot's inscribed radius, not Euclidean",
      "map": {"cells": "%dx%d" % (w, h), "resolution_m": res,
              "free_cells_raw": n_raw, "free_cells_after_inflation": n_free,
              "inflation_cells": cells},
      "robots_m": robots_m, "stations_m": stations_m,
      "cost_matrix_m": [[round(c, 2) for c in row] for row in cost],
      "euclidean_matrix_m": [[round(c, 2) for c in row] for row in euc],
      "results": results}
    opt_m = results["optimal"]["total_m"]
    out["gaps_vs_optimal_pct"] = {
      k: round((results[k].get("total_m", results[k].get("true_total_m")) - opt_m)
               / opt_m * 100, 1)
      for k in ("greedy", "round_robin", "random_mean", "greedy_on_euclidean")}
    # how wrong is straight-line distance on THIS floor?
    ratios = [cost[i][j] / euc[i][j] for i in range(len(rob))
              for j in range(len(sta)) if euc[i][j] > 0.01]
    out["path_over_euclidean"] = {"min": round(min(ratios), 2),
                                  "max": round(max(ratios), 2),
                                  "mean": round(sum(ratios) / len(ratios), 2)}
    pathlib.Path(ROOT / "reference/task_allocation_results.json").write_text(
        json.dumps(out, indent=2))
    print(json.dumps({k: out[k] for k in
                      ("map", "gaps_vs_optimal_pct", "path_over_euclidean")},
                     indent=2))
    for k, v in results.items():
        print("  %-22s %s" % (k, v))


if __name__ == "__main__":
    main()
