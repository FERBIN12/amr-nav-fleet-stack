#!/usr/bin/env python3
"""RRT and RRT* on this project's own map, with the comparison 9.5 needs.

Grid search is complete and optimal but scales with the number of cells. Sampling
planners scale with the number of samples instead, which is a different trade and
the reason they win in high dimensions. On a 2D warehouse they are usually the
WRONG choice, and the measurements here say so plainly rather than selling them.
"""
import math
import random
import sys


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
    grid = [[body[(h - 1 - y) * w + x] for x in range(w)] for y in range(h)]
    return w, h, grid


def free(grid, w, h, x, y):
    ix, iy = int(x), int(y)
    return 0 <= ix < w and 0 <= iy < h and grid[iy][ix] == 254


def seg_free(grid, w, h, a, b, step=2.0):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(d / step))
    for i in range(n + 1):
        t = i / n
        if not free(grid, w, h, a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t):
            return False
    return True


def rrt(grid, w, h, start, goal, n=6000, step=18.0, goal_tol=14.0, seed=1):
    rng = random.Random(seed)
    nodes = [start]
    parent = {0: None}
    for it in range(n):
        # 5% goal bias, which is standard and worth naming in the module
        if rng.random() < 0.05:
            s = goal
        else:
            s = (rng.uniform(0, w), rng.uniform(0, h))
        # nearest existing node, linear scan: this is the cost that bites
        bi, bd = 0, float("inf")
        for i, p in enumerate(nodes):
            dd = (p[0] - s[0]) ** 2 + (p[1] - s[1]) ** 2
            if dd < bd:
                bd, bi = dd, i
        near = nodes[bi]
        d = math.hypot(s[0] - near[0], s[1] - near[1])
        if d < 1e-9:
            continue
        t = min(1.0, step / d)
        new = (near[0] + (s[0] - near[0]) * t, near[1] + (s[1] - near[1]) * t)
        if not free(grid, w, h, *new) or not seg_free(grid, w, h, near, new):
            continue
        nodes.append(new)
        parent[len(nodes) - 1] = bi
        if math.hypot(new[0] - goal[0], new[1] - goal[1]) < goal_tol:
            # walk the tree back for the path cost
            cost, k = 0.0, len(nodes) - 1
            while parent[k] is not None:
                p = nodes[parent[k]]
                cost += math.hypot(nodes[k][0] - p[0], nodes[k][1] - p[1])
                k = parent[k]
            return len(nodes), cost, it + 1, True
    return len(nodes), 0.0, n, False


def rrt_star(grid, w, h, start, goal, n=6000, step=18.0, radius=42.0,
             goal_tol=14.0, seed=1):
    rng = random.Random(seed)
    nodes = [start]
    parent = {0: None}
    cost = {0: 0.0}
    best_goal, best_cost = None, float("inf")
    for it in range(n):
        if rng.random() < 0.05:
            s = goal
        else:
            s = (rng.uniform(0, w), rng.uniform(0, h))
        bi, bd = 0, float("inf")
        for i, p in enumerate(nodes):
            dd = (p[0] - s[0]) ** 2 + (p[1] - s[1]) ** 2
            if dd < bd:
                bd, bi = dd, i
        near = nodes[bi]
        d = math.hypot(s[0] - near[0], s[1] - near[1])
        if d < 1e-9:
            continue
        t = min(1.0, step / d)
        new = (near[0] + (s[0] - near[0]) * t, near[1] + (s[1] - near[1]) * t)
        if not free(grid, w, h, *new) or not seg_free(grid, w, h, near, new):
            continue
        # CHOOSE PARENT: the cheapest reachable neighbour, not just the nearest
        cand = [i for i, p in enumerate(nodes)
                if math.hypot(p[0] - new[0], p[1] - new[1]) <= radius]
        bp, bc = bi, cost[bi] + math.hypot(new[0] - near[0], new[1] - near[1])
        for i in cand:
            c = cost[i] + math.hypot(nodes[i][0] - new[0], nodes[i][1] - new[1])
            if c < bc and seg_free(grid, w, h, nodes[i], new):
                bp, bc = i, c
        nodes.append(new)
        ni = len(nodes) - 1
        parent[ni] = bp
        cost[ni] = bc
        # REWIRE: can existing neighbours reach the goal more cheaply through new?
        for i in cand:
            c = bc + math.hypot(nodes[i][0] - new[0], nodes[i][1] - new[1])
            if c < cost[i] and seg_free(grid, w, h, new, nodes[i]):
                parent[i] = ni
                cost[i] = c
        if math.hypot(new[0] - goal[0], new[1] - goal[1]) < goal_tol:
            if bc < best_cost:
                best_cost, best_goal = bc, ni
    return len(nodes), (best_cost if best_goal is not None else 0.0), n, best_goal is not None


def main():
    w, h, grid = load()
    start, goal = (450.0, 245.0), (150.0, 245.0)
    print("query %s -> %s   (grid A*: 301 expansions, cost 300.0)" % (start, goal))
    print()
    for seed in (1, 2, 3, 4, 5):
        nn, c, it, ok = rrt(grid, w, h, start, goal, seed=seed)
        print("  RRT   seed %d: %5d nodes, %5d iters, cost %7.1f  %s"
              % (seed, nn, it, c, "found" if ok else "NOT FOUND"))
    print()
    for seed in (1, 2, 3):
        nn, c, it, ok = rrt_star(grid, w, h, start, goal, n=1500, seed=seed)
        print("  RRT*  seed %d: %5d nodes, %5d iters, cost %7.1f  %s"
              % (seed, nn, it, c, "found" if ok else "NOT FOUND"))


if __name__ == "__main__":
    sys.exit(main())
