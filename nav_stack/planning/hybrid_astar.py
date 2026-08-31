#!/usr/bin/env python3
"""Hybrid A* on this project's own map, and the comparison that makes 9.4 honest.

A grid A* plans for a POINT: it may turn 90 degrees between cells for free. A
car-like robot cannot. Hybrid A* searches (x, y, theta) instead of (x, y) and
expands by driving short arcs at a bounded steering angle, so every edge in the
search is a motion the vehicle can actually execute.

Our AMR is differential drive and CAN spin, so this module is explicitly about
what changes when it cannot -- which is the case for a tug, a forklift or a
delivery pod. The measured contrast is the point.
"""
import heapq
import math
import sys

RES = 0.05                    # m per cell, from warehouse.yaml
THETA_BINS = 24               # 15 degrees per bin


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
    if ix < 0 or iy < 0 or ix >= w or iy >= h:
        return False
    return grid[iy][ix] == 254


def footprint_free(grid, w, h, x, y, th, half_l=8.0, half_w=5.8):
    """Half-extents in CELLS: 0.40 m and 0.29 m at 0.05 m/cell."""
    c, s = math.cos(th), math.sin(th)
    for dl in (-half_l, 0.0, half_l):
        for dw in (-half_w, 0.0, half_w):
            if not free(grid, w, h, x + dl * c - dw * s, y + dl * s + dw * c):
                return False
    return True


def hybrid(grid, w, h, start, goal, max_steer=0.35, step=6.0, goal_tol=10.0):
    """start/goal are (x, y, theta) in cells and radians."""
    def key(n):
        return (int(n[0]), int(n[1]), int(n[2] / (2 * math.pi) * THETA_BINS) % THETA_BINS)

    def hcost(n):
        return math.hypot(goal[0] - n[0], goal[1] - n[1])

    steers = (-max_steer, -max_steer / 2, 0.0, max_steer / 2, max_steer)
    start_k = key(start)
    g = {start_k: 0.0}
    pq = [(hcost(start), 0.0, start)]
    seen = set()
    expanded = 0
    while pq:
        _, d, cur = heapq.heappop(pq)
        k = key(cur)
        if k in seen:
            continue
        seen.add(k)
        expanded += 1
        if math.hypot(goal[0] - cur[0], goal[1] - cur[1]) < goal_tol:
            return expanded, d, True
        if expanded > 60000:
            return expanded, 0.0, False
        for st in steers:
            th = cur[2] + st
            nx = cur[0] + step * math.cos(th)
            ny = cur[1] + step * math.sin(th)
            if not footprint_free(grid, w, h, nx, ny, th):
                continue
            n = (nx, ny, th)
            nk = key(n)
            # penalise steering so straight lines are preferred
            nd = d + step + abs(st) * 12.0
            if nd < g.get(nk, float("inf")):
                g[nk] = nd
                heapq.heappush(pq, (nd + hcost(n), nd, n))
    return expanded, 0.0, False


def main():
    w, h, grid = load()
    # a start and goal in open space, needing a turn between them
    cases = [
        ((450, 245, math.pi), (150, 245, math.pi), "straight down the corridor"),
        ((450, 245, 0.0), (150, 245, math.pi), "same, but starting FACING AWAY"),
    ]
    for start, goal, label in cases:
        e, d, ok = hybrid(grid, w, h, start, goal)
        print("  %-34s expanded %6d  cost %7.1f  %s"
              % (label, e, d, "found" if ok else "NOT FOUND"))
    print()
    for steer in (0.15, 0.25, 0.35, 0.5):
        e, d, ok = hybrid(grid, w, h, (450, 245, math.pi), (150, 245, math.pi),
                          max_steer=steer)
        print("  max_steer %.2f rad: expanded %6d  cost %7.1f  %s"
              % (steer, e, d, "found" if ok else "NOT FOUND"))


if __name__ == "__main__":
    sys.exit(main())
