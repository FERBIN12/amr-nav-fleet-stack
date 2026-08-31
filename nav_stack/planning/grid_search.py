#!/usr/bin/env python3
"""Run BFS, Dijkstra and A* on this project'S OWN saved map and report real counts.

Everything an earlier module-9.3 quotes comes from here rather than from a textbook, so
the numbers describe the warehouse the learner has been driving in all project.

The map: /tmp/maps/warehouse.pgm, 599 x 461 at 0.05 m/px, values
205 unknown / 254 free / 0 occupied (measured in 7.8).
"""
import heapq
import math
import sys
from collections import deque


def load(path="/tmp/maps/warehouse.pgm"):
    d = open(path, "rb").read()
    i = 0
    for _ in range(4):                       # magic, w, h, maxval
        while d[i:i + 1].isspace():
            i += 1
        while i < len(d) and not d[i:i + 1].isspace():
            i += 1
    i += 1
    hdr = d[:i].split()
    w, h = int(hdr[1]), int(hdr[2])
    body = d[i:]
    # row 0 of a pgm is the TOP; flip so y increases upward like the map frame
    grid = [[body[(h - 1 - y) * w + x] for x in range(w)] for y in range(h)]
    return w, h, grid


def passable(v):
    return v == 254                          # free only; 205 unknown is NOT free


NB4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
NB8 = NB4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))


def bfs(w, h, grid, start, goal):
    q = deque([start])
    seen = {start: None}
    expanded = 0
    while q:
        cur = q.popleft()
        expanded += 1
        if cur == goal:
            return expanded, path_len(seen, goal), True
        for dx, dy in NB4:
            n = (cur[0] + dx, cur[1] + dy)
            if 0 <= n[0] < w and 0 <= n[1] < h and n not in seen \
                    and passable(grid[n[1]][n[0]]):
                seen[n] = cur
                q.append(n)
    return expanded, 0, False


def dijkstra(w, h, grid, start, goal, nb=NB8):
    dist = {start: 0.0}
    prev = {start: None}
    pq = [(0.0, start)]
    expanded = 0
    done = set()
    while pq:
        d, cur = heapq.heappop(pq)
        if cur in done:
            continue
        done.add(cur)
        expanded += 1
        if cur == goal:
            return expanded, d, True
        for dx, dy in nb:
            n = (cur[0] + dx, cur[1] + dy)
            if not (0 <= n[0] < w and 0 <= n[1] < h):
                continue
            if not passable(grid[n[1]][n[0]]):
                continue
            step = math.hypot(dx, dy)
            nd = d + step
            if nd < dist.get(n, float("inf")):
                dist[n] = nd
                prev[n] = cur
                heapq.heappush(pq, (nd, n))
    return expanded, 0.0, False


def astar(w, h, grid, start, goal, weight=1.0, nb=NB8):
    def hcost(a):
        return math.hypot(a[0] - goal[0], a[1] - goal[1])
    g = {start: 0.0}
    pq = [(weight * hcost(start), 0.0, start)]
    expanded = 0
    done = set()
    while pq:
        _, d, cur = heapq.heappop(pq)
        if cur in done:
            continue
        done.add(cur)
        expanded += 1
        if cur == goal:
            return expanded, d, True
        for dx, dy in nb:
            n = (cur[0] + dx, cur[1] + dy)
            if not (0 <= n[0] < w and 0 <= n[1] < h):
                continue
            if not passable(grid[n[1]][n[0]]):
                continue
            nd = d + math.hypot(dx, dy)
            if nd < g.get(n, float("inf")):
                g[n] = nd
                heapq.heappush(pq, (nd + weight * hcost(n), nd, n))
    return expanded, 0.0, False


def path_len(prev, goal):
    n, c = goal, 0
    while prev.get(n) is not None:
        n = prev[n]
        c += 1
    return c


def main():
    w, h, grid = load()
    free = sum(1 for row in grid for v in row if v == 254)
    print("map %dx%d, %d free cells (%.1f%%)" % (w, h, free, 100 * free / (w * h)))
    # a start and goal both in genuinely free space, far apart
    start, goal = (450, 245), (150, 245)
    for name, pt in (("start", start), ("goal", goal)):
        print("  %s %s value=%d" % (name, pt, grid[pt[1]][pt[0]]))
    print()
    e, c, ok = bfs(w, h, grid, start, goal)
    print("BFS       (4-connected, unweighted): expanded %6d  path %4d cells  %s"
          % (e, c, "found" if ok else "FAILED"))
    e, d, ok = dijkstra(w, h, grid, start, goal)
    print("Dijkstra  (8-connected, euclidean) : expanded %6d  cost %7.1f  %s"
          % (e, d, "found" if ok else "FAILED"))
    e, d, ok = astar(w, h, grid, start, goal, 1.0)
    print("A*        (h = euclidean, w=1.0)   : expanded %6d  cost %7.1f  %s"
          % (e, d, "found" if ok else "FAILED"))
    e, d, ok = astar(w, h, grid, start, goal, 2.0)
    print("A* weighted (w=2.0)                : expanded %6d  cost %7.1f  %s"
          % (e, d, "found" if ok else "FAILED"))


if __name__ == "__main__":
    sys.exit(main())
