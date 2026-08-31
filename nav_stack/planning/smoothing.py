#!/usr/bin/env python3
"""Path smoothing measured on real planner output from this project.

Takes the actual A* path on our map and applies the two standard smoothers, then
measures what each one changed: length, total heading change, and whether the
result is still collision free. The last one is the part demos usually skip.
"""
import heapq
import math
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
    return w, h, [[body[(h - 1 - y) * w + x] for x in range(w)] for y in range(h)]


def free(grid, w, h, x, y):
    ix, iy = int(round(x)), int(round(y))
    return 0 <= ix < w and 0 <= iy < h and grid[iy][ix] == 254


def seg_free(grid, w, h, a, b, step=1.0):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(d / step))
    return all(free(grid, w, h, a[0] + (b[0] - a[0]) * i / n,
                    a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1))


NB8 = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def astar_path(grid, w, h, start, goal):
    def hc(a):
        return math.hypot(a[0] - goal[0], a[1] - goal[1])
    g = {start: 0.0}
    prev = {start: None}
    pq = [(hc(start), 0.0, start)]
    done = set()
    while pq:
        _, d, cur = heapq.heappop(pq)
        if cur in done:
            continue
        done.add(cur)
        if cur == goal:
            path = []
            k = cur
            while k is not None:
                path.append(k)
                k = prev[k]
            return path[::-1]
        for dx, dy in NB8:
            n = (cur[0] + dx, cur[1] + dy)
            if not (0 <= n[0] < w and 0 <= n[1] < h) or grid[n[1]][n[0]] != 254:
                continue
            nd = d + math.hypot(dx, dy)
            if nd < g.get(n, float("inf")):
                g[n] = nd
                prev[n] = cur
                heapq.heappush(pq, (nd + hc(n), nd, n))
    return []


def length(p):
    return sum(math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1])
               for i in range(len(p) - 1))


def heading_change(p):
    """Total absolute turning, in degrees. The number a passenger feels."""
    tot = 0.0
    for i in range(1, len(p) - 1):
        a = math.atan2(p[i][1] - p[i - 1][1], p[i][0] - p[i - 1][0])
        b = math.atan2(p[i + 1][1] - p[i][1], p[i + 1][0] - p[i][0])
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        tot += abs(d)
    return math.degrees(tot)


def shortcut(grid, w, h, p):
    """Greedy: from each point, jump to the furthest visible later point."""
    out = [p[0]]
    i = 0
    while i < len(p) - 1:
        j = len(p) - 1
        while j > i + 1 and not seg_free(grid, w, h, p[i], p[j]):
            j -= 1
        out.append(p[j])
        i = j
    return out


def gradient(grid, w, h, p, wd=0.5, ws=0.3, iters=200):
    """The classic two-term smoother: stay near the original, be smooth."""
    q = [list(pt) for pt in p]
    for _ in range(iters):
        for i in range(1, len(q) - 1):
            for c in (0, 1):
                old = q[i][c]
                q[i][c] += wd * (p[i][c] - q[i][c]) \
                    + ws * (q[i - 1][c] + q[i + 1][c] - 2 * q[i][c])
                if not free(grid, w, h, q[i][0], q[i][1]):
                    q[i][c] = old          # reject a step that leaves free space
    return [tuple(pt) for pt in q]


def main():
    w, h, grid = load()
    raw = astar_path(grid, w, h, (395, 145), (120, 320))
    if not raw:
        print("no path"); return 1
    print("raw A* path: %d points, length %.1f cells, turning %.0f deg"
          % (len(raw), length(raw), heading_change(raw)))
    sc = shortcut(grid, w, h, raw)
    print("shortcut   : %d points, length %.1f cells, turning %.0f deg, collision free %s"
          % (len(sc), length(sc), heading_change(sc),
             all(seg_free(grid, w, h, sc[i], sc[i + 1]) for i in range(len(sc) - 1))))
    gr = gradient(grid, w, h, raw)
    print("gradient   : %d points, length %.1f cells, turning %.0f deg, collision free %s"
          % (len(gr), length(gr), heading_change(gr),
             all(seg_free(grid, w, h, gr[i], gr[i + 1]) for i in range(len(gr) - 1))))
    # and the trap: smooth an already-shortcut path
    both = gradient(grid, w, h, sc)
    print("both       : %d points, length %.1f cells, turning %.0f deg, collision free %s"
          % (len(both), length(both), heading_change(both),
             all(seg_free(grid, w, h, both[i], both[i + 1]) for i in range(len(both) - 1))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
