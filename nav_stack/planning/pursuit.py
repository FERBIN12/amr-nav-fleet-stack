#!/usr/bin/env python3
"""Pure pursuit, derived and swept on a real path from this project.

The geometry: pick the point on the path a lookahead distance L ahead, and drive
the arc that passes through it. Curvature is 2y/L^2 where y is the lateral offset
of that point in the robot frame. That is the whole controller.

Simulated on a differential-drive unicycle at our robot's real limits so the
numbers describe THIS machine: vx_max 0.5 m/s, wz_max 1.9 rad/s (from
nav2_params.yaml, measured).
"""
import heapq
import math
import sys

RES = 0.05
VX = 0.5
WZ = 1.9
DT = 0.05


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


def pursue(path, look, max_steps=20000):
    """Drive the path. Returns (cross-track stats, path completion, cmd stats)."""
    if not path:
        return None
    x, y = float(path[0][0]), float(path[0][1])
    # start pointing along the first segment
    th = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
    idx = 0
    errs = []
    wz_used = []
    sat = 0
    for step in range(max_steps):
        # advance the target to the first point at least `look` cells ahead
        while idx < len(path) - 1 and \
                math.hypot(path[idx][0] - x, path[idx][1] - y) < look:
            idx += 1
        tgt = path[idx]
        dx, dy = tgt[0] - x, tgt[1] - y
        # into the robot frame
        c, s = math.cos(-th), math.sin(-th)
        lx = dx * c - dy * s
        ly = dx * s + dy * c
        L2 = lx * lx + ly * ly
        if L2 < 1e-9:
            break
        kappa = 2.0 * ly / L2               # the pure pursuit curvature
        v_cells = VX / RES                  # m/s -> cells/s
        wz = kappa * v_cells
        if abs(wz) > WZ:
            wz = math.copysign(WZ, wz)
            sat += 1
        wz_used.append(abs(wz))
        th += wz * DT
        x += v_cells * math.cos(th) * DT
        y += v_cells * math.sin(th) * DT
        # cross-track: distance to the nearest point on the path
        best = min(math.hypot(p[0] - x, p[1] - y) for p in path[::7])
        errs.append(best)
        if math.hypot(path[-1][0] - x, path[-1][1] - y) < look:
            return (max(errs), sum(errs) / len(errs), step * DT,
                    100.0 * sat / max(1, len(wz_used)), True)
    return (max(errs) if errs else 0, sum(errs) / len(errs) if errs else 0,
            max_steps * DT, 100.0 * sat / max(1, len(wz_used)), False)


def main():
    w, h, grid = load()
    path = astar(grid, w, h, (100, 270), (389, 219))
    if not path:
        print("no path")
        return 1
    print("path: %d points, %.1f cells (%.1f m)"
          % (len(path), sum(math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
                            for i in range(len(path)-1)) * 1,
             sum(math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
                 for i in range(len(path)-1)) * RES))
    print("robot: vx %.2f m/s, wz %.2f rad/s, dt %.2f s" % (VX, WZ, DT))
    print()
    print("  look(cells)  look(m)   max err(m)  mean err(m)   time(s)  wz sat%%  done")
    for look in (4, 8, 12, 20, 30, 45, 60):
        r = pursue(path, float(look))
        print("  %8d    %6.2f   %9.3f   %10.3f  %7.1f  %6.1f   %s"
              % (look, look * RES, r[0] * RES, r[1] * RES, r[2], r[3],
                 "yes" if r[4] else "NO"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
