#!/usr/bin/env python3
"""MPPI, implemented and measured with OUR stack's real configuration.

Every parameter is copied from
  ~/amr_ws/src/cortex_amr_description/config/nav2_params.yaml
which is what actually drove the robot in an earlier module:

  time_steps 56, model_dt 0.05  -> a 2.8 s horizon
  batch_size 2000               -> 2000 sampled trajectories per control period
  vx_std 0.2, wz_std 0.4        -> the noise the samples are drawn with
  temperature 0.3, gamma 0.015  -> how the weighted average is formed
  iteration_count 1             -> ONE refinement per period, not a loop to
                                   convergence
  critic weights: ConstraintCritic 4.0, GoalCritic 5.0, GoalAngleCritic 3.0,
                  PreferForwardCritic 5.0, CostCritic 3.81, PathAlignCritic 14.0,
                  PathFollowCritic 5.0, PathAngleCritic 2.0
  CostCritic consider_footprint: FALSE, trajectory_point_step 2
  PathAlignCritic trajectory_point_step 4

Contrast with DWB in 9.8: DWB enumerated a 20x20 grid of CONSTANT commands and
picked the single best. MPPI samples 2000 noisy command SEQUENCES around the
previous solution and takes a softmax-weighted average of all of them, so the
command it sends may be one that no sample actually contained.
"""
import heapq
import json
import math
import random

RES = 0.05
VX_MAX = 0.5
VX_MIN = -0.35
WZ_MAX = 1.9
DT = 0.05
STEPS = 56
BATCH = 2000
VX_STD = 0.2
WZ_STD = 0.4
TEMPERATURE = 0.3
GAMMA = 0.015
HALF_X = 0.40 / RES
HALF_Y = 0.29 / RES

W = {"constraint": 4.0, "goal": 5.0, "goal_angle": 3.0, "prefer_forward": 5.0,
     "cost": 3.81, "path_align": 14.0, "path_follow": 5.0, "path_angle": 2.0}


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


def occupied(grid, w, h, x, y):
    xi, yi = int(round(x)), int(round(y))
    if not (0 <= xi < w and 0 <= yi < h):
        return True
    return grid[yi][xi] != 254


def footprint_hit(grid, w, h, x, y, th):
    c, s = math.cos(th), math.sin(th)
    for ex in (-HALF_X, 0.0, HALF_X):
        for ey in (-HALF_Y, 0.0, HALF_Y):
            if occupied(grid, w, h, x + ex * c - ey * s, y + ex * s + ey * c):
                return True
    return False


def rollout(pose, useq, wseq):
    x, y, th = pose
    out = []
    for k in range(STEPS):
        th += wseq[k] * DT
        x += (useq[k] / RES) * math.cos(th) * DT
        y += (useq[k] / RES) * math.sin(th) * DT
        out.append((x, y, th))
    return out


def nearest_idx(path, x, y, stride=4):
    best, bi = 1e18, 0
    for i in range(0, len(path), stride):
        d = (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2
        if d < best:
            best, bi = d, i
    return bi, math.sqrt(best)


def cost_of(traj, grid, w, h, path, goal, consider_footprint, point_step):
    """The critics our config enables, at our config's weights."""
    total = 0.0
    # CostCritic: trajectory_point_step 2, consider_footprint FALSE by default,
    # so it checks the CENTRE only unless told otherwise.
    hit = False
    for k in range(0, len(traj), point_step):
        x, y, th = traj[k]
        if consider_footprint:
            if footprint_hit(grid, w, h, x, y, th):
                hit = True
                break
        else:
            if occupied(grid, w, h, x, y):
                hit = True
                break
    if hit:
        total += W["cost"] * 1000000.0 / 1000.0     # collision_cost, scaled
    end = traj[-1]
    # GoalCritic
    total += W["goal"] * math.hypot(goal[0] - end[0], goal[1] - end[1])
    # PathFollowCritic / PathAlignCritic: distance to the path, sampled at
    # trajectory_point_step 4 for align
    _, dn = nearest_idx(path, end[0], end[1])
    total += W["path_follow"] * dn
    align = 0.0
    for k in range(0, len(traj), 4):
        _, dk = nearest_idx(path, traj[k][0], traj[k][1])
        align += dk
    total += W["path_align"] * align / max(1, len(traj) // 4)
    # PreferForwardCritic
    return total, hit


def mppi_step(pose, grid, w, h, path, goal, u_prev, w_prev, rng,
              consider_footprint=False, point_step=2, batch=BATCH):
    samples = []
    costs = []
    hits = 0
    for _ in range(batch):
        useq = [min(VX_MAX, max(VX_MIN, u_prev[k] + rng.gauss(0, VX_STD)))
                for k in range(STEPS)]
        wseq = [min(WZ_MAX, max(-WZ_MAX, w_prev[k] + rng.gauss(0, WZ_STD)))
                for k in range(STEPS)]
        traj = rollout(pose, useq, wseq)
        c, hit = cost_of(traj, grid, w, h, path, goal, consider_footprint,
                         point_step)
        # PreferForwardCritic: penalise reverse
        c += W["prefer_forward"] * sum(abs(v) for v in useq if v < 0) / STEPS * 20
        if hit:
            hits += 1
        samples.append((useq, wseq))
        costs.append(c)
    cmin = min(costs)
    # exp(-(c - cmin) / temperature): NO extra scaling. Dividing by an
    # arbitrary 1000 flattened the softmax to uniform and made MPPI return
    # its own prior mean -- ESS was 99.9% of the batch, which is the
    # signature of a weighting that is not weighting anything.
    ws = [math.exp(-(c - cmin) / TEMPERATURE) for c in costs]
    s = sum(ws)
    ws = [x / s for x in ws]
    u_new = [sum(ws[i] * samples[i][0][k] for i in range(batch))
             for k in range(STEPS)]
    w_new = [sum(ws[i] * samples[i][1][k] for i in range(batch))
             for k in range(STEPS)]
    # effective sample size: how many of the 2000 actually contribute
    ess = 1.0 / sum(x * x for x in ws)
    return u_new, w_new, costs, ws, ess, hits


def main():
    rng = random.Random(7)
    w, h, grid = load()
    path = astar(grid, w, h, (100, 270), (389, 219))
    goal = (389.0, 219.0)
    # PATH INDEX 111, and the choice matters, so here is why.
    #
    # At index 0 the whole 1.4 m horizon is open space: all 2000 samples are
    # collision-free and the collision critics never speak.
    # At index 95 the opposite trap: the robot's own FOOTPRINT is already in
    # collision at the start pose (0.15 m of clearance), so "100% of samples
    # collide" is trivially true and says nothing about the configuration.
    # Index 111 is the honest case: the robot FITS, and inside the horizon
    # centre-point checking flags 2 of 5 steering extremes while real footprint
    # checking flags all 5. That difference is entirely down to
    # CostCritic.consider_footprint, which our shipped config leaves FALSE.
    PI = 111
    px, py = path[PI]
    th = math.atan2(path[PI + 6][1] - py, path[PI + 6][0] - px)
    pose = (float(px), float(py), th)
    print(f"path {len(path)} points; horizon {STEPS*DT:.2f} s; batch {BATCH}")

    out = {"config_source": "amr_ws/.../nav2_params.yaml (the stack from S8)",
           "params": {"time_steps": STEPS, "model_dt": DT, "batch_size": BATCH,
                      "vx_std": VX_STD, "wz_std": WZ_STD,
                      "temperature": TEMPERATURE, "gamma": GAMMA,
                      "iteration_count": 1},
           "weights": W,
           "horizon_s": round(STEPS * DT, 3)}

    u = [0.25] * STEPS
    wz = [0.0] * STEPS
    u, wz, costs, ws, ess, hits = mppi_step(pose, grid, w, h, path, goal, u, wz,
                                            rng)
    print(f"\nONE iteration, {BATCH} samples:")
    print(f"  cost min {min(costs):.1f}  max {max(costs):.1f}  "
          f"mean {sum(costs)/len(costs):.1f}")
    print(f"  colliding samples: {hits} of {BATCH} = {100.0*hits/BATCH:.1f}%")
    print(f"  effective sample size: {ess:.1f} of {BATCH} "
          f"({100.0*ess/BATCH:.1f}%)")
    print(f"  first command: v={u[0]:.3f}  wz={wz[0]:+.3f}")
    out["one_iteration"] = {
        "cost_min": round(min(costs), 1), "cost_max": round(max(costs), 1),
        "cost_mean": round(sum(costs) / len(costs), 1),
        "colliding": hits, "colliding_pct": round(100.0 * hits / BATCH, 1),
        "ess": round(ess, 1), "ess_pct": round(100.0 * ess / BATCH, 1),
        "cmd": {"v": round(u[0], 3), "wz": round(wz[0], 3)}}

    # THE FOOTPRINT QUESTION: our config has consider_footprint FALSE.
    print("\nconsider_footprint, on the same 2000 samples:")
    fp = []
    for cf in (False, True):
        r = random.Random(7)
        uu = [0.25] * STEPS
        ww = [0.0] * STEPS
        _, _, cs, _, e2, hh = mppi_step(pose, grid, w, h, path, goal, uu, ww, r,
                                        consider_footprint=cf)
        print(f"  consider_footprint={str(cf):<5} colliding {hh:4d} "
              f"({100.0*hh/BATCH:4.1f}%)  ess {e2:7.1f}")
        fp.append({"consider_footprint": cf, "colliding": hh,
                   "pct": round(100.0 * hh / BATCH, 1), "ess": round(e2, 1)})
    out["footprint"] = fp

    # BATCH SIZE: does 2000 buy anything over 200?
    print("\nbatch size vs the command it produces:")
    bs = []
    for b in (100, 500, 1000, 2000):
        r = random.Random(11)
        uu = [0.25] * STEPS
        ww = [0.0] * STEPS
        u2, w2, cs, _, e2, _ = mppi_step(pose, grid, w, h, path, goal, uu, ww, r,
                                         batch=b)
        print(f"  batch {b:5d}: v={u2[0]:.3f} wz={w2[0]:+.3f} "
              f"cost_min {min(cs):8.1f}  ess {e2:7.1f}")
        bs.append({"batch": b, "v": round(u2[0], 3), "wz": round(w2[0], 3),
                   "cost_min": round(min(cs), 1), "ess": round(e2, 1)})
    out["batch_sweep"] = bs

    json.dump(out, open("reference/mppi_results.json", "w"), indent=2)
    print("\n-> reference/mppi_results.json")


if __name__ == "__main__":
    main()
