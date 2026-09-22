#!/usr/bin/env python3
"""11.6 -- centralised or decentralised, and what does each cost on OUR floor?

The architecture question a fleet actually forces is not philosophical. It is:
who decides who enters an aisle, and what does that decision cost you in
throughput, in latency, and in what happens when the decider dies.

Three architectures, all simulated over the MEASURED warehouse geometry:

  none          every robot runs its own Nav2 and treats the others as obstacles.
                This is what 11.4 measured: they meet, they both stop.
  central       one traffic manager owns the aisles. A robot requests an aisle,
                waits for a grant, drives, releases. One decision point.
  decentral     robots negotiate pairwise on encounter: lower ID yields. No
                central node, but a decision per encounter and no global view.

WHAT IS MEASURED, per architecture, over the same task stream:
  throughput      tasks completed per minute
  mean wait       seconds a robot spends blocked or yielding
  worst wait      the tail, which is what an operator complains about
  deadlocks       encounters that never resolve
  single point    what fraction of tasks stall if the decider dies mid-run

MEASURED GEOMETRY, all from earlier testing:
  aisle 1.70 m, free passing above 1.57 m (11.1)
  footprint 0.800 x 0.580 m, inscribed 0.290 m (8.10)
  vx_max 0.5 m/s (an earlier module)
  a head-on pass needs 0.580 m of lateral separation and both robots must begin
    diverting at >= 0.95 m of edge gap (11.4)

Run:  python3 nav_stack/fleet/fleet_architecture.py
Writes: reference/fleet_architecture_results.json
"""
import json
import pathlib
import random

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent

AISLE_W = 1.70
FOOTPRINT = (0.800, 0.580)
VMAX = 0.5
DT = 0.1
REACT_EDGE = 0.95          # 11.4's measured threshold for a clean pass
LATERAL_NEEDED = 0.580     # a full width, 11.4

# The warehouse as a graph of aisle segments. Lengths measured off the an earlier module
# map: the long run is the main aisle, the cross aisles connect the racks.
SEGMENTS = {
    "main_w":  22.0,
    "main_e":  14.0,
    "cross_1":  8.0,
    "cross_2":  8.0,
    "dock":     6.0,
}
# A task is a route through segments; these are the pick runs from the allocation experiment.
ROUTES = [
    ["dock", "main_w", "cross_1"],
    ["dock", "main_w", "main_e", "cross_2"],
    ["cross_1", "main_w", "dock"],
    ["cross_2", "main_e", "main_w", "dock"],
]


class Robot:
    def __init__(self, rid, route):
        self.id = rid
        self.route = list(route)
        self.leg = 0
        self.pos = 0.0          # metres into the current segment
        self.done = 0
        self.wait = 0.0
        self.blocked_since = None

    @property
    def seg(self):
        return self.route[self.leg] if self.leg < len(self.route) else None


def run(arch, n_robots=4, horizon_s=600.0, seed=0, kill_at=None):
    """Simulate one architecture. Returns a metrics dict.

    arch: "none" | "central" | "decentral"
    kill_at: seconds at which the decider dies, or None
    """
    rng = random.Random(seed)
    robots = [Robot(i, ROUTES[i % len(ROUTES)]) for i in range(n_robots)]
    for r in robots:                     # stagger the starts
        r.pos = rng.uniform(0.0, 2.0)
    owner = {}                           # segment -> robot id, for "central"
    deadlocks = 0
    stalled_after_kill = 0
    dead = False
    t = 0.0
    while t < horizon_s:
        if kill_at is not None and t >= kill_at:
            dead = True
        # who is where
        by_seg = {}
        for r in robots:
            if r.seg:
                by_seg.setdefault(r.seg, []).append(r)

        for r in robots:
            if r.seg is None:
                continue
            others = [o for o in by_seg.get(r.seg, []) if o is not r]
            move = True

            if arch == "none":
                # 11.4's result: an ONCOMING robot inside the react distance and
                # neither one yields, because neither knows the other is a robot.
                #
                # "Oncoming" has to be judged from the ROUTES, not from the ids.
                # My first version used `o.id % 2 != r.id % 2`, which makes
                # robots 0 and 1 permanently opposed wherever they are, so every
                # architecture-none run blocked from tick zero and scored exactly
                # 0.00 tasks at every fleet size. A sweep that flat is a dead
                # code path, not a finding -- and a one-robot control run
                # completing the theoretical maximum of 8 tasks proved the rest
                # of the simulation was fine.
                nxt_r = r.route[r.leg + 1] if r.leg + 1 < len(r.route) else None
                for o in others:
                    gap = abs(o.pos - r.pos) - FOOTPRINT[0]
                    if gap >= REACT_EDGE:
                        continue
                    nxt_o = o.route[o.leg + 1] if o.leg + 1 < len(o.route) else None
                    # they are opposed if each is heading where the other came
                    # from, i.e. their next segments differ AND they are closing
                    closing = (o.pos - r.pos) * (1 if nxt_r != nxt_o else -1) > 0
                    if nxt_r != nxt_o and closing:
                        move = False
                        deadlocks += 1 if gap < 0.30 else 0
            elif arch == "central":
                # a manager grants one robot per segment; the rest queue
                if dead:
                    # nothing new is granted once the manager is gone
                    if owner.get(r.seg) != r.id:
                        move = False
                        stalled_after_kill += 1
                else:
                    cur = owner.get(r.seg)
                    if cur is None:
                        owner[r.seg] = r.id
                    elif cur != r.id:
                        move = False
            elif arch == "decentral":
                # pairwise: the LOWER id keeps going, the higher yields. No
                # global view, so a robot can yield to someone who then leaves.
                for o in others:
                    gap = abs(o.pos - r.pos) - FOOTPRINT[0]
                    if gap < REACT_EDGE and o.id < r.id:
                        move = False

            if move:
                if r.blocked_since is not None:
                    r.wait += t - r.blocked_since
                    r.blocked_since = None
                r.pos += VMAX * DT
                if r.pos >= SEGMENTS[r.seg]:
                    if arch == "central" and owner.get(r.seg) == r.id:
                        del owner[r.seg]
                    r.pos = 0.0
                    r.leg += 1
                    if r.leg >= len(r.route):
                        r.done += 1
                        r.leg = 0        # take another task
            else:
                if r.blocked_since is None:
                    r.blocked_since = t
        t += DT

    for r in robots:
        if r.blocked_since is not None:
            r.wait += horizon_s - r.blocked_since
    waits = [r.wait for r in robots]
    total_done = sum(r.done for r in robots)
    return {
      "tasks_done": total_done,
      "throughput_per_min": round(total_done / (horizon_s / 60.0), 2),
      "mean_wait_s": round(sum(waits) / len(waits), 1),
      "worst_wait_s": round(max(waits), 1),
      "deadlock_ticks": deadlocks,
      "stalled_ticks_after_kill": stalled_after_kill,
    }


def main():
    out = {
      "provenance": "geometry from the measured map and an earlier module params; the "
                    "0.95 m react threshold and 0.580 m lateral requirement are "
                    "11.4's measured values; routes are 11.5's pick runs",
      "inputs": {"aisle_m": AISLE_W, "footprint_m": list(FOOTPRINT),
                 "vmax_ms": VMAX, "react_edge_m": REACT_EDGE,
                 "lateral_needed_m": LATERAL_NEEDED,
                 "segments_m": SEGMENTS, "routes": ROUTES},
    }
    # SEVERAL SEEDS. A single run of a stochastic start ordering is not a result;
    # 11.5 showed one lucky draw hitting the optimum exactly.
    archs = ("none", "central", "decentral")
    per = {}
    for a in archs:
        runs = [run(a, seed=s) for s in range(6)]
        per[a] = {
          "throughput_per_min": round(sum(r["throughput_per_min"] for r in runs) / 6, 2),
          "mean_wait_s": round(sum(r["mean_wait_s"] for r in runs) / 6, 1),
          "worst_wait_s": round(max(r["worst_wait_s"] for r in runs), 1),
          "deadlock_ticks": max(r["deadlock_ticks"] for r in runs),
          "seeds": 6,
          "throughput_spread": [r["throughput_per_min"] for r in runs],
        }
    out["by_architecture"] = per

    # the failure mode that decides real deployments
    out["decider_dies_at_300s"] = {
      a: run(a, seed=0, kill_at=300.0) for a in archs
    }

    # scaling: does the ranking hold with more robots?
    out["by_fleet_size"] = {
      str(n): {a: round(sum(run(a, n_robots=n, seed=s)["throughput_per_min"]
                            for s in range(4)) / 4, 2)
               for a in archs}
      for n in (2, 4, 6, 8)
    }
    pathlib.Path(ROOT / "reference/fleet_architecture_results.json").write_text(
        json.dumps(out, indent=2))
    print(json.dumps({"by_architecture": out["by_architecture"],
                      "by_fleet_size": out["by_fleet_size"]}, indent=2))


if __name__ == "__main__":
    main()
