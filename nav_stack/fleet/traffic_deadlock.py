#!/usr/bin/env python3
"""11.4 -- when do two AMRs deadlock in a shared aisle, and what breaks the tie?

Not a hand-waved "they might block each other". This measures the geometry from
OUR OWN numbers, and simulates the interaction rule that Nav2 actually gives you
by default, which is: nothing. Two robots each treat the other as an unlabelled
obstacle in their local costmap.

INPUTS, all measured earlier in this project:
  footprint      0.800 x 0.580 m          (cortex_amr, MiR250 class)
  inscribed r    0.290 m                  (half the 0.580 width)
  circumscribed  0.494 m                  (half diagonal of 0.800x0.580)
  aisle width    1.70 m                   (measured from the map)
  two robots hugging opposite walls sit 1.70 - 0.58 = 1.12 m apart, cost 0
  inflation radius 0.55 m, cost_scaling 3.0 (an earlier module params)
  max speed      0.5 m/s                  (an earlier module)

THE QUESTION. Passing is geometrically fine (11.1 proved 1.12 m clearance at
cost 0). So why do fleets deadlock at all? Answer has to come from the CONTROLLER
and the SEQUENCE, not from the widths. This script tests that claim instead of
asserting it.

TWO BUGS I HAD TO FIX FIRST, both found because the first run returned DEADLOCK
with identical ticks (110) and identical min_gap (0.250) on all TWELVE inputs. A
sweep that flat is a dead code path, not a result.

  BUG 1  cost read at CENTRE-TO-CENTRE distance. Inflation is measured from the
         obstacle's occupied CELLS, i.e. from its footprint edge. Two robots at
         0.58 m centre separation are already touching. The edge gap is
         gap - 2*half_width for a side-by-side pass and gap - 2*half_length
         head-on, and using centres understated the cost by a whole footprint.
  BUG 2  lateral step of 0.0075 m per 0.1 s tick, i.e. 0.075 m/s of sideways
         travel. Between the first non-zero cost and the lethal ring there are
         only about 3 ticks, so a robot could shift 0.02 m total. That is why
         every lateral_policy from 0.0 to 0.56 gave the SAME answer: the policy
         was unreachable. A differential robot turning at 1.0 rad/s while moving
         at 0.5 m/s develops lateral rate on the order of 0.25 m/s, so the step
         is 0.025 m per tick, and it must start when the cost first goes
         non-zero rather than when the robot is already committed.

Run:  python3 nav_stack/fleet/traffic_deadlock.py
Writes: reference/traffic_deadlock_results.json
"""
import json
import math
import pathlib

FOOTPRINT = (0.800, 0.580)
INSCRIBED = 0.290
CIRCUM = math.hypot(0.800, 0.580) / 2.0
AISLE = 1.70
INFL_R = 0.55
COST_SCALE = 3.0
VMAX = 0.5
DT = 0.1

ROOT = pathlib.Path(__file__).resolve().parent.parent.parent


def inflation_cost(d_edge):
    """Nav2 inflation cost at distance d_edge from the obstacle's occupied cells.

    Takes an EDGE distance, not a centre separation. Passing centres in here was
    bug 1: it understated the cost by a whole footprint width.

    253 inside the inscribed radius (lethal to the footprint), an exponential
    decay out to the inflation radius, 0 beyond it. This is the curve derived in
    costmap derivation and its inversion, not a guess.
    """
    if d_edge <= 0.0:
        return 254.0                      # in collision
    if d_edge <= INSCRIBED:
        return 253.0                      # lethal to our own footprint
    if d_edge >= INFL_R:
        return 0.0
    return 252.0 * math.exp(-COST_SCALE * (d_edge - INSCRIBED))


def head_on(aisle_w, lateral_policy, steps=900, sym_break=0.0,
            react_at=INFL_R):
    """Two robots driving at each other down one aisle.

    lateral_policy: how far off centreline each robot will go, in m.
    sym_break: if non-zero, robot B aims for the OTHER side by this fraction --
        the tie-break a fleet manager would impose. 0.0 means both robots run
        identical code and pick identical sides.
    react_at: EDGE distance at which a robot begins diverting. The default is the
        inflation radius, which is where a costmap-driven planner first feels
        anything. Raising it is what a planner with a longer horizon, or a fleet
        manager with foreknowledge, buys you.

    Returns (outcome, ticks, min_edge_gap, final_separation).

    Distances are EDGE gaps. Head-on, the closing dimension is LENGTH, so the
    edge gap along the aisle is |dx| - 0.800. Across the aisle the relevant
    dimension is WIDTH, so two robots pass when |dy| >= 0.580.
    """
    half_len = FOOTPRINT[0] / 2.0
    half_wid = FOOTPRINT[1] / 2.0
    y_lim = aisle_w / 2.0 - half_wid          # wall-limited centreline offset
    a = {"x": -4.0, "y": 0.0}
    b = {"x": +4.0, "y": 0.0}
    min_gap = 1e9
    stalled = 0
    LAT_RATE = 0.25                            # m/s achievable sideways, bug 2
    for t in range(steps):
        dx = abs(b["x"] - a["x"])
        dy = abs(b["y"] - a["y"])
        # already clear of each other laterally? then the pass is free
        lateral_clear = dy >= FOOTPRINT[1]
        edge_gap = dx - 2 * half_len
        min_gap = min(min_gap, edge_gap)
        c = 0.0 if lateral_clear else inflation_cost(edge_gap)
        # A robot may begin diverting BEFORE the costmap makes it hurt, if
        # something tells it to. That is the variable this script sweeps.
        diverting = (not lateral_clear) and (edge_gap <= react_at)
        moving = 0
        for r, sgn, want_sign in ((a, +1, +1.0), (b, -1, +1.0 - 2.0 * sym_break)):
            if c >= 253.0:
                v = 0.0
            elif diverting:
                target = math.copysign(min(lateral_policy, y_lim), want_sign)
                d = target - r["y"]
                r["y"] += math.copysign(min(LAT_RATE * DT, abs(d)), d) if d else 0.0
                v = sgn * VMAX * 0.5
            else:
                v = sgn * VMAX
            r["x"] += v * DT
            if abs(v) > 1e-6:
                moving += 1
        if moving == 0:
            stalled += 1
            if stalled > 40:
                return ("DEADLOCK", t, round(min_gap, 3),
                        round(abs(b["x"] - a["x"]), 3))
        else:
            stalled = 0
        if a["x"] > b["x"] + 1.0:
            return ("PASSED", t, round(min_gap, 3), round(abs(a["x"] - b["x"]), 3))
    return ("TIMEOUT", steps, round(min_gap, 3), round(abs(b["x"] - a["x"]), 3))


def main():
    res = {"provenance": "geometry from the measured map and an earlier module params; "
                         "the interaction is DEFAULT Nav2 behaviour, which is "
                         "that neither robot knows the other is a robot",
           "inputs": {"footprint_m": list(FOOTPRINT), "inscribed_r_m": INSCRIBED,
                      "circumscribed_r_m": round(CIRCUM, 3),
                      "aisle_m": AISLE, "inflation_r_m": INFL_R,
                      "cost_scaling": COST_SCALE, "vmax_ms": VMAX,
                      "lateral_rate_ms": 0.25},
           "bugs_fixed_first": [
             "cost was read at CENTRE separation, not edge distance: understated "
             "by a whole footprint",
             "lateral step was 0.075 m/s, so no lateral_policy was reachable in "
             "the ~3 ticks before lethal; all 12 rows returned identical values"]}

    # 1. the cost curve at EDGE distances that matter
    res["cost_at_edge_distance"] = {
        "%.2f" % d: round(inflation_cost(d), 1)
        for d in (0.0, 0.10, 0.29, 0.35, 0.40, 0.45, 0.55, 0.60, 1.12)}

    # 2. lateral policy sweep, BOTH robots choosing the same side (the default)
    same = []
    for pol in (0.0, 0.10, 0.20, 0.30, 0.40, 0.56):
        out, ticks, mg, sep = head_on(AISLE, pol, sym_break=0.0)
        same.append({"lateral_policy_m": pol, "sym_break": 0.0,
                     "outcome": out, "ticks": ticks,
                     "min_edge_gap_m": mg, "final_sep_m": sep})
    res["same_side_sweep"] = same

    # 3. the SAME sweep with the tie broken: B aims for the other wall
    opp = []
    for pol in (0.0, 0.10, 0.20, 0.30, 0.40, 0.56):
        out, ticks, mg, sep = head_on(AISLE, pol, sym_break=1.0)
        opp.append({"lateral_policy_m": pol, "sym_break": 1.0,
                    "outcome": out, "ticks": ticks,
                    "min_edge_gap_m": mg, "final_sep_m": sep})
    res["opposite_side_sweep"] = opp

    # 4. THE ONE THAT DISCRIMINATES: how early must a robot start diverting?
    #    Sweep react_at, the edge distance at which divert begins, with the tie
    #    broken and a full-shoulder policy.
    early = []
    for ra in (0.55, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0):
        out, ticks, mg, sep = head_on(AISLE, 0.56, sym_break=1.0, react_at=ra)
        early.append({"react_at_edge_m": ra, "outcome": out, "ticks": ticks,
                      "min_edge_gap_m": mg})
    res["react_distance_sweep"] = early

    # 5. and the same sweep WITHOUT the tie broken, to separate the two causes
    early_same = []
    for ra in (0.55, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0):
        out, ticks, mg, sep = head_on(AISLE, 0.56, sym_break=0.0, react_at=ra)
        early_same.append({"react_at_edge_m": ra, "outcome": out,
                           "ticks": ticks, "min_edge_gap_m": mg})
    res["react_distance_sweep_no_tiebreak"] = early_same

    # 6. aisle width at the react distance that first worked
    aisles = []
    for w in (1.00, 1.20, 1.40, 1.57, 1.70, 2.00, 2.40):
        out, ticks, mg, sep = head_on(w, 0.56, sym_break=1.0, react_at=3.0)
        aisles.append({"aisle_m": w, "outcome": out, "ticks": ticks,
                       "y_limit_m": round(w / 2.0 - FOOTPRINT[1] / 2.0, 3),
                       "min_edge_gap_m": mg})
    res["aisle_sweep_early_and_tie_broken"] = aisles

    pathlib.Path(ROOT / "reference/traffic_deadlock_results.json").write_text(
        json.dumps(res, indent=2))
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
