#!/usr/bin/env python3
"""Namespacing two AMRs: what actually has to change, measured.

11.1 established that the namespace stops being private and verified three sharp
edges on this machine. This file turns those into the concrete list of changes,
counted against OUR OWN files rather than a tutorial's.

THE THREE EDGES, all verified in 11.1 (see
reference/why_fleets_differ_results.json):
  frame_prefix is a SEPARATE parameter from the namespace: namespacing a node
    does not prefix the TF frames it publishes
  a params file keyed on an unqualified node name STOPS MATCHING once the node is
    namespaced, and the node DIES rather than warning
  absolute topic names ignore the namespace, and our own collision_monitor has
    two of them
"""
import json
import pathlib
import re
import sys

CFG = pathlib.Path.home() / "amr_ws/src/cortex_amr_description/config/nav2_params.yaml"
BRIDGE = CFG.parent / "bridge.yaml"
URDF = CFG.parent.parent / "urdf/cortex_amr.urdf.xacro"
GAZEBO = CFG.parent.parent / "urdf/cortex_amr.gazebo.xacro"


def count_absolute_topics(path):
    """Topic-looking strings with a LEADING SLASH: these ignore a namespace."""
    hits = []
    if not path.exists():
        return hits
    for n, line in enumerate(path.read_text().splitlines(), 1):
        if line.strip().startswith("#"):
            continue
        for m in re.finditer(r'["\'](/[a-z_][a-z0-9_/]*)["\']', line):
            t = m.group(1)
            # a filesystem path is not a topic
            if t.startswith("/home") or t.startswith("/opt") or t.startswith("/tmp"):
                continue
            hits.append({"line": n, "topic": t})
    return hits


def count_node_keys(path):
    """Top-level YAML keys that are node names needing a namespace prefix."""
    if not path.exists():
        return []
    out = []
    for n, line in enumerate(path.read_text().splitlines(), 1):
        m = re.match(r'^([a-z_][a-z0-9_]*):\s*$', line)
        if m:
            out.append({"line": n, "node": m.group(1)})
    return out


def count_frames(path):
    """Frame names in the urdf: every one needs the prefix."""
    if not path.exists():
        return 0
    return len(re.findall(r'<link\s+name=', path.read_text()))


def main():
    print("WHAT HAS TO CHANGE, counted in our own files")
    print()

    nodes = count_node_keys(CFG)
    print("1. NODE KEYS in nav2_params.yaml: %d" % len(nodes))
    for d in nodes[:8]:
        print("     line %4d  %s" % (d["line"], d["node"]))
    if len(nodes) > 8:
        print("     ... and %d more" % (len(nodes) - 8))
    print("   each must become /r1/<node> or the node starts with NO parameters")
    print("   and dies on the first one it needs. Verified in 11.1.")
    print()

    abs_cfg = count_absolute_topics(CFG)
    abs_br = count_absolute_topics(BRIDGE)
    print("2. ABSOLUTE TOPIC NAMES (leading slash, so namespace-immune)")
    print("   nav2_params.yaml: %d" % len(abs_cfg))
    for d in abs_cfg:
        print("     line %4d  %s" % (d["line"], d["topic"]))
    print("   bridge.yaml: %d" % len(abs_br))
    print("   the bridge ones are FINE to leave absolute if each robot gets its")
    print("   own bridge with its own gz topics; the nav2 ones are not.")
    print()

    frames = count_frames(URDF)
    print("3. TF FRAMES in the urdf: %d links" % frames)
    print("   frame_prefix 'r1/' renames all %d at once, which is why it exists"
          % frames)
    print("   as a parameter rather than a per-link edit.")
    print()

    print("4. WHAT MUST NOT BE DUPLICATED")
    print("   the map. Two map_servers serving the same file is wasteful but")
    print("   harmless; two /map topics under different namespaces means the two")
    print("   robots cannot share a costmap or reason about each other at all.")
    print("   So map_server stays GLOBAL and only the consumers are namespaced.")
    print()

    total = len(nodes) + len(abs_cfg) + 1 + 1
    print("SO THE CHANGE LIST IS %d ITEMS, not one:" % total)
    print("   %d node keys re-qualified" % len(nodes))
    print("   %d absolute topics made relative" % len(abs_cfg))
    print("   1 frame_prefix parameter added per robot")
    print("   1 decision about what stays global (the map)")
    print()
    print("   and NONE of them is inside the navigation algorithms. This is")
    print("   plumbing, which is why it is easy once you know the three edges")
    print("   and impossible to debug if you do not.")

    out = {
        "provenance": "counted directly out of "
                      "~/amr_ws/src/cortex_amr_description/{config,urdf}; the "
                      "three edges were verified live in 11.1",
        "node_keys": nodes,
        "node_key_count": len(nodes),
        "absolute_topics_nav2": abs_cfg,
        "absolute_topics_nav2_count": len(abs_cfg),
        "absolute_topics_bridge_count": len(abs_br),
        "urdf_links": frames,
        "change_list_items": total,
        "stays_global": ["map_server / the /map topic"],
        "finding": (
            "Namespacing two AMRs is a %d item change list against our own files: "
            "%d node keys in nav2_params.yaml re-qualified (an unqualified key "
            "stops matching and the node dies rather than warning), %d absolute "
            "topic names made relative, one frame_prefix per robot which renames "
            "all %d urdf links at once, and one decision that the map stays "
            "global. None of it is inside the navigation algorithms."
            % (total, len(nodes), len(abs_cfg), frames)),
    }
    with open("reference/namespace_two_results.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nwrote reference/namespace_two_results.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
