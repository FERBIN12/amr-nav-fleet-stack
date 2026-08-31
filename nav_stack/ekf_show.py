#!/usr/bin/env python3
"""Print one ekf.yaml sensor config as "N entries, K true: [names]".

WHY THIS IS A FILE. Inlining this as `python3 -c '...'` inside a take body needs
four levels of quoting -- the outer double-quoted bash string, the `say 'cmd'`
single quotes, python's own quotes, and the dict keys -- and over-escaped bodies
are the documented way this capture rig breaks (a printf once mangled to
'?xml: No such file'). A file has no quoting problem at all.

Usage: ekf_show.py <ekf.yaml> <odom0_config|imu0_config|process_noise_covariance>
"""
import sys

import yaml

NAMES = ["x", "y", "z", "roll", "pitch", "yaw",
         "vx", "vy", "vz", "vroll", "vpitch", "vyaw",
         "ax", "ay", "az"]


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    cfg, key = sys.argv[1], sys.argv[2]
    params = yaml.safe_load(open(cfg))["ekf_filter_node"]["ros__parameters"]
    v = params[key]
    if key == "process_noise_covariance":
        print(f"{len(v)} values on the diagonal")
        return 0
    true_names = [NAMES[i] for i, b in enumerate(v) if b]
    print(f"{len(v)} entries, {sum(bool(b) for b in v)} true: {true_names}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
