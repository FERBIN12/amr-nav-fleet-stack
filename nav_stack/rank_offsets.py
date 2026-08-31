#!/usr/bin/env python3
"""Rank the offsets probed by probe_offsets.sh, best framing first."""
import glob, importlib.util, sys
from PIL import Image

spec = importlib.util.spec_from_file_location(
    "c", "scripts/check_robot_in_frame.py")
m = importlib.util.module_from_spec(spec)
sys.argv = ["x"]
try:
    spec.loader.exec_module(m)
except SystemExit:
    pass

rows = []
for f in glob.glob("/tmp/off_*.png"):
    rows.append((m.smooth_mass(Image.open(f)), f.split("off_")[1][:-4]))
if not rows:
    print("no probe frames; run probe_offsets.sh first")
    raise SystemExit(2)
print(f"{'offset':26s} {'machine mass':>12s}   (gate needs 70000)")
for v, n in sorted(rows, reverse=True):
    print(f"  {n.replace('_',' '):24s} {v:9d}   {'OK' if v >= 70000 else 'too small'}")
