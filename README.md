# Warehouse AMR Navigation & Fleet Stack

A ROS 2 Nav2 navigation stack for a warehouse-class autonomous mobile robot
(AMR), built around real machine dimensions (MiR250 / OTTO 100 class:
0.800 x 0.580 m footprint, 250 kg payload, differential drive). The project
covers the full pipeline from state estimation through single-robot planning
to multi-robot fleet coordination, with an emphasis on measuring claims
instead of asserting them.

## What's here

**`ros2_ws/src/cortex_amr_description/`** — the robot description and Nav2
configuration: URDF/xacro with Gazebo plugins, an EKF sensor-fusion config,
SLAM params, two alternate local controllers (DWB and Regulated Pure Pursuit),
a ROS 2/Gazebo bridge config, and a warehouse launch file.

**`nav_stack/planning/`** — path planning and control:
- `grid_search.py`, `rrt.py`, `hybrid_astar.py` — global planners, compared
  on the same warehouse map
- `smoothing.py` — post-processing a raw plan into a drivable path
- `pursuit.py`, `regulated_pursuit.py` — pure pursuit and its
  obstacle/curvature-regulated variant
- `dwb.py` — local trajectory scoring
- `mppi.py` — sampling-based model-predictive control

**`nav_stack/perception/`** — obstacle handling and safety:
- `costmap_from_depth.py`, `voxel_layer.py` — building a costmap from depth
  data
- `velocity_obstacles.py`, `dodge_test.py` — reactive avoidance
- `detect_person_pallet.py`, `static_vs_dynamic.py` — classifying what's in
  the way
- `sensor_failures.py`, `safety_stop.py` — degraded-sensor handling and a
  fail-safe stop

**`nav_stack/fleet/`** — multi-robot coordination:
- `fleet_architecture.py`, `why_fleets_differ.py` — namespacing and
  architecture tradeoffs for running more than one robot
- `traffic_deadlock.py` — measures where two AMRs actually deadlock in a
  shared aisle (not asserted — the script found two real bugs in its own
  first version: a centre-to-centre vs. edge-to-edge distance error, and a
  simulation tick step too coarse to resolve the recovery window)
- `task_allocation.py` — assigning jobs across a fleet
- `namespace_two.py`, `two_robots_run.py`, `scope_sdf.py` — running and
  scoping two robots in the same simulation

**`nav_stack/capstone/mission_node.py`** — a ROS 2 action client that ties
the above together: drives a robot through a sequence of warehouse station
poses via `NavigateToPose`, with the same recovery policy tuned and
stress-tested elsewhere in the project.

**`nav_stack/*.py`** (top level) — sensor-fusion utilities: covariance
tuning (`odom_cov_fill.py`), an EKF visualizer (`ekf_show.py`), IMU/wheel
offset ranking (`rank_offsets.py`), and laser-scan declipping
(`scan_declip.py`).

**`data/state_estimation_runs/`** — real logged odometry, gyro, and
EKF-fused CSV traces from four drive profiles (final, lap, run, slow), used
to validate the sensor-fusion tuning against actual runs rather than
simulation alone.

## Notes

Several scripts document real bugs found during development rather than a
clean success story — e.g. `traffic_deadlock.py`'s docstring walks through
two measurement bugs that produced a suspiciously uniform result before the
real deadlock geometry was found. That's intentional: a flat, identical
result across many different inputs is usually a sign of a bug, not a
finding, and the fix is left in the code as documentation.

## Stack

ROS 2 (Nav2, robot_localization EKF, slam_toolbox), Gazebo, Python 3.
