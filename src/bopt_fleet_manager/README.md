# BOPT Fleet Manager — Phase 1 (Read-only)

This package is deliberately a **read-only monitor** for the Gazebo pallet-cycle test setup. It subscribes to robot poses and selected status strings, calculates pairwise XY separation, detects stale/missing poses, and prints advisory status. It publishes no ROS topics and calls no services.

## Source review findings

The uploaded `workflow_node/handlers/workflow_handler.py` subscribes to the configured `conflict_action` topic as `std_msgs/msg/String`. `reroute_handler.py` parses `msg.data` as JSON and expects `{"type":"halt"}` / `{"type":"resume"}`; it then calls `/robot_client/set_parameters` to set `should_halt`.

However, the uploaded active entry point `nmpc_controller/main.py` instantiates `nmpc_controller.core.controller.NMPCController`. That class subscribes to `current_pose` and publishes `velocity`, `steering_angle`, `state`, and `safety_turnoff`; it does **not** declare `should_halt` and does **not** subscribe to `conflict_action`. The `should_halt` implementation appears in a separate `nmpc_controller_v2_fast.py`, but `setup.py` only registers `main = nmpc_controller.main:main`. Therefore the source archive does not establish that the current active NMPC process will honor the workflow halt/resume path. Do not enable automated halt/resume until the actually running executable and its command interface are verified and tested.

The supplied `bopt_fleet_manager_v2(4).py` is a separate test orchestrator for spawning/removing pallets and starting Pickup/Drop workflows with staggered delays. Keep it unchanged for now; this package is an independent monitor and does not launch, stop, or alter its cycles.

## Install into `~/bopt_ws`

Copy this package folder into the workspace source tree:

```bash
cp -r bopt_fleet_manager ~/bopt_ws/src/
cd ~/bopt_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bopt_fleet_manager
source install/setup.bash
```

## Run in a terminal

Start Gazebo and your existing robot/workflow launch as usual first. Then:

```bash
source /opt/ros/humble/setup.bash
source ~/bopt_ws/install/setup.bash
ros2 launch bopt_fleet_manager fleet_monitor.launch.py
```

Or run directly:

```bash
ros2 run bopt_fleet_manager fleet_monitor
```

## Verify read-only behavior and topics

In another terminal:

```bash
ros2 topic info /robot001/current_pose --verbose
ros2 topic info /robot002/current_pose --verbose
ros2 topic info /robot003/current_pose --verbose
ros2 node info /bopt_fleet_monitor
```

Expected subscriptions include `/robot00X/current_pose`, `/robot00X/state`, and `/robot00X/byd/current_task`. There should be no publishers from `/bopt_fleet_monitor`.

If your active task topic differs, update the topic in `fleet_monitor.py` only after confirming it with `ros2 topic list -t`. If current_pose is not `geometry_msgs/msg/PoseStamped`, adjust the subscriber type after inspecting the live type.

## Thresholds

`4.0 m` stop threshold and `4.5 m` release threshold are **advisory display thresholds only** in this phase. They are not a safety guarantee. The monitor uses XY distance and pose receive-time freshness; it does not predict trajectories or braking distance.

## Validation

The source includes unit tests for the advisory hysteresis logic. They can be run in a ROS/Python environment with pytest:

```bash
python3 -m pytest test/test_pair_advisory.py
```

No physical or simulated robot behavior is changed by this package.
