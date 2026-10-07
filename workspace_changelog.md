# BOPT Workspace (bopt_ws) Changelog

This document outlines all current uncommitted modifications and new additions across the `bopt_ws` workspace.

## 1. New Packages Added
- **`costmap_pkg`**: Added a new package for handling 2D/3D costmap generation and configuration.
- **`costmap_to_laserscan`**: Added a new package for converting costmaps into LaserScan messages for use with obstacle avoidance and safety layers.

## 2. `bopt_controller` (Teleop Keyboard)
*File: `src/bopt_controller/bopt_controller/teleop_keyboard.py`*
- **Instant Deceleration:** Removed deceleration logic for stopping. The robot instantly snaps to `0.0` when the target speed is `0.0`.
- **Exponential Steering Approach:** Added an exponential approach function (`k_steer = 3.0`) so the steering wheel slows down smoothly as it gets closer to the target angle.
- **Instant Steering Return-to-Center:** Restored the instant-snap logic so the steering wheel instantly snaps back to `0.0` when keys are released.
- **Steering-Aware Velocity Throttling:** Introduced an exponential speed reduction multiplier based on the steering error (`k_speed_reduction = 4.0`). Forward wheel velocity is exponentially penalized while the steering is off-target, preventing jerky acceleration.

## 3. `safety_demo` (Major Refactor)
*Multiple files modified (Core, Detectors, Interfaces, Utils)*
- **Removed Pickdrop Mode:** Removed `pickdrop_mode` entirely from the `selectPolicies` and `selectSinglePolicy` signatures and logic. The special exemption for the "Pickdrop Zone" policy was deleted.
- **Simplified Policy Filtering:** Refactored the loop that filters applicable policies. It now cleanly includes only "PRKNG_POLICY" when in parking mode, and excludes it during normal operation.
- **Refactored Policy Selection:** Simplified policy sorting to only sort by `max_speed`. Improved the logic that selects the tightest policy covering the current linear speed.
- **General Cleanup:** Over 3,000 lines changed across headers and source files to streamline the safety node, database manager, error handler, and velocity controller.
- **New Config:** Added new `safety_params.yaml` and updated SQLite databases.

## 4. `bopt_description` (Simulation & Worlds)
- **New Warehouse World:** Added `fg_warehouse.world` and `FG_warehouse_map.stl` for a new warehouse simulation environment.
- **Multi-Robot Support:** Added a new `multi_robot_spawn.launch.py` script.
- **Configuration Updates:** Updated `gazebo.launch.py`, `simulation.launch.py`, URDF/Xacro files, and tweaked RViz display configurations.

## 5. `bopt_localization` & `bopt_mapping`
- **Map Replacements:** Deleted the old `bopt_map` and `bopt_map3` files. Added new maps for the warehouse (`warehouse_fg.pgm`, `.yaml`, `.data`, `.posegraph`).
- **Launch Updates:** Updated `localization.launch.py`, `map.launch.py`, and `robot_localize.launch.py` to point to the new maps and configurations.
- **SLAM Toolbox:** Tweaked `slam_toolbox.yaml` parameters and `mapping.launch.py` to optimize mapping.

## 6. `workflow_node` (Setup & Node)
- **Map Details Installation:** Updated `setup.py` to correctly install the `map_details` directory and its contents into `share/workflow_node/map_details`.
- **Code Formatting:** Minor formatting and linting improvements in `workflow_node.py` (consolidated `holded_nodes_publisher.publish` onto a single line).
- **Data Update:** Binary changes in `constructed_rs_path.pkl` (likely updated generated paths).
