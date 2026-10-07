# safety_demo

Modular ROS2 safety monitoring system for autonomous mobile robots. Uses multiple LiDAR sensors to enforce configurable warning and danger zones, and automatically modulates velocity or triggers an emergency stop based on obstacle detection and system health.

---

## Package Structure

```text
safety_demo/
├── config/                          # SQLite database files
│   ├── bopt_2000.db                 # Default configuration (used at runtime)
│   ├── Updated_bopt2000.db
│   ├── Updated_bopt2000_with_frktip_zero.db
│   ├── amr_policies_01.db
│   └── amr_policies_with_third_lidar.db
├── include/safety_demo/
│   ├── database_manager.hpp
│   ├── error_handler.hpp
│   ├── policy_selector.hpp
│   ├── safety_node.hpp
│   ├── safety_zone_detector.hpp
│   ├── velocity_controller.hpp
│   └── visualization_publisher.hpp
├── launch/
│   └── ld19_launch.py               # Launches LiDAR drivers + static TF publishers
├── src/
│   ├── core/safety_node.cpp
│   ├── detectors/safety_zone_detector.cpp
│   ├── interfaces/velocity_controller.cpp
│   ├── interfaces/visualization_publisher.cpp
│   ├── nodes/main.cpp
│   └── utils/
│       ├── database_manager.cpp
│       ├── error_handler.cpp
│       └── policy_selector.cpp
├── safety_viz.rviz
├── CMakeLists.txt
└── package.xml
```

---

## Architecture

All components are independent classes instantiated and orchestrated by `SafetyNode`.

| Component | Responsibility |
| --- | --- |
| `DatabaseManager` | Opens SQLite DB, loads policies and LiDAR configs |
| `SafetyZoneDetector` | Defines zone polygons; point-in-polygon checks |
| `PolicySelector` | Picks the best-fit danger/warning policy for current speed + direction |
| `VelocityController` | Publishes velocity commands clamped by safety status |
| `VisualizationPublisher` | Publishes RViz `LINE_STRIP` markers for warning/danger zones |
| `ErrorHandler` | Watchdog for LiDAR timeouts and MQTT connectivity |
| `SafetyNode` | Main node — wires all components together |

---

## Dependencies

| Package | Use |
| --- | --- |
| `rclcpp` | ROS2 C++ client library |
| `sensor_msgs` | `LaserScan` |
| `nav_msgs` | `Odometry` |
| `std_msgs` | `String`, `Float64` |
| `geometry_msgs` | `Twist`, `Point` |
| `visualization_msgs` | `Marker` |
| `sqlite3_vendor` / `sqlite3` | SQLite database access |
| `tf2_ros` | Runtime only — `static_transform_publisher` in launch file |

---

## Building

```bash
cd /path/to/workspace
colcon build --packages-select safety_demo
source install/setup.bash
```

---

## Running

### Node only

```bash
ros2 run safety_demo safety_node_viz
```

> The node reads `src/safety_demo/config/bopt_2000.db` relative to the working directory. Run from the workspace root.

### Node with parameters

Tunable parameters live in `config/safety_params.yaml`. Load them with `--params-file`:

```bash
ros2 run safety_demo safety_node_viz \
    --ros-args --params-file src/safety_demo/config/safety_params.yaml
```

Without the flag the node uses the built-in defaults (which match the YAML).

---

## Topics

### Subscribed

| Topic | Type | Description |
| --- | --- | --- |
| *(loaded from DB)* | `sensor_msgs/LaserScan` | One subscription per LiDAR topic in the database |
| `/odometry/filtered` | `nav_msgs/Odometry` | Robot velocity (linear + angular) |
| `safety_turnoff` | `std_msgs/String` | `"TurnOff"` disables safety enforcement; any other value re-enables it |
| `/byd/safety` | `std_msgs/String` | `"pickdrop"`, `"normal"`, or `"parking"` — controls operational mode |
| `/mqtt/status` | `std_msgs/String` | `"connected"` or `"disconnected"` — MQTT broker health |

### Published

| Topic | Type | Description |
| --- | --- | --- |
| `safety_status` | `std_msgs/String` | Current status: `safe`, `warning`, `danger`, or `broker_disconnected` |
| `/cmd_vel_remapped` | `geometry_msgs/Twist` | Velocity command after safety scaling |
| `/velocity_remapped` | `std_msgs/Float64` | Scalar velocity after safety scaling |
| `visualization_marker_{id}_warning` | `visualization_msgs/Marker` | Yellow `LINE_STRIP` for warning zone (one per LiDAR ID) |
| `visualization_marker_{id}_danger` | `visualization_msgs/Marker` | Red `LINE_STRIP` for danger zone (one per LiDAR ID) |
| `machine/error/status` | `std_msgs/String` | Error codes (e.g. `E011`) |

---

## Safety Behaviors

### Status levels (in priority order)

| Status | Trigger | Velocity effect |
| --- | --- | --- |
| `broker_disconnected` | MQTT timeout (> 2 s) or explicit `"disconnected"` message | Zero — emergency stop |
| `danger` | ≥ 5 LiDAR points in a danger zone, or LiDAR timeout (> 5 s) | Zero — emergency stop |
| `warning` | ≥ 5 LiDAR points in a warning zone | **Proximity-scaled** — slows continuously with obstacle distance |
| `safe` | No obstacles detected, all systems healthy | Unmodified velocity passed through |

If `safety_turnoff` is active, velocities are always published unmodified regardless of status.

### Proximity-based velocity scaling

Inside the warning zone, velocity is no longer cut by a flat 1/3. Instead the node finds
the **closest obstacle point** across all LiDARs (the smallest sensor-to-point range that
falls inside a warning polygon) and maps it to a speed multiplier with a linear ramp:

```text
closest_range ≤ near  ->  factor = min_factor        (slowest, obstacle at danger edge)
closest_range ≥ far   ->  factor = 1.0               (full speed, obstacle at outer edge)
in between            ->  factor = min_factor + (1 - min_factor) * (range - near)/(far - near)
```

The most conservative (smallest) factor across all sensors is applied to both
`/cmd_vel_remapped` and `/velocity_remapped`. Danger zones still force a hard stop.

**`near` and `far` are not fixed numbers — they are derived per scan from the active
policy.** Each scan the node already selects a danger and a warning policy (based on
speed/direction) and builds their polygons; `near` is taken as the **danger field reach**
and `far` as the **warning field reach** (distance from the sensor to the farthest vertex).
So the ramp automatically spans whatever the live warning band is for the current speed,
direction and sensor — no per-policy tuning needed.

> Note: the warning band in `bopt_2000.db` is thin (~0.1–0.2 m between the danger and
> warning edges), so the ramp acts over a short distance. To make the slowdown more
> gradual, widen the `Warning` fields in the database relative to their `Danger` fields.

Only two values are set in `config/safety_params.yaml` (see [Running](#node-with-parameters)):

| Parameter | Default | Description |
| --- | --- | --- |
| `proximity_scaling_enabled` | `true` | When `false`, reverts to the fixed 1/3 warning reduction |
| `proximity_min_factor` | `0.15` | Slowest multiplier, applied at the danger edge of the band |

> If a policy's warning field is not larger than its danger field (degenerate band),
> the factor falls back to `proximity_min_factor`.

### Operational modes

| Mode | Trigger | Effect |
| --- | --- | --- |
| Normal | `/byd/safety` = `"normal"` | All LiDARs active |
| Pickdrop | `/byd/safety` = `"pickdrop"` | LiDAR IDs 3, 5, and 6 are forced to `safe` and skipped |
| Parking | `/byd/safety` = `"parking"` | Pickdrop logic is bypassed entirely |

---

## Configuration Database

The system uses SQLite. The default file is `config/bopt_2000.db`.

### `lidar` table

| Column | Description |
| --- | --- |
| `lidar_id` | Integer ID used throughout the system |
| `topic` | ROS topic name for this sensor |
| `shape_type` | Zone shape: `Rectangle`, `L-Shape`, or `Mirror L-Shape` |
| `x_offset`, `y_offset` | Sensor position relative to robot base |
| `theta`, `theta_N` | Rotation parameters for zone polygon |
| `description` | Human-readable label |

### `policy` table

Each policy defines speed thresholds and zone dimensions for a specific LiDAR, direction, and field type.

| Column | Description |
| --- | --- |
| `id` | Policy ID |
| `field_type` | `Warning` or `Danger` |
| `max_speed` | Linear speed threshold this policy applies up to |
| `angular_max_speed` | Angular speed threshold |
| `direction` | `Forward` or `Reverse` |
| `angular_direction` | `Clockwise` or `Counter-Clockwise` |
| `warning_zone` / `danger_zone` | Zone dimensions: `a`, `b`, `i1`, `i2`, `o1`, `o2` |

Policy selection picks the tightest policy whose `max_speed` ≥ current speed and whose direction matches the robot's current motion.

---

## Error Codes

| Code | Meaning |
| --- | --- |
| `E011` | LiDAR data timeout (no message received for > 5 s) |

---

## Watchdog Timers

Both watchdogs fire every 200 ms.

- **LiDAR watchdog**: if any LiDAR topic has not published in > 5 s, sets `lidar_timeout_fault_` → status becomes `danger` and `E011` is published once.
- **MQTT watchdog**: if `/mqtt/status` has not been received in > 2 s, or the last message was `"disconnected"`, sets `mqtt_timeout_fault_` → status becomes `broker_disconnected`.

---

## Visualization

Markers are published as `LINE_STRIP` in the LiDAR's own frame (`frame_{topic}`):

- **Warning zone** — yellow (RGBA: 1, 1, 0, 0.5), line width 0.03 m
- **Danger zone** — red (RGBA: 1, 0, 0, 0.5), line width 0.05 m

Load `safety_viz.rviz` in RViz to see pre-configured zone overlays.
