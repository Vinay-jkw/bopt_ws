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

> The node reads `share/safety_demo/config/bopt_ws_body.db` relative to the working directory. Run from the workspace root.

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
| `warning` | ≥ 5 LiDAR points in a warning zone | Reduced to 1/3 of commanded velocity |
| `safe` | No obstacles detected, all systems healthy | Unmodified velocity passed through |

If `safety_turnoff` is active, velocities are always published unmodified regardless of status.

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

Markers are published as `LINE_STRIP` in the LiDAR's own frame (`base_link`):

- **Warning zone** — yellow (RGBA: 1, 1, 0, 0.5), line width 0.03 m
- **Danger zone** — red (RGBA: 1, 0, 0, 0.5), line width 0.05 m

Load `safety_viz.rviz` in RViz to see pre-configured zone overlays.


## BOPT LiDAR mapping

The runtime database uses these exact ROS topics:

| ID | Topic | Shape | Frame source | Purpose |
|---:|---|---|---|---|
| 1 | `/lidar/left/scan` | L-Shape | `LaserScan.header.frame_id` → TF | Left body safety |
| 2 | `/lidar/right/scan` | Mirror L-Shape | `LaserScan.header.frame_id` → TF | Right body safety |
| 3 | `/lidar/front/scan` | Rectangle | `LaserScan.header.frame_id` → TF | Front safety |
| 4 | `/lidar/back/scan` | Rectangle | `LaserScan.header.frame_id` → TF | Rear safety |
| 5 | `/Lidar_LFT` | Rectangle | `front_lidar_frame_left` in current BOPT simulation | Fork-left safety |
| 6 | `/Lidar_RFT` | Rectangle | `front_lidar_frame_right` in current BOPT simulation | Fork-right safety |

The `x_offset`, `y_offset`, `theta`, and `theta_N` columns are retained for database compatibility but are not used to place sensors. URDF/TF supplies the real sensor transform. Zone dimensions are built in each sensor's local frame and transformed to `base_link` using one TF lookup per scan; every LaserScan point uses that cached transform.

## Safety geometry rules

- `LaserScan` angle 0 is treated as the sensor +X axis.
- Rectangle width `a` spans local Y; length `b` extends along local +X.
- Sensor mounting position and orientation come only from TF.
- A valid point inside a danger polygon immediately sets that LiDAR to `danger`.
- Warning still requires 5 valid points.
- Pick/drop mode does not disable LiDARs 3, 5, or 6. Their normal safety policies remain active until a validated pick/drop-specific policy is added.
- If a required TF transform cannot be obtained, the affected LiDAR is treated as `danger` rather than `safe`.
- RViz zone markers are published in `base_link`.
