# 360degree safety module

## Overview

The 360degree safety module is a ROS2 safety feature for the AMR. It monitors LiDAR data around the robot and changes the robot output velocity based on obstacle position, operating mode, and system health.

This document is only for feature understanding and testing reference.

## LiDAR Coverage

| LiDAR | Topic | Area Covered |
| --- | --- | --- |
| 1 | `Lidar_1` | Left side |
| 2 | `Lidar_2` | Right side |
| 3 | `Lidar_3` | Fork-side/front area |
| 4 | `Lidar_4` | Non-fork-side area |

The module uses warning and danger zones for each LiDAR. Zone size is selected from the safety database based on robot speed and movement direction.

## Main Features

### 1. Obstacle Detection

- Reads scan data from all configured LiDARs.
- Checks whether obstacle points are inside warning or danger zones.
- Requires 5 or more valid points in a zone before changing status, to reduce false triggers.

### 2. Safety Status

Published topic: `safety_status`

| Status | Meaning |
| --- | --- |
| `safe` | No obstacle or active fault |
| `warning` | Obstacle detected in warning zone |
| `danger` | Obstacle detected in danger zone or LiDAR timeout |
| `broker_disconnected` | MQTT connection fault |

Priority order: MQTT fault, danger, warning, safe.

### 3. Velocity Control

Input topics:

| Topic | Use |
| --- | --- |
| `/cmd_vel` | Original robot command |
| `/velocity` | Original scalar velocity |
| `/odometry/filtered` | Actual robot speed used for zone selection |

Output topics:

| Topic | Use |
| --- | --- |
| `/cmd_vel_remapped` | Safety-modified robot command |
| `/velocity_remapped` | Safety-modified scalar velocity |

Expected output behavior:

| Status | Velocity Output |
| --- | --- |
| `safe` | No reduction |
| `warning` | Reduced to one-third |
| `danger` | Zero velocity |
| `broker_disconnected` | Zero velocity |

### 4. Safety Turnoff

Subscribed topic: `safety_turnoff`

| Message | Behavior |
| --- | --- |
| `TurnOff` | Safety speed limiting is disabled; velocity passes through unchanged |
| Any other value | Safety speed limiting is enabled again |

### 5. Operating Modes

Subscribed topic: `/byd/safety`

| Message | Behavior |
| --- | --- |
| `normal` | Normal safety behavior |
| `pickdrop` | Fork-area LiDARs are skipped and forced safe |
| `parking` | Parking-specific safety policy is used |

In the current four-LiDAR setup, `pickdrop` mainly affects `Lidar_3`.

### 6. Fault Handling

| Fault | Trigger | Result |
| --- | --- | --- |
| LiDAR timeout | Any configured LiDAR has no scan for more than 5 seconds | Status becomes `danger`; error `E011` is published |
| MQTT disconnected | `/mqtt/status = disconnected` | Status becomes `broker_disconnected`; velocity becomes zero |
| MQTT timeout | No MQTT update for more than 2 seconds after first message | Status becomes `broker_disconnected`; velocity becomes zero |

Error topic: `machine/error/status`

### 7. Visualization

The module publishes RViz markers for active safety zones:

| Zone | Marker Color | Topic Pattern |
| --- | --- | --- |
| Warning | Yellow | `visualization_marker_<lidar_id>_warning` |
| Danger | Red | `visualization_marker_<lidar_id>_danger` |

The package includes `safety_viz.rviz` for viewing the zones.

## Testing Checklist

| Scenario | Expected Result |
| --- | --- |
| No obstacle and all systems healthy | `safe`, velocity unchanged |
| Obstacle in warning zone | `warning`, velocity reduced to one-third |
| Obstacle in danger zone | `danger`, velocity zero |
| `safety_turnoff = TurnOff` | Velocity passes through unchanged |
| `/byd/safety = pickdrop` | Fork-area LiDAR detection is skipped |
| LiDAR scan timeout over 5 seconds | `danger`, `E011` published |
| `/mqtt/status = disconnected` | `broker_disconnected`, velocity zero |
| MQTT update timeout over 2 seconds | `broker_disconnected`, velocity zero |
| RViz markers visible | Warning zone yellow, danger zone red |
