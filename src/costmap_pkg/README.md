# costmap_to_laserscan

Projects a Nav2 costmap into a `sensor_msgs/LaserScan` expressed in the robot base frame.
ROS 2 Humble, C++17, composable.

```
/local_costmap/costmap  (nav_msgs/OccupancyGrid, or nav2_msgs/Costmap)
        |  + TF costmap_frame -> base_link
        v
/costmap_scan           (sensor_msgs/LaserScan, frame_id = base_link)
```

## Algorithm

One Amanatides–Woo DDA walk per beam, cast from the base-frame origin into the raster:

* **Exact traversal.** Every cell the beam crosses is visited exactly once — no
  step-size oversampling, no skipped cells at grazing angles.
* **Sub-cell ranges.** The reported range is the parametric distance at which the beam
  *enters* the first blocked cell, not the distance to the cell centre. Error is bounded
  by float precision, not by the grid resolution.
* **First hit wins**, so occlusion is handled correctly: a beam never reports something
  behind a wall.

## Performance notes

| Technique | Why |
|---|---|
| Precomputed `cos`/`sin` beam tables, rotated by one 2×2 matrix per beam | 4 multiplies per beam instead of a `sin`/`cos` pair, per cycle |
| 256-entry `blocked[]` LUT instead of comparisons | branchless cell test; makes `int8` (OccupancyGrid) and `uint8` (Costmap) one code path |
| Linear index carried incrementally (`idx += ±1` / `±stride`) | no multiply inside the walk |
| `static_cast<uint32_t>(ix) >= width` | one compare covers both bounds |
| Slab clip before the walk | out-of-raster beams cost O(1); starting at `range_min` skips the footprint cells outright |
| Single `MutuallyExclusive` callback group for sub + timer | grid state needs **no locking**, even under a `MultiThreadedExecutor` |
| One `memcpy` mirror + in-place `OccupancyGridUpdate` patching | no per-cell conversion on ingest |
| `publish(std::move(unique_ptr))` | zero-copy on the intra-process path when composed |
| Skips the cast when nothing is subscribed | idle cost ≈ 0 |

Measured on one Xeon 2.8 GHz core, `-O3`, single-threaded (median / p99 per scan):

| Costmap | Beams | Median | p99 |
|---|---|---|---|
| 6×6 m @ 5 cm, empty (worst case) | 720 | 101 µs | 137 µs |
| 6×6 m @ 5 cm, 2 % occupied | 720 | 60 µs | 89 µs |
| 6×6 m @ 5 cm, 10 % occupied | 720 | 23 µs | 45 µs |
| 6×6 m @ 5 cm, 0.25° resolution | 1440 | 135 µs | 163 µs |
| 10×10 m @ 2.5 cm, 2 % occupied | 720 | 73 µs | 102 µs |
| 30×30 m @ 5 cm, sparse | 720 | 181 µs | 217 µs |

Cost scales with *free space traversed*, not with map area — a cluttered environment is
cheaper because beams terminate early, and an empty costmap is the worst case. Expect
roughly 3–4× these numbers on a Jetson/ARM class core, so a 20 Hz scan from a typical local
costmap still costs well under 1 % of a core. Enable
`-DCOSTMAP_TO_LASERSCAN_OPENMP=ON` only well beyond these sizes; below ~200 µs the thread
wake-up costs more than the walk and adds jitter.

Correctness of the kernel was checked against a dense-sampling reference caster over 28 800
randomised beams (random poses, yaws, obstacle fields, sensor origins inside and outside the
raster): zero hit/no-return classification mismatches, and the worst range deviation equalled
the reference's own sampling step.

**Never build this with `-ffast-math`.** The traversal uses `+inf` to mean "this axis is
never crossed" and the scan contract uses `inf` to mean "no return". `-ffast-math` assumes
neither exists.

## Parameters that actually matter

* **`lethal_cost_threshold`** (default `254`) — `254` stops only on lethal cells, so the
  scan geometry matches the real obstacles. Use `253` to also stop on the inscribed ring,
  which makes obstacles appear inflated by the inscribed radius. Feeding `253` into a
  consumer that already does footprint collision checking double-counts the robot radius.
* **`target_frame`** (default `base_link`) — must match the costmap's `robot_base_frame`.
  Roll/pitch of that transform is ignored (planar projection).
* **`costmap_transient_local_qos`** (default `true`) — Nav2 publishes the costmap
  `TRANSIENT_LOCAL`. A `VOLATILE` subscriber on a non-rolling costmap can miss the base
  raster forever and only ever see updates. Check with `ros2 topic info -v <topic>` if no
  scan appears. Note that rclcpp forbids intra-process comms on a transient-local endpoint,
  so the node opts out of intra-process for that subscription automatically.
* **`publish_rate`** (default `0.0`) — `0` publishes on every costmap message (lowest
  latency, scan stamp = costmap stamp). Setting it above the costmap rate re-casts the
  cached raster against the *latest* TF, which gives a smoother scan while the robot moves;
  the scan is then stamped with the transform's own time.
* **`range_min`** (default `0.15`) — obstacles nearer than this are *skipped*, and the beam
  continues past them. This is what keeps an inflated cell under the footprint from
  collapsing the whole scan to zero.
* **`range_max`** (default `0.0` → auto) — auto derives half the costmap diagonal on the
  first message and latches it, so consumers see a constant `range_max`.

Beam count is `round((angle_max - angle_min) / angle_increment)`; beam *i* is at
`angle_min + i * angle_increment`, so a full circle produces 720 beams at 0.5° with no
duplicated wrap-around beam.

## Build and run

```bash
colcon build --packages-select costmap_to_laserscan --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select costmap_to_laserscan   # 12 kernel unit tests

ros2 launch costmap_to_laserscan costmap_to_laserscan.launch.py
# or composed into a container (set costmap_transient_local_qos:=false to get the
# intra-process path, only if the costmap publisher is volatile):
ros2 launch costmap_to_laserscan costmap_to_laserscan.launch.py use_composition:=true
```

Watch the per-cycle cast time:

```bash
ros2 run costmap_to_laserscan costmap_to_laserscan --ros-args --log-level debug
```

## Caveats

* `nav2_msgs/Costmap` (`input_type: costmap_raw`) has no incremental-update path here;
  full messages only. The `OccupancyGrid` path supports `map_msgs/OccupancyGridUpdate`.
* If you feed this scan back into the same costmap's obstacle layer you create a positive
  feedback loop. Use it for a *different* consumer (collision monitor, a second costmap,
  a legacy 2D consumer), not the costmap it came from.
* The scan is a planar slice of an already-flattened 2D raster, so anything the costmap
  layers dropped (height filtering, voxel decay) is already gone.