// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#ifndef COSTMAP_TO_LASERSCAN__COSTMAP_TO_LASERSCAN_NODE_HPP_
#define COSTMAP_TO_LASERSCAN__COSTMAP_TO_LASERSCAN_NODE_HPP_

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace costmap_to_laserscan
{

/// 256-entry lookup table: blocked[raw_cell_byte] != 0  =>  the beam stops there.
/// Using a table instead of comparisons keeps the inner loop branchless and makes the
/// int8 (OccupancyGrid) and uint8 (nav2_msgs/Costmap) encodings a single code path.
using BlockedLut = std::array<uint8_t, 256>;

/// Non-owning view of the raster being traversed.
struct GridView
{
  const uint8_t * data{nullptr};
  uint32_t width{0};
  uint32_t height{0};
};

/// Everything the kernel needs, pre-reduced to cell units and float precision.
struct RayCastConfig
{
  float origin_x_cells{0.0F};   //!< sensor position in continuous cell coordinates
  float origin_y_cells{0.0F};
  float base_cos{1.0F};         //!< rotation of the scan frame inside the grid frame
  float base_sin{0.0F};
  float t_min_cells{0.0F};      //!< range_min / resolution
  float t_max_cells{0.0F};      //!< range_max / resolution
  float resolution{0.05F};      //!< metres per cell
  float no_return{std::numeric_limits<float>::infinity()};
};

/// Amanatides-Woo grid traversal, one exact DDA walk per beam.
///
/// Reported range is the parametric distance at which the beam *enters* the first
/// blocked cell, so it is sub-cell accurate rather than quantised to cell centres.
/// Beams that leave the raster, exceed t_max_cells or find nothing get `no_return`.
///
/// noexcept + no allocation + no shared mutable state: safe to parallelise over beams.
void castScan(
  const GridView & grid,
  const BlockedLut & blocked,
  const RayCastConfig & cfg,
  const float * beam_cos,
  const float * beam_sin,
  float * ranges,
  std::size_t num_beams) noexcept;

/// nav2_costmap_2d::Costmap2DPublisher forward map (cost 0..255 -> occupancy -1..100).
int costToOccupancy(int cost) noexcept;

/// Runtime-tunable parameters, applied atomically between cycles.
struct Tunables
{
  double angle_min{-M_PI};
  double angle_max{M_PI};
  double angle_increment{M_PI / 360.0};   //!< 0.5 deg -> 720 beams over a full circle
  double range_min{0.15};
  double range_max{0.0};                  //!< <= 0 -> derived from the costmap extent
  int lethal_cost_threshold{254};         //!< 254 = lethal only, 253 = include inscribed
  bool unknown_is_obstacle{false};
  bool use_inf{true};
  double inf_epsilon{1.0};
};

/**
 * Projects a Nav2 costmap into a sensor_msgs/LaserScan expressed in the robot base frame.
 *
 * Threading contract: the costmap subscription, the update subscription and the publish
 * timer all live in one MutuallyExclusive callback group, so all grid state is touched by
 * exactly one thread at a time and needs no locking - even under a MultiThreadedExecutor.
 * The only cross-thread state is the parameter staging buffer (mutex + atomic flag).
 */
class CostmapToLaserScanNode : public rclcpp::Node
{
public:
  explicit CostmapToLaserScanNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- callbacks -----------------------------------------------------------------
  void onOccupancyGrid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void onCostmapRaw(nav2_msgs::msg::Costmap::ConstSharedPtr msg);
  void onOccupancyGridUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr msg);
  void onTimer();
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  // ---- pipeline ------------------------------------------------------------------
  void ingestGrid(
    const std_msgs::msg::Header & header, double resolution, uint32_t width, uint32_t height,
    const geometry_msgs::msg::Pose & origin, const uint8_t * data, std::size_t size);
  void publishScan(bool from_timer);
  rclcpp::Time stampOrNow(const builtin_interfaces::msg::Time & stamp);

  // ---- (re)configuration ---------------------------------------------------------
  Tunables declareTunables();
  static bool validate(const Tunables & t, std::string & reason);
  void applyTunablesIfDirty();
  void rebuildBeamTables();
  void rebuildBlockedLut();
  void resolveRangeMax();

  // ---- interfaces ----------------------------------------------------------------
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr raw_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ---- static configuration ------------------------------------------------------
  std::string target_frame_{"base_link"};
  double transform_tolerance_{0.1};
  double costmap_timeout_{0.5};
  double publish_rate_{0.0};
  bool raw_input_{false};
  bool skip_when_no_subscribers_{true};

  // ---- costmap mirror (single-threaded by construction) --------------------------
  std::vector<uint8_t> grid_;
  uint32_t width_{0};
  uint32_t height_{0};
  float resolution_{0.0F};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  std::string grid_frame_;
  rclcpp::Time grid_stamp_{0, 0, RCL_ROS_TIME};
  bool have_grid_{false};

  // ---- derived scan geometry -----------------------------------------------------
  std::vector<float> beam_cos_;
  std::vector<float> beam_sin_;
  std::size_t num_beams_{0};
  BlockedLut blocked_lut_{};
  double range_max_{0.0};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_scan_stamp_{false};
  double ema_cast_us_{-1.0};

  // ---- parameter staging (cross-thread) ------------------------------------------
  Tunables active_;
  std::optional<Tunables> pending_;
  std::mutex tunables_mutex_;
  std::atomic<bool> params_dirty_{false};
};

}  // namespace costmap_to_laserscan

#endif  // COSTMAP_TO_LASERSCAN__COSTMAP_TO_LASERSCAN_NODE_HPP_