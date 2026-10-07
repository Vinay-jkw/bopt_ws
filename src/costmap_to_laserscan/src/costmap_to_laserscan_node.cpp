// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "costmap_to_laserscan/costmap_to_laserscan_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
// Defines tf2::fromMsg for geometry_msgs types; tf2/utils.h only declares it, so
// tf2::getYaw() below links only if this is present.
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace costmap_to_laserscan
{
namespace
{

constexpr float kDirEps = 1e-6F;
constexpr float kInf = std::numeric_limits<float>::infinity();

std::string stripLeadingSlash(std::string frame)
{
  if (!frame.empty() && frame.front() == '/') {
    frame.erase(0, 1);
  }
  return frame;
}

}  // namespace

int costToOccupancy(int cost) noexcept
{
  if (cost <= 0) {return 0;}
  if (cost >= 254) {return 100;}
  if (cost == 253) {return 99;}
  return 1 + (97 * (cost - 1)) / 251;
}

// ---------------------------------------------------------------------------------------
// Ray-cast kernel
// ---------------------------------------------------------------------------------------
void castScan(
  const GridView & grid,
  const BlockedLut & blocked,
  const RayCastConfig & cfg,
  const float * beam_cos,
  const float * beam_sin,
  float * ranges,
  std::size_t num_beams) noexcept
{
  const uint8_t * __restrict const data = grid.data;
  const float wf = static_cast<float>(grid.width);
  const float hf = static_cast<float>(grid.height);
  const std::ptrdiff_t stride = static_cast<std::ptrdiff_t>(grid.width);
  const std::ptrdiff_t n = static_cast<std::ptrdiff_t>(num_beams);

#ifdef COSTMAP_TO_LASERSCAN_OPENMP
  #pragma omp parallel for schedule(static)
#endif
  for (std::ptrdiff_t i = 0; i < n; ++i) {
    // Beam direction in grid axes. One 2x2 rotation of a pre-tabulated unit vector:
    // 4 mults instead of a sin/cos pair per beam per cycle.
    const float ca = beam_cos[i];
    const float sa = beam_sin[i];
    const float dx = cfg.base_cos * ca - cfg.base_sin * sa;
    const float dy = cfg.base_sin * ca + cfg.base_cos * sa;

    // --- clip the beam against the raster (slab test) --------------------------------
    // Also handles a sensor origin that sits outside the costmap (offset mounts,
    // global costmaps, robot momentarily outside a non-rolling window).
    float t_enter = 0.0F;
    float t_exit = cfg.t_max_cells;
    bool valid = true;

    if (std::fabs(dx) < kDirEps) {
      valid = (cfg.origin_x_cells >= 0.0F) && (cfg.origin_x_cells < wf);
    } else {
      const float inv = 1.0F / dx;
      float ta = (0.0F - cfg.origin_x_cells) * inv;
      float tb = (wf - cfg.origin_x_cells) * inv;
      if (ta > tb) {std::swap(ta, tb);}
      t_enter = std::max(t_enter, ta);
      t_exit = std::min(t_exit, tb);
    }
    if (valid) {
      if (std::fabs(dy) < kDirEps) {
        valid = (cfg.origin_y_cells >= 0.0F) && (cfg.origin_y_cells < hf);
      } else {
        const float inv = 1.0F / dy;
        float ta = (0.0F - cfg.origin_y_cells) * inv;
        float tb = (hf - cfg.origin_y_cells) * inv;
        if (ta > tb) {std::swap(ta, tb);}
        t_enter = std::max(t_enter, ta);
        t_exit = std::min(t_exit, tb);
      }
    }

    // Starting at range_min both enforces the minimum range and skips the cells under
    // the footprint entirely - it is a shortcut, not just a filter.
    const float t_start = std::max(t_enter, cfg.t_min_cells);
    if (!valid || t_start >= t_exit) {
      ranges[i] = cfg.no_return;
      continue;
    }

    const float px = cfg.origin_x_cells + dx * t_start;
    const float py = cfg.origin_y_cells + dy * t_start;

    int32_t ix = static_cast<int32_t>(std::floor(px));
    int32_t iy = static_cast<int32_t>(std::floor(py));
    ix = std::min(std::max(ix, 0), static_cast<int32_t>(grid.width) - 1);
    iy = std::min(std::max(iy, 0), static_cast<int32_t>(grid.height) - 1);

    const int32_t step_x = (dx >= 0.0F) ? 1 : -1;
    const int32_t step_y = (dy >= 0.0F) ? 1 : -1;

    // 1/|d| is the parametric cost of crossing one full cell on that axis.
    // An exactly axis-aligned beam gets +inf, which makes the comparison below always
    // pick the other axis. This is why -ffast-math must never be enabled here.
    const float inv_ax = (std::fabs(dx) < kDirEps) ? kInf : 1.0F / std::fabs(dx);
    const float inv_ay = (std::fabs(dy) < kDirEps) ? kInf : 1.0F / std::fabs(dy);

    float t_next_x = (inv_ax == kInf) ?
      kInf :
      t_start + ((dx >= 0.0F) ?
      (static_cast<float>(ix + 1) - px) :
      (px - static_cast<float>(ix))) * inv_ax;
    float t_next_y = (inv_ay == kInf) ?
      kInf :
      t_start + ((dy >= 0.0F) ?
      (static_cast<float>(iy + 1) - py) :
      (py - static_cast<float>(iy))) * inv_ay;

    // The linear index is carried incrementally: no multiply inside the walk.
    std::ptrdiff_t idx = static_cast<std::ptrdiff_t>(iy) * stride + ix;
    const std::ptrdiff_t idx_step_y = static_cast<std::ptrdiff_t>(step_y) * stride;

    float t = t_start;
    float hit = cfg.no_return;

    for (;;) {
      if (blocked[data[idx]] != 0U) {
        hit = t * cfg.resolution;   // t is the entry distance into this cell
        break;
      }
      if (t_next_x < t_next_y) {
        t = t_next_x;
        if (t >= t_exit) {break;}
        ix += step_x;
        if (static_cast<uint32_t>(ix) >= grid.width) {break;}   // one compare, both bounds
        idx += step_x;
        t_next_x += inv_ax;
      } else {
        t = t_next_y;
        if (t >= t_exit) {break;}
        iy += step_y;
        if (static_cast<uint32_t>(iy) >= grid.height) {break;}
        idx += idx_step_y;
        t_next_y += inv_ay;
      }
    }

    ranges[i] = hit;
  }
}

// ---------------------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------------------
CostmapToLaserScanNode::CostmapToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("costmap_to_laserscan", options)
{
  // ---- static parameters ------------------------------------------------------------
  const std::string costmap_topic =
    declare_parameter<std::string>("costmap_topic", "local_costmap/costmap");
  std::string updates_topic =
    declare_parameter<std::string>("costmap_updates_topic", "");
  const std::string input_type =
    declare_parameter<std::string>("input_type", "occupancy_grid");
  const std::string scan_topic =
    declare_parameter<std::string>("scan_topic", "costmap_scan");
  const bool use_updates = declare_parameter<bool>("use_costmap_updates", true);
  const bool transient_local = declare_parameter<bool>("costmap_transient_local_qos", true);
  const bool best_effort_scan = declare_parameter<bool>("scan_best_effort_qos", true);

  target_frame_ = stripLeadingSlash(declare_parameter<std::string>("target_frame", "base_link"));
  transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
  costmap_timeout_ = declare_parameter<double>("costmap_timeout", 0.5);
  publish_rate_ = declare_parameter<double>("publish_rate", 0.0);
  skip_when_no_subscribers_ = declare_parameter<bool>("skip_when_no_subscribers", true);

  raw_input_ = (input_type == "costmap_raw");
  if (!raw_input_ && input_type != "occupancy_grid") {
    RCLCPP_WARN(
      get_logger(), "Unknown input_type '%s', falling back to 'occupancy_grid'.",
      input_type.c_str());
  }
  if (updates_topic.empty()) {
    updates_topic = costmap_topic + "_updates";
  }

  // ---- tunables ----------------------------------------------------------------------
  active_ = declareTunables();
  std::string reason;
  if (!validate(active_, reason)) {
    throw rclcpp::exceptions::InvalidParametersException("costmap_to_laserscan: " + reason);
  }
  rebuildBeamTables();
  rebuildBlockedLut();
  range_max_ = active_.range_max;   // may be 0 -> resolved from the first costmap

  // ---- TF ------------------------------------------------------------------------
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // The listener runs its own executor thread, which is what makes a blocking
  // lookupTransform() with a timeout legal from inside our callback.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_buffer_->setUsingDedicatedThread(true);

  // ---- interfaces --------------------------------------------------------------------
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
  costmap_qos.reliable();
  if (transient_local) {
    costmap_qos.transient_local();
  }

  rclcpp::SubscriptionOptions costmap_sub_opts;
  costmap_sub_opts.callback_group = cb_group_;
  // rclcpp refuses intra-process on a transient-local endpoint; opt out explicitly so the
  // node still composes cleanly into a container with intra-process comms enabled.
  costmap_sub_opts.use_intra_process_comm = transient_local ?
    rclcpp::IntraProcessSetting::Disable :
    rclcpp::IntraProcessSetting::NodeDefault;

  rclcpp::SubscriptionOptions volatile_sub_opts;
  volatile_sub_opts.callback_group = cb_group_;

  using std::placeholders::_1;
  if (raw_input_) {
    raw_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic, costmap_qos,
      std::bind(&CostmapToLaserScanNode::onCostmapRaw, this, _1), costmap_sub_opts);
  } else {
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, costmap_qos,
      std::bind(&CostmapToLaserScanNode::onOccupancyGrid, this, _1), costmap_sub_opts);
    if (use_updates) {
      update_sub_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        updates_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&CostmapToLaserScanNode::onOccupancyGridUpdate, this, _1),
        volatile_sub_opts);
    }
  }

  rclcpp::QoS scan_qos = best_effort_scan ?
    rclcpp::QoS(rclcpp::SensorDataQoS()) :
    rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, scan_qos);

  if (publish_rate_ > 0.0) {
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate_),
      std::bind(&CostmapToLaserScanNode::onTimer, this), cb_group_);
  }

  param_cb_ = add_on_set_parameters_callback(
    std::bind(&CostmapToLaserScanNode::onSetParameters, this, _1));

  RCLCPP_INFO(
    get_logger(),
    "costmap_to_laserscan: %s -> %s | frame '%s' | %zu beams | mode %s",
    costmap_topic.c_str(), scan_topic.c_str(), target_frame_.c_str(), num_beams_,
    publish_rate_ > 0.0 ? "timer" : "event-driven");
}

// ---------------------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------------------
Tunables CostmapToLaserScanNode::declareTunables()
{
  Tunables t;
  t.angle_min = declare_parameter<double>("angle_min", t.angle_min);
  t.angle_max = declare_parameter<double>("angle_max", t.angle_max);
  t.angle_increment = declare_parameter<double>("angle_increment", t.angle_increment);
  t.range_min = declare_parameter<double>("range_min", t.range_min);
  t.range_max = declare_parameter<double>("range_max", t.range_max);
  t.lethal_cost_threshold = declare_parameter<int>("lethal_cost_threshold",
      t.lethal_cost_threshold);
  t.unknown_is_obstacle = declare_parameter<bool>("unknown_is_obstacle", t.unknown_is_obstacle);
  t.use_inf = declare_parameter<bool>("use_inf", t.use_inf);
  t.inf_epsilon = declare_parameter<double>("inf_epsilon", t.inf_epsilon);
  return t;
}

bool CostmapToLaserScanNode::validate(const Tunables & t, std::string & reason)
{
  if (!(t.angle_max > t.angle_min)) {
    reason = "angle_max must be greater than angle_min";
    return false;
  }
  if (!(t.angle_increment > 0.0)) {
    reason = "angle_increment must be positive";
    return false;
  }
  if ((t.angle_max - t.angle_min) / t.angle_increment > 100000.0) {
    reason = "angle_increment too small for the requested span (> 100000 beams)";
    return false;
  }
  if (t.range_min < 0.0) {
    reason = "range_min must be >= 0";
    return false;
  }
  if (t.range_max > 0.0 && t.range_max <= t.range_min) {
    reason = "range_max must be greater than range_min";
    return false;
  }
  if (t.lethal_cost_threshold < 1 || t.lethal_cost_threshold > 254) {
    reason = "lethal_cost_threshold must be in [1, 254]";
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult
CostmapToLaserScanNode::onSetParameters(const std::vector<rclcpp::Parameter> & params)
{
  Tunables candidate;
  {
    std::lock_guard<std::mutex> lock(tunables_mutex_);
    candidate = pending_.has_value() ? *pending_ : active_;
  }

  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "angle_min") {candidate.angle_min = p.as_double();} else if (n == "angle_max") {
      candidate.angle_max = p.as_double();
    } else if (n == "angle_increment") {
      candidate.angle_increment = p.as_double();
    } else if (n == "range_min") {
      candidate.range_min = p.as_double();
    } else if (n == "range_max") {
      candidate.range_max = p.as_double();
    } else if (n == "lethal_cost_threshold") {
      candidate.lethal_cost_threshold = static_cast<int>(p.as_int());
    } else if (n == "unknown_is_obstacle") {
      candidate.unknown_is_obstacle = p.as_bool();
    } else if (n == "use_inf") {
      candidate.use_inf = p.as_bool();
    } else if (n == "inf_epsilon") {
      candidate.inf_epsilon = p.as_double();
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  std::string reason;
  result.successful = validate(candidate, reason);
  result.reason = result.successful ? "ok" : reason;

  if (result.successful) {
    // Stage only. The values are swapped in between cycles so a reconfiguration can
    // never tear a scan that is mid-computation.
    std::lock_guard<std::mutex> lock(tunables_mutex_);
    pending_ = candidate;
    params_dirty_.store(true, std::memory_order_release);
  }
  return result;
}

void CostmapToLaserScanNode::applyTunablesIfDirty()
{
  if (!params_dirty_.load(std::memory_order_acquire)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(tunables_mutex_);
    if (pending_.has_value()) {
      active_ = *pending_;
      pending_.reset();
    }
    params_dirty_.store(false, std::memory_order_release);
  }
  rebuildBeamTables();
  rebuildBlockedLut();
  resolveRangeMax();
  RCLCPP_INFO(get_logger(), "Reconfigured: %zu beams, range_max %.3f m", num_beams_, range_max_);
}

void CostmapToLaserScanNode::rebuildBeamTables()
{
  const double span = active_.angle_max - active_.angle_min;
  // Round rather than ceil: a full circle with an evenly dividing increment must not
  // pick up a 721st beam because of a 1-ulp overshoot.
  num_beams_ = static_cast<std::size_t>(std::max(1L, std::lround(span / active_.angle_increment)));

  beam_cos_.resize(num_beams_);
  beam_sin_.resize(num_beams_);
  for (std::size_t i = 0; i < num_beams_; ++i) {
    const double a = active_.angle_min + static_cast<double>(i) * active_.angle_increment;
    beam_cos_[i] = static_cast<float>(std::cos(a));
    beam_sin_[i] = static_cast<float>(std::sin(a));
  }
}

void CostmapToLaserScanNode::rebuildBlockedLut()
{
  const int thr = active_.lethal_cost_threshold;
  const uint8_t unknown = active_.unknown_is_obstacle ? 1U : 0U;

  if (raw_input_) {
    // nav2_msgs/Costmap: raw 0..252 cost, 253 inscribed, 254 lethal, 255 unknown.
    for (int v = 0; v < 256; ++v) {
      blocked_lut_[static_cast<std::size_t>(v)] =
        (v == 255) ? unknown : ((v >= thr) ? 1U : 0U);
    }
  } else {
    // nav_msgs/OccupancyGrid: int8, -1 unknown, 0..100 occupancy. Index the table with the
    // raw byte so -1 lands on 255 and the kernel stays encoding-agnostic.
    const int occ_thr = costToOccupancy(thr);
    for (int v = 0; v < 256; ++v) {
      const int s = static_cast<int>(static_cast<int8_t>(v));
      blocked_lut_[static_cast<std::size_t>(v)] =
        (s < 0 || s > 100) ? unknown : ((s >= occ_thr) ? 1U : 0U);
    }
  }
}

void CostmapToLaserScanNode::resolveRangeMax()
{
  if (active_.range_max > 0.0) {
    range_max_ = active_.range_max;
    return;
  }
  if (!have_grid_) {
    return;
  }
  // Half the raster diagonal: the furthest a beam can travel from a centred sensor.
  const double w = static_cast<double>(width_) * static_cast<double>(resolution_);
  const double h = static_cast<double>(height_) * static_cast<double>(resolution_);
  const double derived = 0.5 * std::hypot(w, h);
  if (std::fabs(derived - range_max_) > 1e-6) {
    range_max_ = derived;
    RCLCPP_INFO(get_logger(), "range_max auto-derived from costmap extent: %.3f m", range_max_);
  }
}

// ---------------------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------------------
rclcpp::Time CostmapToLaserScanNode::stampOrNow(const builtin_interfaces::msg::Time & stamp)
{
  const rclcpp::Time t(stamp, RCL_ROS_TIME);
  if (t.nanoseconds() == 0) {
    // nav2's Costmap2DPublisher emits OccupancyGridUpdate with a default-constructed
    // (zero) stamp. Substitute arrival time rather than declaring the raster ancient.
    RCLCPP_INFO_ONCE(
      get_logger(),
      "Publisher sends unstamped costmap headers; using arrival time instead.");
    return now();
  }
  return t;
}

void CostmapToLaserScanNode::ingestGrid(
  const std_msgs::msg::Header & header, double resolution, uint32_t width, uint32_t height,
  const geometry_msgs::msg::Pose & origin, const uint8_t * data, std::size_t size)
{
  const std::size_t expected = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (width == 0 || height == 0 || resolution <= 0.0 || size != expected) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Malformed costmap (%ux%u, res %.4f, %zu cells) - ignored.",
      width, height, resolution, size);
    return;
  }

  const bool geometry_changed = (width != width_) || (height != height_) ||
    (std::fabs(static_cast<double>(resolution_) - resolution) > 1e-9);

  if (grid_.size() != size) {
    grid_.resize(size);
  }
  std::memcpy(grid_.data(), data, size);

  width_ = width;
  height_ = height;
  resolution_ = static_cast<float>(resolution);
  origin_x_ = origin.position.x;
  origin_y_ = origin.position.y;
  origin_yaw_ = tf2::getYaw(origin.orientation);
  grid_frame_ = stripLeadingSlash(header.frame_id);
  grid_stamp_ = stampOrNow(header.stamp);
  have_grid_ = true;

  if (geometry_changed) {
    resolveRangeMax();
  }
}

void CostmapToLaserScanNode::onOccupancyGrid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  ingestGrid(
    msg->header, msg->info.resolution, msg->info.width, msg->info.height, msg->info.origin,
    reinterpret_cast<const uint8_t *>(msg->data.data()), msg->data.size());
  if (publish_rate_ <= 0.0) {
    publishScan(false);
  }
}

void CostmapToLaserScanNode::onCostmapRaw(nav2_msgs::msg::Costmap::ConstSharedPtr msg)
{
  ingestGrid(
    msg->header, msg->metadata.resolution, msg->metadata.size_x, msg->metadata.size_y,
    msg->metadata.origin, msg->data.data(), msg->data.size());
  if (publish_rate_ <= 0.0) {
    publishScan(false);
  }
}

void CostmapToLaserScanNode::onOccupancyGridUpdate(
  map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr msg)
{
  if (!have_grid_) {
    return;   // no base raster to patch yet
  }
  const std::string update_frame = stripLeadingSlash(msg->header.frame_id);
  if (!update_frame.empty() && update_frame != grid_frame_) {
    return;
  }
  const std::size_t patch = static_cast<std::size_t>(msg->width) *
    static_cast<std::size_t>(msg->height);
  if (msg->data.size() != patch ||
    static_cast<uint64_t>(msg->x) + msg->width > width_ ||
    static_cast<uint64_t>(msg->y) + msg->height > height_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Costmap update does not fit the mirror - ignored.");
    return;
  }

  const uint8_t * const src = reinterpret_cast<const uint8_t *>(msg->data.data());
  for (uint32_t row = 0; row < msg->height; ++row) {
    std::memcpy(
      grid_.data() + static_cast<std::size_t>(msg->y + row) * width_ + msg->x,
      src + static_cast<std::size_t>(row) * msg->width,
      msg->width);
  }
  grid_stamp_ = stampOrNow(msg->header.stamp);

  if (publish_rate_ <= 0.0) {
    publishScan(false);
  }
}

void CostmapToLaserScanNode::onTimer()
{
  publishScan(true);
}

// ---------------------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------------------
void CostmapToLaserScanNode::publishScan(bool from_timer)
{
  applyTunablesIfDirty();

  if (!have_grid_ || num_beams_ == 0) {
    return;
  }
  if (skip_when_no_subscribers_ &&
    scan_pub_->get_subscription_count() == 0 &&
    scan_pub_->get_intra_process_subscription_count() == 0)
  {
    return;   // do not pay for a ray-cast nobody consumes
  }

  const rclcpp::Time now_ros = now();
  if (costmap_timeout_ > 0.0 && (now_ros - grid_stamp_).seconds() > costmap_timeout_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Costmap is %.2f s old (> %.2f s) - not publishing a stale scan.",
      (now_ros - grid_stamp_).seconds(), costmap_timeout_);
    return;
  }

  // ---- sensor pose inside the raster --------------------------------------------------
  double sx = 0.0;
  double sy = 0.0;
  double syaw = 0.0;
  builtin_interfaces::msg::Time stamp = grid_stamp_;

  if (target_frame_ != grid_frame_) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      if (from_timer) {
        // Latest available pose gives a fresher scan than the costmap's own stamp; the
        // transform's own stamp is then the honest acquisition time.
        tf = tf_buffer_->lookupTransform(grid_frame_, target_frame_, tf2::TimePointZero);
        stamp = tf.header.stamp;
      } else {
        tf = tf_buffer_->lookupTransform(
          grid_frame_, target_frame_, grid_stamp_,
          rclcpp::Duration::from_seconds(transform_tolerance_));
        stamp = grid_stamp_;
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF %s -> %s unavailable: %s",
        grid_frame_.c_str(), target_frame_.c_str(), ex.what());
      return;
    }
    sx = tf.transform.translation.x;
    sy = tf.transform.translation.y;
    syaw = tf2::getYaw(tf.transform.rotation);   // planar projection: roll/pitch ignored
  } else if (from_timer) {
    stamp = now_ros;
  }

  // ---- reduce everything to cell units ------------------------------------------------
  const double c0 = std::cos(origin_yaw_);
  const double s0 = std::sin(origin_yaw_);
  const double ddx = sx - origin_x_;
  const double ddy = sy - origin_y_;
  const double inv_res = 1.0 / static_cast<double>(resolution_);

  RayCastConfig cfg;
  cfg.origin_x_cells = static_cast<float>((ddx * c0 + ddy * s0) * inv_res);
  cfg.origin_y_cells = static_cast<float>((-ddx * s0 + ddy * c0) * inv_res);
  const double base_yaw = syaw - origin_yaw_;
  cfg.base_cos = static_cast<float>(std::cos(base_yaw));
  cfg.base_sin = static_cast<float>(std::sin(base_yaw));
  cfg.t_min_cells = static_cast<float>(active_.range_min * inv_res);
  cfg.t_max_cells = static_cast<float>(range_max_ * inv_res);
  cfg.resolution = resolution_;
  cfg.no_return = active_.use_inf ?
    kInf :
    static_cast<float>(range_max_ + active_.inf_epsilon);

  // ---- build and fill the message -----------------------------------------------------
  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = target_frame_;
  scan->header.stamp = stamp;
  scan->angle_min = static_cast<float>(active_.angle_min);
  scan->angle_max = static_cast<float>(active_.angle_max);
  scan->angle_increment = static_cast<float>(active_.angle_increment);
  scan->time_increment = 0.0F;   // the whole raster is a single instant
  scan->range_min = static_cast<float>(active_.range_min);
  scan->range_max = static_cast<float>(range_max_);

  const rclcpp::Time scan_stamp(stamp, RCL_ROS_TIME);
  double dt = 0.0;
  if (have_last_scan_stamp_) {
    dt = (scan_stamp - last_scan_stamp_).seconds();
  }
  scan->scan_time = static_cast<float>((dt > 0.0 && dt < 10.0) ? dt : 0.0);
  last_scan_stamp_ = scan_stamp;
  have_last_scan_stamp_ = true;

  scan->ranges.resize(num_beams_);

  const auto t0 = std::chrono::steady_clock::now();
  castScan(
    GridView{grid_.data(), width_, height_}, blocked_lut_, cfg,
    beam_cos_.data(), beam_sin_.data(), scan->ranges.data(), num_beams_);
  const double us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - t0).count();
  ema_cast_us_ = (ema_cast_us_ < 0.0) ? us : (0.9 * ema_cast_us_ + 0.1 * us);

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "cast %zu beams over %ux%u cells in %.1f us (ema %.1f us)",
    num_beams_, width_, height_, us, ema_cast_us_);

  // std::move keeps the intra-process path zero-copy when composed in a container.
  scan_pub_->publish(std::move(scan));
}

}  // namespace costmap_to_laserscan

RCLCPP_COMPONENTS_REGISTER_NODE(costmap_to_laserscan::CostmapToLaserScanNode)