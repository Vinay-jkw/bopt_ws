// Core orchestrator — wires all safety components together, manages
// subscriptions, and drives the safety status pipeline.
#include "safety_demo/safety_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

namespace safety_demo {

SafetyNode::SafetyNode(const std::vector<std::string> &lidar_topics)
    : Node("safety_node"), safety_turn_off_(false), pickdrop_mode_(false),
      robot_in_parking_(false) {

  RCLCPP_INFO(get_logger(), "Initializing Safety Node for BOPT");
  RCLCPP_INFO(get_logger(), "Safety geometry frame: %s (TF authoritative)",
              base_frame_.c_str());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

  // Initialize database manager and load data
  try {
    // Path is relative to workspace root — must be launched from there
    db_manager_ = std::make_unique<DatabaseManager>(
        ament_index_cpp::get_package_share_directory("safety_demo") +
        "/config/bopt_ws_body.db");
    policies_ = db_manager_->loadPolicies();
    lidar_configs_ = db_manager_->loadLiDARConfigs();
    RCLCPP_INFO(get_logger(), "Loaded %zu BOPT LiDAR configurations",
                lidar_configs_.size());

    for (const auto &[topic, config] : lidar_configs_) {
      RCLCPP_INFO(get_logger(),
                  "BOPT LiDAR %d | topic=%s | shape=%s | description=%s",
                  config.lidar_id, topic.c_str(), config.shape_type.c_str(),
                  config.description.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize database: %s", e.what());
    throw;
  }

  // Initialize components
  zone_detector_ = std::make_unique<SafetyZoneDetector>();
  velocity_controller_ = std::make_unique<VelocityController>(this);
  viz_publisher_ =
      std::make_unique<VisualizationPublisher>(this, lidar_configs_);
  error_handler_ = std::make_unique<ErrorHandler>(this, lidar_topics);
  policy_selector_ = std::make_unique<PolicySelector>(policies_);

  // Initialize status map
  for (const auto &[topic, config] : lidar_configs_) {
    lidar_status_map_[config.lidar_id] = "safe";
  }

  // Lambda captures topic by value so each callback knows which sensor
  // triggered it
  for (const auto &topic : lidar_topics) {
    lidar_subscriptions_.push_back(
        create_subscription<sensor_msgs::msg::LaserScan>(
            topic, 10,
            [this, topic](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
              lidarCallback(msg, topic);
            }));
  }

  // Velocity comes from odometry (actual measured speed), not /cmd_vel
  // (commanded speed)
  odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&SafetyNode::odometryCallback, this, std::placeholders::_1));

  // Forward incoming velocity commands to VelocityController so it has
  // something to publish
  cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        velocity_controller_->updateCmdVel(msg);
      });

  velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "/velocity", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        velocity_controller_->updateVelocity(msg);
      });

  bopt_cmd_subscriber_ =
      create_subscription<bopt_interfaces::msg::BoptCommandStamped>(
          "bopt/relay_cmd", 10,
          [this](
              const bopt_interfaces::msg::BoptCommandStamped::SharedPtr msg) {
            velocity_controller_->updateBoptCommand(msg);
          });

  safety_turnoff_subscriber_ = create_subscription<std_msgs::msg::String>(
      "safety_turnoff", 10,
      std::bind(&SafetyNode::safetyTurnoffCallback, this,
                std::placeholders::_1));

  // /byd/safety drives operational mode: "pickdrop", "normal", or "parking"
  pickdrop_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/byd/safety", 10,
      std::bind(&SafetyNode::pickdropCallback, this, std::placeholders::_1));

  mqtt_status_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/mqtt/status", 10,
      std::bind(&SafetyNode::mqttStatusCallback, this, std::placeholders::_1));

  // Published by the localization_watchdog package at 10 Hz.
  // "localization_broken" there is a hard stop; see
  // ErrorHandler::updateLocalizationStatus.
  localization_status_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/localization/status", 10,
      std::bind(&SafetyNode::localizationStatusCallback, this,
                std::placeholders::_1));

  safety_status_publisher_ =
      create_publisher<std_msgs::msg::String>("safety_status", 10);

  RCLCPP_INFO(get_logger(), "Safety Node initialized successfully");
}

void SafetyNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                               const std::string &topic) {
  // Update error handler timestamp
  error_handler_->updateLidarTimestamp(topic);

  // Process the LiDAR scan
  processLidarScan(msg, topic);
}

void SafetyNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(velocity_mutex_);

  // Round to 2dp to avoid jitter around zero affecting direction and policy
  // selection
  current_velocity_.linear.x =
      std::round(msg->twist.twist.linear.x * 100.0) / 100.0;
  current_velocity_.linear.y =
      std::round(msg->twist.twist.linear.y * 100.0) / 100.0;
  current_velocity_.linear.z =
      std::round(msg->twist.twist.linear.z * 100.0) / 100.0;
  current_velocity_.angular.x =
      std::round(msg->twist.twist.angular.x * 100.0) / 100.0;
  current_velocity_.angular.y =
      std::round(msg->twist.twist.angular.y * 100.0) / 100.0;
  current_velocity_.angular.z =
      std::round(msg->twist.twist.angular.z * 100.0) / 100.0;

  RCLCPP_INFO(get_logger(),
              ">>> [ODOM - policy speed]  linear.x=%.4f  angular.z=%.4f  (used "
              "for zone sizing only)",
              current_velocity_.linear.x, current_velocity_.angular.z);
}

void SafetyNode::safetyTurnoffCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  if (msg->data == "TurnOff") {
    safety_turn_off_ = true;
    RCLCPP_INFO(get_logger(), "Safety system turned off");
  } else {
    safety_turn_off_ = false;
    RCLCPP_INFO(get_logger(), "Safety system turned on");
  }
}

void SafetyNode::pickdropCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> parking_lock(parking_mutex_);
  robot_in_parking_ = (msg->data == "parking");
  if (robot_in_parking_) {
    // Parking mode is handled entirely by the policy selector — nothing else to
    // do here
    return;
  }

  std::lock_guard<std::mutex> lock(selected_policy_mutex_);
  std::lock_guard<std::mutex> pickdrop_lock(pickdrop_mutex_);

  if (msg->data == "pickdrop") {
    pickdrop_mode_ = true;
    RCLCPP_INFO(get_logger(), "Pickdrop mode activated");

    // LiDAR 3 is the fork-tip sensor; pre-load its pickdrop zone policy
    if (policies_.find(3) != policies_.end()) {
      const auto &lidar_policies = policies_[3];
      for (const auto &policy : lidar_policies) {
        if (policy.description == "Pickdrop Zone") {
          pickdrop_policy_ = policy;
          RCLCPP_INFO(get_logger(),
                      "Selected Pickdrop Policy ID: %d for LiDAR 3", policy.id);
          break;
        }
      }
    }
  } else if (msg->data == "normal") {
    if (pickdrop_mode_) {
      pickdrop_mode_ = false;
      pickdrop_policy_ = Policy(); // reset to default-constructed (id = 0)
      RCLCPP_INFO(get_logger(), "Pickdrop mode deactivated");
    }
  }
}

void SafetyNode::mqttStatusCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  error_handler_->updateMqttStatus(msg->data);
}

void SafetyNode::localizationStatusCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  error_handler_->updateLocalizationStatus(msg->data);

  // The safety pipeline is otherwise driven entirely by LiDAR scans. Re-running
  // it here means the stop lands within one watchdog tick even if the scans
  // have also stopped arriving — it re-reads the existing status map and
  // publishes, so it is safe to call from a second callback under the
  // single-threaded executor.
  updateOverallSafetyStatus();
}

void SafetyNode::processLidarScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg,
    const std::string &topic) {
  // Get LiDAR configuration
  auto config_it = lidar_configs_.find(topic);
  if (config_it == lidar_configs_.end()) {
    RCLCPP_WARN(get_logger(), "No configuration found for LiDAR topic: %s",
                topic.c_str());
    return;
  }
  const auto &config = config_it->second;
  int lidar_id = config.lidar_id;

  // Pick/drop does NOT disable LiDARs. The fork/front sensors remain active.
  // A future validated pickdrop policy can change zone dimensions, but a sensor
  // failure must never be converted into a false "safe" state.

  // Get current velocity and determine directions
  geometry_msgs::msg::Twist current_vel;
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    current_vel = current_velocity_;
  }

  auto [direction, angular_direction] = determineDirections();

  // Get parking status
  bool in_parking;
  {
    std::lock_guard<std::mutex> lock(parking_mutex_);
    in_parking = robot_in_parking_;
  }

  // Select policies
  bool in_pickdrop;
  {
    std::lock_guard<std::mutex> lock(pickdrop_mutex_);
    in_pickdrop = pickdrop_mode_;
  }

  auto [danger_policy, warning_policy] = policy_selector_->selectPolicies(
      std::abs(current_vel.linear.x), std::abs(current_vel.angular.z),
      direction, angular_direction, lidar_id, in_parking, in_pickdrop);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "LiDAR %d: mode=%s parking=%s speed=%.3f direction=%s | "
                       "Danger Policy=%d Warning Policy=%d",
                       lidar_id, in_pickdrop ? "PICKDROP" : "NORMAL",
                       in_parking ? "YES" : "NO",
                       std::abs(current_vel.linear.x), direction.c_str(),
                       danger_policy.id, warning_policy.id);

  // Log selected policies
  // if (danger_policy.id != -1) {
  //   RCLCPP_INFO(get_logger(),
  //               "LiDAR %d: Selected Danger Policy ID=%d, max_speed=%.2f",
  //               lidar_id, danger_policy.id, danger_policy.max_speed);
  // }
  // if (warning_policy.id != -1) {
  //   RCLCPP_INFO(get_logger(),
  //               "LiDAR %d: Selected Warning Policy ID=%d, max_speed=%.2f",
  //               lidar_id, warning_policy.id, warning_policy.max_speed);
  // }

  // Define polygons
  std::vector<std::pair<double, double>> danger_polygon;
  std::vector<std::pair<double, double>> warning_polygon;

  if (danger_policy.id != -1) {
    danger_polygon = zone_detector_->defineFieldPolygon(
        config.shape_type, danger_policy.danger_zone);
  }
  if (warning_policy.id != -1) {
    warning_polygon = zone_detector_->defineFieldPolygon(
        config.shape_type, warning_policy.warning_zone);
  }

  // Rotate polygon by 180 degrees for LiDAR 1
  if (lidar_id == 1) {
    for (auto &pt : danger_polygon) {
      pt.first = -pt.first;
      pt.second = -pt.second;
    }
    for (auto &pt : warning_polygon) {
      pt.first = -pt.first;
      pt.second = -pt.second;
    }
  }
  // Flip polygon along Y-axis for LiDAR 1 and LiDAR 2 (negate X coordinate)
  if (lidar_id == 1 || lidar_id == 2) {
    for (auto &pt : danger_polygon) {
      pt.first = -pt.first;
    }
    for (auto &pt : warning_polygon) {
      pt.first = -pt.first;
    }
  }
  if (lidar_id == 4) {
    for (auto &point : danger_polygon) {
      point.first -= 1.2;
    }

    for (auto &point : warning_polygon) {
      point.first -= 1.4;
    }
  }

  // Resolve the sensor frame once for this scan. TF is used only once per
  // message; every range point then uses the cached transform, which avoids
  // hundreds of TF buffer lookups per scan.
  geometry_msgs::msg::TransformStamped sensor_to_base;
  if (!lookupSensorToBase(msg, sensor_to_base)) {
    {
      std::lock_guard<std::mutex> lock(status_map_mutex_);
      lidar_status_map_[lidar_id] = "danger zone";
    }
    RCLCPP_ERROR(get_logger(),
                 "LiDAR %d (%s): missing TF %s -> %s; forcing safety danger",
                 lidar_id, topic.c_str(), msg->header.frame_id.c_str(),
                 base_frame_.c_str());
    viz_publisher_->publishZoneMarkers(lidar_id, topic, {}, {});
    updateOverallSafetyStatus();
    return;
  }

  // Transform the local safety fields to base_link for RViz/debugging. The
  // actual obstacle test also uses base_link coordinates so sensor offsets and
  // orientations are taken directly from URDF/TF, never from DB offsets.
  std::vector<std::pair<double, double>> danger_polygon_base =
      transformPolygonToBase(danger_polygon, sensor_to_base);
  std::vector<std::pair<double, double>> warning_polygon_base =
      transformPolygonToBase(warning_polygon, sensor_to_base);

  // Process LiDAR points in base_link using the single cached transform.
  int danger_count = 0;
  int warning_count = 0;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double range = msg->ranges[i];
    if (!std::isfinite(range) || range < msg->range_min ||
        range > msg->range_max || range <= 0.05) {
      continue;
    }

    double angle =
        msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    auto point_sensor = zone_detector_->polarToCartesian(range, angle);
    auto point_base = transformPointToBase(point_sensor, sensor_to_base);

    if (!danger_polygon_base.empty() &&
        zone_detector_->isPointInPolygon(danger_polygon_base, point_base)) {
      danger_count++;
      continue;
    }

    if (!warning_polygon_base.empty() &&
        zone_detector_->isPointInPolygon(warning_polygon_base, point_base)) {
      warning_count++;
    }
  }

  // Update status map
  {
    std::lock_guard<std::mutex> lock(status_map_mutex_);
    // Safety rule: any valid point inside the danger zone is an intrusion.
    // Warning keeps a small persistence filter to suppress single reflections.
    if (danger_count > 0) {
      RCLCPP_WARN(get_logger(), "LiDAR %s detected %d obstacles in danger zone",
                  topic.c_str(), danger_count);
      lidar_status_map_[lidar_id] = "danger zone";
    } else if (warning_count >=
               5) { // danger takes precedence — a point can't trigger both
      RCLCPP_WARN(get_logger(),
                  "LiDAR %s detected %d obstacles in warning zone",
                  topic.c_str(), warning_count);
      lidar_status_map_[lidar_id] = "warning zone";
    } else {
      lidar_status_map_[lidar_id] = "safe";
    }
  }

  // Publish visualization markers
  viz_publisher_->publishZoneMarkers(lidar_id, topic, danger_polygon_base,
                                     warning_polygon_base);

  // Update overall safety status
  updateOverallSafetyStatus();
}

bool SafetyNode::lookupSensorToBase(
    const sensor_msgs::msg::LaserScan::SharedPtr &msg,
    geometry_msgs::msg::TransformStamped &transform) {
  if (msg->header.frame_id.empty()) {
    RCLCPP_ERROR(get_logger(), "LiDAR scan has empty frame_id");
    return false;
  }

  try {
    /*
     * All BOPT LiDAR mounting transforms are static relative
     * to base_link, as defined by URDF/robot_state_publisher.
     *
     * Therefore DO NOT request the transform at the LaserScan
     * timestamp. Doing so can produce:
     *
     *   "extrapolation into the future"
     *
     * when Gazebo sensor timestamps are slightly ahead of the
     * TF buffer.
     *
     * TimePointZero requests the latest available transform and
     * is appropriate for a static sensor->base_link transform.
     */
    transform = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id,
                                            tf2::TimePointZero,
                                            tf2::durationFromSec(0.5));

    return true;

  } catch (const tf2::TransformException &ex) {

    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "LiDAR TF failure: %s -> %s : %s",
        msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());

    return false;
  }
}

std::pair<double, double> SafetyNode::transformPointToBase(
    const std::pair<double, double> &point_sensor,
    const geometry_msgs::msg::TransformStamped &transform) {
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);

  tf2::Matrix3x3 rotation(q);

  /*
   * LaserScan is a planar measurement.
   *
   * The measured point is therefore represented as:
   *
   *   (x_sensor, y_sensor, 0)
   *
   * We then apply the COMPLETE 3D TF rotation.
   *
   * This is important for:
   *   toplidar_front  (+26.9 deg pitch)
   *   toplidar_back   (-25.8 deg pitch)
   *
   * Even though safety logic is 2D, the sensor itself is mounted
   * with pitch, so the complete rotation must be applied first.
   */

  const double x = point_sensor.first;
  const double y = point_sensor.second;
  const double z = 0.0;

  const double x_base = rotation[0][0] * x + rotation[0][1] * y +
                        rotation[0][2] * z + transform.transform.translation.x;

  const double y_base = rotation[1][0] * x + rotation[1][1] * y +
                        rotation[1][2] * z + transform.transform.translation.y;

  return {x_base, y_base};
}

std::vector<std::pair<double, double>> SafetyNode::transformPolygonToBase(
    const std::vector<std::pair<double, double>> &polygon_sensor,
    const geometry_msgs::msg::TransformStamped &transform) {

  std::vector<std::pair<double, double>> polygon_base;
  polygon_base.reserve(polygon_sensor.size());
  for (const auto &point : polygon_sensor) {
    polygon_base.push_back(transformPointToBase(point, transform));
  }
  return polygon_base;
}

std::pair<std::string, std::string> SafetyNode::determineDirections() const {
  std::lock_guard<std::mutex> lock(velocity_mutex_);

  // Zero velocity is treated as Forward — policy selector handles the
  // stationary case
  std::string direction =
      (current_velocity_.linear.x >= 0) ? "Forward" : "Reverse";
  std::string angular_direction =
      (current_velocity_.angular.z >= 0) ? "Counter-Clockwise" : "Clockwise";

  return {direction, angular_direction};
}

void SafetyNode::updateOverallSafetyStatus() {
  std::string status = "safe";
  bool has_danger = false;
  bool has_warning = false;

  {
    std::lock_guard<std::mutex> lock(status_map_mutex_);
    for (const auto &[id, lidar_status] : lidar_status_map_) {
      if (lidar_status == "danger zone") {
        has_danger = true;
        break; // one danger sensor is enough — no need to check the rest
      } else if (lidar_status == "warning zone") {
        has_warning = true;
      }
    }
  }

  if (has_danger) {
    status = "danger";
  } else if (has_warning) {
    status = "warning";
  }

  // Fault conditions (LiDAR timeout, MQTT loss) can override the zone-based
  // status
  status = error_handler_->getFaultModifiedStatus(status);

  // Suppress zone alerts when safety is off, but always show system faults
  // (MQTT/LiDAR timeout)
  bool has_fault = error_handler_->hasActiveFaults();
  safety_status_.data = (safety_turn_off_ && !has_fault) ? "safe" : status;
  publishSafetyStatus();
  velocity_controller_->publishModifiedVelocities(status, safety_turn_off_);
}

void SafetyNode::publishSafetyStatus() {
  safety_status_publisher_->publish(safety_status_);
  RCLCPP_INFO(get_logger(), "Published safety status: %s",
              safety_status_.data.c_str());
}

} // namespace safety_demo