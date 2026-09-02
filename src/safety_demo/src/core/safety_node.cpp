// Core orchestrator — wires all safety components together, manages subscriptions, and drives the safety status pipeline.
#include "safety_demo/safety_node.hpp"
#include <algorithm>
#include <limits>

namespace safety_demo {

SafetyNode::SafetyNode(const std::vector<std::string>& lidar_topics)
    : Node("safety_node"),
      safety_turn_off_(false),
      pickdrop_mode_(false),
      robot_in_parking_(false) {

    RCLCPP_INFO(get_logger(), "Initializing Safety Node");

    // Initialize database manager and load data
    try {
        // Path is relative to workspace root — must be launched from there
        db_manager_ = std::make_unique<DatabaseManager>("src/safety_demo/config/bopt_2000.db");
        policies_ = db_manager_->loadPolicies();
        lidar_configs_ = db_manager_->loadLiDARConfigs();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize database: %s", e.what());
        throw;
    }

    // Initialize components
    zone_detector_ = std::make_unique<SafetyZoneDetector>();
    velocity_controller_ = std::make_unique<VelocityController>(this);
    viz_publisher_ = std::make_unique<VisualizationPublisher>(this, lidar_configs_);
    error_handler_ = std::make_unique<ErrorHandler>(this, lidar_topics);
    policy_selector_ = std::make_unique<PolicySelector>(policies_);

    // Initialize status map
    for (const auto& [topic, config] : lidar_configs_) {
        lidar_status_map_[config.lidar_id] = "safe";
    }

    // Lambda captures topic by value so each callback knows which sensor triggered it
    for (const auto& topic : lidar_topics) {
        lidar_subscriptions_.push_back(
            create_subscription<sensor_msgs::msg::LaserScan>(
                topic, 10,
                [this, topic](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    lidarCallback(msg, topic);
                }));
    }

    // Velocity comes from odometry (actual measured speed), not /cmd_vel (commanded speed)
    odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&SafetyNode::odometryCallback, this, std::placeholders::_1));

    // Forward incoming velocity commands to VelocityController so it has something to publish
    cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            velocity_controller_->updateCmdVel(msg);
        });

    velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
        "/velocity", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            velocity_controller_->updateVelocity(msg);
        });

    safety_turnoff_subscriber_ = create_subscription<std_msgs::msg::String>(
        "safety_turnoff", 10,
        std::bind(&SafetyNode::safetyTurnoffCallback, this, std::placeholders::_1));

    // /byd/safety drives operational mode: "pickdrop", "normal", or "parking"
    pickdrop_subscriber_ = create_subscription<std_msgs::msg::String>(
        "/byd/safety", 10,
        std::bind(&SafetyNode::pickdropCallback, this, std::placeholders::_1));

    mqtt_status_subscriber_ = create_subscription<std_msgs::msg::String>(
        "/mqtt/status", 10,
        std::bind(&SafetyNode::mqttStatusCallback, this, std::placeholders::_1));

    safety_status_publisher_ = create_publisher<std_msgs::msg::String>("safety_status", 10);

    RCLCPP_INFO(get_logger(), "Safety Node initialized successfully");
}

void SafetyNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& topic) {
    // Update error handler timestamp
    error_handler_->updateLidarTimestamp(topic);

    // Process the LiDAR scan
    processLidarScan(msg, topic);
}

void SafetyNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(velocity_mutex_);

    // Round to 2dp to avoid jitter around zero affecting direction and policy selection
    current_velocity_.linear.x = std::round(msg->twist.twist.linear.x * 100.0) / 100.0;
    current_velocity_.linear.y = std::round(msg->twist.twist.linear.y * 100.0) / 100.0;
    current_velocity_.linear.z = std::round(msg->twist.twist.linear.z * 100.0) / 100.0;
    current_velocity_.angular.x = std::round(msg->twist.twist.angular.x * 100.0) / 100.0;
    current_velocity_.angular.y = std::round(msg->twist.twist.angular.y * 100.0) / 100.0;
    current_velocity_.angular.z = std::round(msg->twist.twist.angular.z * 100.0) / 100.0;

    RCLCPP_INFO(get_logger(),
                ">>> [ODOM - policy speed]  linear.x=%.4f  angular.z=%.4f  (used for zone sizing only)",
                current_velocity_.linear.x, current_velocity_.angular.z);
}

void SafetyNode::safetyTurnoffCallback(const std_msgs::msg::String::SharedPtr msg) {
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
        // Parking mode is handled entirely by the policy selector — nothing else to do here
        return;
    }

    std::lock_guard<std::mutex> lock(selected_policy_mutex_);
    std::lock_guard<std::mutex> pickdrop_lock(pickdrop_mutex_);

    if (msg->data == "pickdrop") {
        pickdrop_mode_ = true;
        RCLCPP_INFO(get_logger(), "Pickdrop mode activated");

        // LiDAR 3 is the fork-tip sensor; pre-load its pickdrop zone policy
        if (policies_.find(3) != policies_.end()) {
            const auto& lidar_policies = policies_[3];
            for (const auto& policy : lidar_policies) {
                if (policy.description == "Pickdrop Zone") {
                    pickdrop_policy_ = policy;
                    RCLCPP_INFO(get_logger(), "Selected Pickdrop Policy ID: %d for LiDAR 3", policy.id);
                    break;
                }
            }
        }
    } else if (msg->data == "normal") {
        if (pickdrop_mode_) {
            pickdrop_mode_ = false;
            pickdrop_policy_ = Policy();  // reset to default-constructed (id = 0)
            RCLCPP_INFO(get_logger(), "Pickdrop mode deactivated");
        }
    }
}

void SafetyNode::mqttStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    error_handler_->updateMqttStatus(msg->data);
}

void SafetyNode::processLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& topic) {
    // Get LiDAR configuration
    auto config_it = lidar_configs_.find(topic);
    if (config_it == lidar_configs_.end()) {
        RCLCPP_WARN(get_logger(), "No configuration found for LiDAR topic: %s", topic.c_str());
        return;
    }
    const auto& config = config_it->second;
    int lidar_id = config.lidar_id;

    // Skip certain LiDARs in pickdrop mode
    {
        std::lock_guard<std::mutex> pickdrop_lock(pickdrop_mutex_);
        // LiDARs 3, 5, 6 cover the fork area — intentionally ignored during pickdrop
        if (pickdrop_mode_ && (lidar_id == 3 || lidar_id == 5 || lidar_id == 6)) {
            std::lock_guard<std::mutex> status_lock(status_map_mutex_);
            lidar_status_map_[lidar_id] = "safe";
            return;
        }
    }

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
    auto [danger_policy, warning_policy] = policy_selector_->selectPolicies(
        std::abs(current_vel.linear.x), std::abs(current_vel.angular.z),
        direction, angular_direction, lidar_id, in_parking);

    // Log selected policies
    if (danger_policy.id != -1) {
        RCLCPP_INFO(get_logger(), "LiDAR %d: Selected Danger Policy ID=%d, max_speed=%.2f",
                   lidar_id, danger_policy.id, danger_policy.max_speed);
    }
    if (warning_policy.id != -1) {
        RCLCPP_INFO(get_logger(), "LiDAR %d: Selected Warning Policy ID=%d, max_speed=%.2f",
                   lidar_id, warning_policy.id, warning_policy.max_speed);
    }

    // Define polygons
    std::vector<std::pair<double, double>> danger_polygon;
    std::vector<std::pair<double, double>> warning_polygon;

    if (danger_policy.id != -1) {
        danger_polygon = zone_detector_->defineFieldPolygon(
            config.shape_type, danger_policy.danger_zone,
            config.theta, config.theta_N, config.x_offset, config.y_offset);
    }
    if (warning_policy.id != -1) {
        warning_polygon = zone_detector_->defineFieldPolygon(
            config.shape_type, warning_policy.warning_zone,
            config.theta, config.theta_N, config.x_offset, config.y_offset);
    }

    // Process LiDAR points
    int danger_count = 0;
    int warning_count = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double range = msg->ranges[i];
        if (range < msg->range_min || range > msg->range_max || range <= 0.05) {
            continue;
        }

        double angle = msg->angle_min + i * msg->angle_increment;
        auto point = zone_detector_->polarToCartesian(range, angle);

        // Check danger zone first
        if (!danger_polygon.empty() && zone_detector_->isPointInPolygon(danger_polygon, point)) {
            danger_count++;
            continue;  // Skip warning check for this point
        }

        // Check warning zone
        if (!warning_polygon.empty() && zone_detector_->isPointInPolygon(warning_polygon, point)) {
            warning_count++;
        }
    }

    // Update status map
    {
        std::lock_guard<std::mutex> lock(status_map_mutex_);
        // 5-point threshold filters out single stray readings / reflections
        if (danger_count >= 5) {
            RCLCPP_WARN(get_logger(), "LiDAR %s detected %d obstacles in danger zone",
                       topic.c_str(), danger_count);
            lidar_status_map_[lidar_id] = "danger zone";
        } else if (warning_count >= 5) {  // danger takes precedence — a point can't trigger both
            RCLCPP_WARN(get_logger(), "LiDAR %s detected %d obstacles in warning zone",
                       topic.c_str(), warning_count);
            lidar_status_map_[lidar_id] = "warning zone";
        } else {
            lidar_status_map_[lidar_id] = "safe";
        }
    }

    // Publish visualization markers
    viz_publisher_->publishZoneMarkers(lidar_id, topic, danger_polygon, warning_polygon);

    // Update overall safety status
    updateOverallSafetyStatus();
}

std::pair<std::string, std::string> SafetyNode::determineDirections() const {
    std::lock_guard<std::mutex> lock(velocity_mutex_);

    // Zero velocity is treated as Forward — policy selector handles the stationary case
    std::string direction = (current_velocity_.linear.x >= 0) ? "Forward" : "Reverse";
    std::string angular_direction = (current_velocity_.angular.z >= 0) ? "Counter-Clockwise" : "Clockwise";

    return {direction, angular_direction};
}

void SafetyNode::updateOverallSafetyStatus() {
    std::string status = "safe";
    bool has_danger = false;
    bool has_warning = false;

    {
        std::lock_guard<std::mutex> lock(status_map_mutex_);
        for (const auto& [id, lidar_status] : lidar_status_map_) {
            if (lidar_status == "danger zone") {
                has_danger = true;
                break;  // one danger sensor is enough — no need to check the rest
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

    // Fault conditions (LiDAR timeout, MQTT loss) can override the zone-based status
    status = error_handler_->getFaultModifiedStatus(status);

    // When safety is turned off, publish "safe" so lights/buzzer are also unaffected
    safety_status_.data = safety_turn_off_ ? "safe" : status;
    publishSafetyStatus();
    velocity_controller_->publishModifiedVelocities(status, safety_turn_off_);
}

void SafetyNode::publishSafetyStatus() {
    safety_status_publisher_->publish(safety_status_);
    RCLCPP_INFO(get_logger(), "Published safety status: %s", safety_status_.data.c_str());
}

} // namespace safety_demo