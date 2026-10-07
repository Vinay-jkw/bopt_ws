// Watchdog for LiDAR timeouts and MQTT connectivity — overrides safety status and publishes error codes when faults are detected.
#include "safety_demo/error_handler.hpp"

namespace safety_demo {

ErrorHandler::ErrorHandler(rclcpp::Node* node, const std::vector<std::string>& lidar_topics)
    : node_(node),
      lidar_timeout_fault_(false),
      mqtt_timeout_fault_(false),
      mqtt_disconnected_(false),
      e011_sent_(false),
      mqtt_fault_reported_(false),
      lidar_timeout_sec_(5.0),
      mqtt_timeout_sec_(2.0) {

    // Initialize timestamps
    auto now = node_->now();
    for (const auto& topic : lidar_topics) {
        last_lidar_msg_time_[topic] = now;
    }
    last_mqtt_msg_time_ = now;

    // Create publishers
    machine_error_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "machine/error/status", 10);

    // Watchdogs fire every 200ms; actual timeout thresholds are 5s (lidar) and 2s (mqtt)
    lidar_watchdog_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&ErrorHandler::lidarWatchdogCallback, this));

    mqtt_watchdog_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&ErrorHandler::mqttWatchdogCallback, this));
}

void ErrorHandler::updateLidarTimestamp(const std::string& topic) {
    last_lidar_msg_time_[topic] = node_->now();
}

void ErrorHandler::updateMqttStatus(const std::string& status) {
    if (status == "disconnected") {
        mqtt_disconnected_ = true;
        RCLCPP_ERROR(node_->get_logger(), "MQTT DISCONNECTED → FORCING STOP");
    } else if (status == "connected") {
        mqtt_disconnected_ = false;
        RCLCPP_INFO(node_->get_logger(), "MQTT CONNECTED");
    }
    last_mqtt_msg_time_ = node_->now();
}

bool ErrorHandler::hasActiveFaults() const {
    return lidar_timeout_fault_ || mqtt_timeout_fault_ || mqtt_disconnected_;
}

std::string ErrorHandler::getFaultModifiedStatus(const std::string& base_status) {
    // MQTT disconnection takes highest priority
    if (mqtt_timeout_fault_ || mqtt_disconnected_) {
        if (!mqtt_fault_reported_) {
            RCLCPP_ERROR(node_->get_logger(), "MQTT TIMEOUT → FORCING STOP");
            mqtt_fault_reported_ = true;  // prevent log spam on every 200ms tick
        }
        return "broker_disconnected";
    } else {
        // Reset MQTT fault flag when recovered
        if (mqtt_fault_reported_) {
            RCLCPP_INFO(node_->get_logger(), "MQTT RESTORED → RESUMING NORMAL OPERATION");
            mqtt_fault_reported_ = false;
        }
    }

    // LiDAR timeout forces danger behavior
    if (lidar_timeout_fault_) {
        if (!e011_sent_) {
            publishMachineError("E011");
            e011_sent_ = true;  // only send once per fault, reset when timeout clears
        }
        return "danger";
    } else {
        e011_sent_ = false;  // Reset when timeout is resolved
    }

    return base_status;
}

void ErrorHandler::publishMachineError(const std::string& error_code) {
    std_msgs::msg::String msg;
    msg.data = error_code;
    machine_error_publisher_->publish(msg);

    RCLCPP_ERROR(node_->get_logger(), "Published Machine Error: %s", error_code.c_str());
}

void ErrorHandler::lidarWatchdogCallback() {
    bool timeout_detected = false;

    for (const auto& entry : last_lidar_msg_time_) {
        double elapsed = (node_->now() - entry.second).seconds();
        if (elapsed > lidar_timeout_sec_) {
            RCLCPP_ERROR(node_->get_logger(),
                        "E011: LiDAR topic %s timeout (%.2f sec)",
                        entry.first.c_str(), elapsed);
            timeout_detected = true;
            break;
        }
    }

    lidar_timeout_fault_ = timeout_detected;
}

void ErrorHandler::mqttWatchdogCallback() {
    double mqtt_elapsed = (node_->now() - last_mqtt_msg_time_).seconds();

    if (mqtt_elapsed > mqtt_timeout_sec_ || mqtt_disconnected_) {
        mqtt_timeout_fault_ = true;
    } else {
        mqtt_timeout_fault_ = false;
    }
}

} // namespace safety_demo