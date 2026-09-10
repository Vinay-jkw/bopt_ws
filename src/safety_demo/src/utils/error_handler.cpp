// Watchdog for LiDAR timeouts, MQTT connectivity and localization integrity —
// overrides safety status and publishes error codes when faults are detected.
#include "safety_demo/error_handler.hpp"

namespace safety_demo {

ErrorHandler::ErrorHandler(rclcpp::Node* node, const std::vector<std::string>& lidar_topics)
    : node_(node),
      mqtt_msg_received_(false),
      localization_msg_received_(false),
      lidar_timeout_fault_(false),
      mqtt_timeout_fault_(false),
      mqtt_disconnected_(false),
      localization_broken_(false),
      localization_timeout_fault_(false),
      e011_sent_(false),
      mqtt_fault_reported_(false),
      e014_sent_(false),
      localization_fault_reported_(false),
      lidar_timeout_sec_(5.0),
      mqtt_timeout_sec_(2.0),
      // The watchdog ticks at 10 Hz, but it is a Python node doing KD-tree scan
      // matching — a brief stall must not be read as a fault on a loaded forklift.
      localization_timeout_sec_(5.0),
      report_localization_error_(true) {

    // Initialize timestamps
    auto now = node_->now();
    for (const auto& topic : lidar_topics) {
        last_lidar_msg_time_[topic] = now;
    }
    last_mqtt_msg_time_ = now;
    last_localization_msg_time_ = now;

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

    localization_watchdog_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&ErrorHandler::localizationWatchdogCallback, this));
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
    mqtt_msg_received_ = true;
    last_mqtt_msg_time_ = node_->now();
}

void ErrorHandler::updateLocalizationStatus(const std::string& status) {
    // Exactly one value stops the robot. Every other string the watchdog can emit
    // ("localization_ok", "localization_suspect", "localization_unknown") is
    // non-stopping by construction — a typo or a new state cannot cause a stop.
    // LOCALIZATION_SUSPECT in particular fires on healthy runs and must not stop.
    const bool broken = (status == "localization_broken");

    if (broken && !localization_broken_) {
        RCLCPP_ERROR(node_->get_logger(), "LOCALIZATION BROKEN → FORCING STOP");
    } else if (!broken && localization_broken_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Localization recovered (%s) → releasing stop. The pose still "
                    "needs verifying: call /localization/clear_fault when satisfied.",
                    status.c_str());
    }

    localization_broken_ = broken;
    localization_msg_received_ = true;
    last_localization_msg_time_ = node_->now();
}

bool ErrorHandler::hasActiveFaults() const {
    return lidar_timeout_fault_ || mqtt_timeout_fault_ || mqtt_disconnected_ ||
           localization_broken_ || localization_timeout_fault_;
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

    // Localization loss ranks above LiDAR timeout: the LiDARs may be perfectly
    // healthy while the robot has no idea where it is, which is the more dangerous
    // of the two. Its own status string keeps /safety_status diagnosable — a stop
    // here is not an obstacle in the danger zone and should not read as one.
    if (localization_broken_ || localization_timeout_fault_) {
        if (!localization_fault_reported_) {
            RCLCPP_ERROR(node_->get_logger(), "%s → FORCING STOP",
                         localization_broken_ ? "LOCALIZATION LOST"
                                              : "LOCALIZATION WATCHDOG SILENT");
            localization_fault_reported_ = true;  // prevent log spam on every 200ms tick
        }
        if (report_localization_error_ && !e014_sent_) {
            publishMachineError("E014");   // -> machine/error/status -> MQTT -> system manager
            e014_sent_ = true;             // once per fault, reset when it clears
        }
        return "localization_lost";
    } else {
        if (localization_fault_reported_) {
            RCLCPP_INFO(node_->get_logger(),
                        "LOCALIZATION RESTORED → RESUMING NORMAL OPERATION");
            localization_fault_reported_ = false;
        }
        e014_sent_ = false;
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
    // Don't trigger timeout before the first message has ever arrived
    if (!mqtt_msg_received_) {
        return;
    }

    double mqtt_elapsed = (node_->now() - last_mqtt_msg_time_).seconds();

    if (mqtt_elapsed > mqtt_timeout_sec_ || mqtt_disconnected_) {
        mqtt_timeout_fault_ = true;
    } else {
        mqtt_timeout_fault_ = false;
    }
}

void ErrorHandler::localizationWatchdogCallback() {
    // Never fault before the first message has ever arrived. Without this guard a
    // machine running without the localization_watchdog node would refuse to move,
    // which would make this feature impossible to roll out incrementally.
    if (!localization_msg_received_) {
        return;
    }

    double elapsed = (node_->now() - last_localization_msg_time_).seconds();
    bool timed_out = elapsed > localization_timeout_sec_;

    if (timed_out && !localization_timeout_fault_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "/localization/status silent for %.2f sec — the watchdog was "
                     "running and stopped. Treating as a fault.", elapsed);
    }

    localization_timeout_fault_ = timed_out;
}

} // namespace safety_demo