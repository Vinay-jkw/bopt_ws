// Monitors LiDAR and MQTT health via watchdog timers and exposes fault state to the safety pipeline.
#ifndef SAFETY_DEMO_ERROR_HANDLER_HPP
#define SAFETY_DEMO_ERROR_HANDLER_HPP

#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace safety_demo {

/**
 * @brief Class for handling system errors and timeouts
 */
class ErrorHandler {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for creating publishers and timers
     * @param lidar_topics List of LiDAR topics to monitor
     */
    ErrorHandler(rclcpp::Node* node, const std::vector<std::string>& lidar_topics);

    /**
     * @brief Update timestamp for LiDAR message reception
     * @param topic LiDAR topic name
     */
    void updateLidarTimestamp(const std::string& topic);

    /**
     * @brief Update MQTT status
     * @param status MQTT status ("connected" or "disconnected")
     */
    void updateMqttStatus(const std::string& status);

    /**
     * @brief Check if there are any active faults
     * @return True if any faults are active
     */
    bool hasActiveFaults() const;

    /**
     * @brief Get the current safety status considering faults
     * @param base_status Base safety status from zone detection
     * @return Modified safety status considering faults
     */
    std::string getFaultModifiedStatus(const std::string& base_status);

    /**
     * @brief Publish machine error message
     * @param error_code Error code to publish
     */
    void publishMachineError(const std::string& error_code);

private:
    rclcpp::Node* node_;  ///< ROS2 node reference

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr machine_error_publisher_;

    // Timers
    rclcpp::TimerBase::SharedPtr lidar_watchdog_timer_;
    rclcpp::TimerBase::SharedPtr mqtt_watchdog_timer_;

    // Timeout tracking
    std::map<std::string, rclcpp::Time> last_lidar_msg_time_;
    rclcpp::Time last_mqtt_msg_time_;

    // Fault flags
    bool lidar_timeout_fault_;
    bool mqtt_timeout_fault_;
    bool mqtt_disconnected_;

    // Error tracking flags
    bool e011_sent_;  // LiDAR timeout error
    bool mqtt_fault_reported_;

    // Configuration
    double lidar_timeout_sec_;
    double mqtt_timeout_sec_;

    /**
     * @brief LiDAR watchdog timer callback
     */
    void lidarWatchdogCallback();

    /**
     * @brief MQTT watchdog timer callback
     */
    void mqttWatchdogCallback();
};

} // namespace safety_demo

#endif // SAFETY_DEMO_ERROR_HANDLER_HPP