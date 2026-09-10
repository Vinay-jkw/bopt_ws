// Stores incoming velocity commands and republishes them scaled or zeroed according to the active safety status.
#ifndef SAFETY_DEMO_VELOCITY_CONTROLLER_HPP
#define SAFETY_DEMO_VELOCITY_CONTROLLER_HPP

#include <string>
#include <mutex>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"

namespace safety_demo {

/**
 * @brief Class for controlling robot velocity based on safety status
 */
class VelocityController {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for creating publishers
     */
    explicit VelocityController(rclcpp::Node* node);

    /**
     * @brief Update current velocity commands
     * @param cmd_vel Twist message from /cmd_vel
     */
    void updateCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Update current velocity command
     * @param velocity Float64 message from /velocity
     */
    void updateVelocity(const std_msgs::msg::Float64::SharedPtr msg);

    /**
     * @brief Publish modified velocities based on safety status
     * @param safety_status Current safety status ("safe", "warning", "danger",
     *                      "broker_disconnected", "localization_lost")
     * @param safety_turn_off Whether safety is turned off
     */
    void publishModifiedVelocities(const std::string& safety_status, bool safety_turn_off);

private:
    rclcpp::Node* node_;  ///< ROS2 node reference

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_remapped_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_remapped_publisher_;

    // Stored velocity commands
    geometry_msgs::msg::Twist current_cmd_vel_;
    std_msgs::msg::Float64 current_velocity_msg_;

    // Mutexes for thread safety
    std::mutex cmd_vel_mutex_;
    std::mutex velocity_msg_mutex_;

    /**
     * @brief Round a float value to two decimal places
     * @param value Input value
     * @return Rounded value
     */
    static float roundToTwoDecimals(float value);
};

} // namespace safety_demo

#endif // SAFETY_DEMO_VELOCITY_CONTROLLER_HPP