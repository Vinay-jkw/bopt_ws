// Main ROS2 node — holds all component instances, subscriptions, and state variables for the safety system.
#ifndef SAFETY_DEMO_SAFETY_NODE_HPP
#define SAFETY_DEMO_SAFETY_NODE_HPP

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "bopt_interfaces/msg/bopt_command_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "safety_demo/database_manager.hpp"
#include "safety_demo/safety_zone_detector.hpp"
#include "safety_demo/velocity_controller.hpp"
#include "safety_demo/visualization_publisher.hpp"
#include "safety_demo/error_handler.hpp"
#include "safety_demo/policy_selector.hpp"

namespace safety_demo {

/**
 * @brief Main safety node class that orchestrates all safety monitoring components
 */
class SafetyNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param lidar_topics List of LiDAR topics to monitor
     */
    explicit SafetyNode(const std::vector<std::string>& lidar_topics);

    /**
     * @brief Destructor
     */
    ~SafetyNode() override = default;

private:
    // Core components
    std::unique_ptr<DatabaseManager> db_manager_;
    std::unique_ptr<SafetyZoneDetector> zone_detector_;
    std::unique_ptr<VelocityController> velocity_controller_;
    std::unique_ptr<VisualizationPublisher> viz_publisher_;
    std::unique_ptr<ErrorHandler> error_handler_;
    std::unique_ptr<PolicySelector> policy_selector_;

    // TF2 is the single source of truth for every LiDAR pose/orientation.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    const std::string base_frame_ = "base_link";

    // Data loaded from database
    std::map<int, std::multiset<Policy>> policies_;
    std::map<std::string, LiDARConfig> lidar_configs_;

    // ROS subscriptions
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> lidar_subscriptions_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<bopt_interfaces::msg::BoptCommandStamped>::SharedPtr bopt_cmd_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_turnoff_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pickdrop_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mqtt_status_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_subscriber_;

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_publisher_;

    // State variables
    std::map<int, std::string> lidar_status_map_;  ///< Status per LiDAR ("safe", "warning zone", "danger zone")
    geometry_msgs::msg::Twist current_velocity_;   ///< Current robot velocity
    std_msgs::msg::String safety_status_;          ///< Current overall safety status

    // Control flags
    bool safety_turn_off_;        ///< Whether safety system is turned off
    bool pickdrop_mode_;          ///< Whether in pick/drop mode
    bool robot_in_parking_;       ///< Whether robot is in parking area

    // Special policies
    Policy pickdrop_policy_;      ///< Policy for pick/drop operations

    // Mutexes for thread safety
    mutable std::mutex velocity_mutex_;
    std::mutex status_map_mutex_;
    std::mutex pickdrop_mutex_;
    std::mutex parking_mutex_;
    std::mutex selected_policy_mutex_;

    // Callback functions
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& topic);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void safetyTurnoffCallback(const std_msgs::msg::String::SharedPtr msg);
    void pickdropCallback(const std_msgs::msg::String::SharedPtr msg);
    void mqttStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void localizationStatusCallback(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief Process LiDAR scan and update safety status
     * @param msg LiDAR scan message
     * @param topic LiDAR topic name
     */
    void processLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, const std::string& topic);

    /**
     * @brief Determine movement directions from current velocity
     * @return Pair of (linear_direction, angular_direction)
     */
    std::pair<std::string, std::string> determineDirections() const;

    /**
     * @brief Check all LiDAR statuses and update overall safety status
     */
    void updateOverallSafetyStatus();

    /**
     * @brief Publish current safety status
     */
    void publishSafetyStatus();

    /** Look up the sensor -> base_link transform at the LaserScan timestamp. */
    bool lookupSensorToBase(
        const sensor_msgs::msg::LaserScan::SharedPtr& msg,
        geometry_msgs::msg::TransformStamped& transform);

    /** Apply a cached TF transform to a 2D point (one TF lookup per scan). */
    std::pair<double, double> transformPointToBase(
        const std::pair<double, double>& point_sensor,
        const geometry_msgs::msg::TransformStamped& transform);

    /** Transform a local LiDAR-frame polygon into base_link. */
    std::vector<std::pair<double, double>> transformPolygonToBase(
        const std::vector<std::pair<double, double>>& polygon_sensor,
        const geometry_msgs::msg::TransformStamped& transform);
};

} // namespace safety_demo

#endif // SAFETY_DEMO_SAFETY_NODE_HPP