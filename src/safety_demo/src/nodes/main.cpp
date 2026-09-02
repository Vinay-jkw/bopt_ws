// Entry point — loads LiDAR topics from the database and spins the SafetyNode.
#include <rclcpp/rclcpp.hpp>
#include "safety_demo/safety_node.hpp"
#include "safety_demo/database_manager.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        // Topics are stored in the DB so the node doesn't need to be recompiled when sensors change
        safety_demo::DatabaseManager db_manager("src/safety_demo/config/bopt_2000.db");
        std::vector<std::string> lidar_topics = db_manager.getLidarTopics();

        // SafetyNode takes ownership of all subscriptions and component lifetime
        auto node = std::make_shared<safety_demo::SafetyNode>(lidar_topics);
        rclcpp::spin(node);

    } catch (const std::exception& e) {
        // DB failure or missing config is fatal — nothing to fall back to
        RCLCPP_ERROR(rclcpp::get_logger("safety_node"), "Failed to start safety node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}