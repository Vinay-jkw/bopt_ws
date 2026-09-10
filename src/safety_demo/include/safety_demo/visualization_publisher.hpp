// Creates and publishes RViz markers that outline the active warning and danger zones for each LiDAR.
#ifndef SAFETY_DEMO_VISUALIZATION_PUBLISHER_HPP
#define SAFETY_DEMO_VISUALIZATION_PUBLISHER_HPP

#include <map>
#include <string>
#include <vector>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>
#include "rclcpp/rclcpp.hpp"
#include "safety_demo/database_manager.hpp"

namespace safety_demo {

/**
 * @brief Class for publishing visualization markers for safety zones
 */
class VisualizationPublisher {
public:
    /**
     * @brief Constructor
     * @param node ROS2 node for creating publishers
     * @param lidar_configs Map of LiDAR configurations
     */
    VisualizationPublisher(rclcpp::Node* node, const std::map<std::string, LiDARConfig>& lidar_configs);

    /**
     * @brief Publish visualization markers for danger and warning zones
     * @param lidar_id LiDAR ID
     * @param topic LiDAR topic name
     * @param danger_polygon Danger zone polygon points
     * @param warning_polygon Warning zone polygon points
     */
    void publishZoneMarkers(
        int lidar_id,
        const std::string& topic,
        const std::vector<std::pair<double, double>>& danger_polygon,
        const std::vector<std::pair<double, double>>& warning_polygon);

private:
    rclcpp::Node* node_;  ///< ROS2 node reference

    // Publishers for visualization markers
    std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> marker_publishers_;

    /**
     * @brief Create a visualization marker for a zone
     * @param frame_id Frame ID for the marker
     * @param ns Namespace for the marker
     * @param id Marker ID
     * @param color_r Red color component (0.0-1.0)
     * @param color_g Green color component (0.0-1.0)
     * @param color_b Blue color component (0.0-1.0)
     * @param color_a Alpha (transparency) component (0.0-1.0)
     * @param scale_x Scale in X direction
     * @param polygon Polygon points
     * @return Configured marker message
     */
    visualization_msgs::msg::Marker createZoneMarker(
        const std::string& frame_id,
        const std::string& ns,
        int id,
        float color_r,
        float color_g,
        float color_b,
        float color_a,
        float scale_x,
        const std::vector<std::pair<double, double>>& polygon);
};

} // namespace safety_demo

#endif // SAFETY_DEMO_VISUALIZATION_PUBLISHER_HPP