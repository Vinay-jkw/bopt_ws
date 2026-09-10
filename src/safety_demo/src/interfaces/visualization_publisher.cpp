// Publishes warning and danger zone outlines as RViz LINE_STRIP markers, one pair of topics per LiDAR.
#include "safety_demo/visualization_publisher.hpp"

namespace safety_demo {

VisualizationPublisher::VisualizationPublisher(
    rclcpp::Node* node,
    const std::map<std::string, LiDARConfig>& lidar_configs)
    : node_(node) {

    // One warning + one danger publisher per LiDAR, keyed by topic name for fast lookup
    for (const auto& [topic, config] : lidar_configs) {
        std::string warning_topic = "visualization_marker_" + std::to_string(config.lidar_id) + "_warning";
        std::string danger_topic = "visualization_marker_" + std::to_string(config.lidar_id) + "_danger";

        marker_publishers_[warning_topic] = node_->create_publisher<visualization_msgs::msg::Marker>(
            warning_topic, 10);
        marker_publishers_[danger_topic] = node_->create_publisher<visualization_msgs::msg::Marker>(
            danger_topic, 10);

        RCLCPP_INFO(node_->get_logger(),
                   "Created visualization publishers for LiDAR %d: warning=%s, danger=%s",
                   config.lidar_id, warning_topic.c_str(), danger_topic.c_str());
    }
}

void VisualizationPublisher::publishZoneMarkers(
    int lidar_id,
    const std::string& topic,
    const std::vector<std::pair<double, double>>& danger_polygon,
    const std::vector<std::pair<double, double>>& warning_polygon) {

    std::string warning_topic = "visualization_marker_" + std::to_string(lidar_id) + "_warning";
    std::string danger_topic = "visualization_marker_" + std::to_string(lidar_id) + "_danger";

    // Only publish if a policy was actually selected (empty polygon = no active policy)
    if (!warning_polygon.empty()) {
        // Zone points are already expressed in base_link by SafetyNode.
        auto warning_marker = createZoneMarker(
            "base_link", "warning_zone", lidar_id,
            1.0f, 1.0f, 0.0f, 0.5f,  // yellow, semi-transparent
            0.03f, warning_polygon);

        auto it_warning = marker_publishers_.find(warning_topic);
        if (it_warning != marker_publishers_.end()) {
            it_warning->second->publish(warning_marker);
        }
    }

    if (!danger_polygon.empty()) {
        auto danger_marker = createZoneMarker(
            "base_link", "danger_zone", lidar_id,
            1.0f, 0.0f, 0.0f, 0.5f,  // red, semi-transparent
            0.05f, danger_polygon);   // thicker line than warning to stand out

        auto it_danger = marker_publishers_.find(danger_topic);
        if (it_danger != marker_publishers_.end()) {
            it_danger->second->publish(danger_marker);
        }
    }
}

visualization_msgs::msg::Marker VisualizationPublisher::createZoneMarker(
    const std::string& frame_id,
    const std::string& ns,
    int id,
    float color_r,
    float color_g,
    float color_b,
    float color_a,
    float scale_x,
    const std::vector<std::pair<double, double>>& polygon) {

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;

    // Set color
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = color_a;

    for (const auto& point : polygon) {
        geometry_msgs::msg::Point p;
        p.x = point.first;
        p.y = point.second;
        p.z = 0.0;  // zones are 2D, keep flat in the sensor plane
        marker.points.push_back(p);
    }

    // LINE_STRIP doesn't auto-close, so repeat the first point to complete the outline
    if (!polygon.empty()) {
        geometry_msgs::msg::Point p;
        p.x = polygon[0].first;
        p.y = polygon[0].second;
        p.z = 0.0;
        marker.points.push_back(p);
    }

    return marker;
}

} // namespace safety_demo