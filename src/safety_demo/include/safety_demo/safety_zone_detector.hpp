// Polygon geometry utilities — zone construction and point-in-polygon detection for LiDAR scan processing.
#ifndef SAFETY_DEMO_SAFETY_ZONE_DETECTOR_HPP
#define SAFETY_DEMO_SAFETY_ZONE_DETECTOR_HPP

#include <vector>
#include <string>
#include <utility>
#include "safety_demo/database_manager.hpp"

namespace safety_demo {

/**
 * @brief Class for detecting obstacles in safety zones
 */
class SafetyZoneDetector {
public:
    /**
     * @brief Constructor
     */
    SafetyZoneDetector() = default;

    /**
     * @brief Define a safety-zone polygon in the LiDAR's LOCAL frame.
     *
     * Physical LiDAR mounting position/orientation is NOT applied here.
     * SafetyNode transforms both the polygon and scan points to base_link via TF2.
     */
    std::vector<std::pair<double, double>> defineFieldPolygon(
        const std::string& shape_type,
        const ZoneParameters& params);

    /**
     * @brief Check if a point is inside a polygon using ray casting algorithm
     * @param polygon Vector of polygon points
     * @param point Point to check (x, y coordinates)
     * @return True if point is inside polygon, false otherwise
     */
    bool isPointInPolygon(
        const std::vector<std::pair<double, double>>& polygon,
        std::pair<double, double> point);

    /**
     * @brief Convert polar coordinates to Cartesian coordinates
     * @param range Distance from origin
     * @param angle Angle in radians
     * @return Cartesian coordinates (x, y)
     */
    std::pair<double, double> polarToCartesian(double range, double angle);

private:
    /**
     * @brief Utility function to convert degrees to radians
     * @param degrees Angle in degrees
     * @return Angle in radians
     */
    static double deg2rad(double degrees);
};

} // namespace safety_demo

#endif // SAFETY_DEMO_SAFETY_ZONE_DETECTOR_HPP