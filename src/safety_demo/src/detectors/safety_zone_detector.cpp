// Handles all geometry — builds zone polygons from DB parameters and checks whether LiDAR points fall inside them.
#include "safety_demo/safety_zone_detector.hpp"
#include <cmath>
#include <algorithm>

namespace safety_demo {

double SafetyZoneDetector::deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

std::vector<std::pair<double, double>> SafetyZoneDetector::defineFieldPolygon(
    const std::string& shape_type,
    const ZoneParameters& params,
    float theta,
    float theta_N,
    float x_offset,
    float y_offset) {

    std::vector<std::pair<double, double>> polygon;

    // theta_N compensates for the sensor's tilt angle — affects how far the far edge is projected
    double denom = std::cos(deg2rad(theta_N));

    if (shape_type == "Rectangle") {
        // Symmetric around sensor origin, front-facing
        polygon = {
            {-params.a / 2.0, 0.0},
            {-params.a / 2.0, params.b / denom},
            {params.a / 2.0, params.b / denom},
            {params.a / 2.0, 0.0}
        };
    } else if (shape_type == "L-Shape") {
        // Used for corner-mounted sensors covering two sides of the robot
        polygon = {
            {0.0, 0.0},
            {params.i1, 0.0},
            {params.i1, params.o1},
            {-params.o2, params.o1},
            {-params.o2, -params.i2},
            {0.0, -params.i2}
        };
    } else if (shape_type == "Mirror L-Shape") {
        // Horizontally mirrored version of L-Shape for the opposite corner
        polygon = {
            {0.0, 0.0},
            {-params.i1, 0.0},
            {-params.i1, params.o1},
            {params.o2, params.o1},
            {params.o2, -params.i2},
            {0.0, -params.i2}
        };
    } else {
        // Unexpected shape type — fall back to a centered rectangle so we don't silently skip detection
        polygon = {
            {-params.a / 2.0, -params.b / 2.0},
            {params.a / 2.0, -params.b / 2.0},
            {params.a / 2.0, params.b / 2.0},
            {-params.a / 2.0, params.b / 2.0}
        };
    }

    // Polygon is built in sensor-local frame; rotate + offset to put it in robot frame
    std::vector<std::pair<double, double>> transformed_polygon;
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    for (const auto& point : polygon) {
        double rotated_x = cos_theta * point.first - sin_theta * point.second + x_offset;
        double rotated_y = sin_theta * point.first + cos_theta * point.second + y_offset;
        transformed_polygon.emplace_back(rotated_x, rotated_y);
    }

    return transformed_polygon;
}

bool SafetyZoneDetector::isPointInPolygon(
    const std::vector<std::pair<double, double>>& polygon,
    std::pair<double, double> point) {

    if (polygon.empty()) {
        return false;
    }

    bool inside = false;
    size_t n = polygon.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const auto& xi_yi = polygon[i];
        const auto& xj_yj = polygon[j];

        double xi = xi_yi.first, yi = xi_yi.second;
        double xj = xj_yj.first, yj = xj_yj.second;

        // Boundary hit counts as inside — avoids ambiguity at zone edges
        if (std::abs((yj - yi) * (point.first - xi) - (xj - xi) * (point.second - yi)) < 1e-9 &&
            std::min(xi, xj) <= point.first && point.first <= std::max(xi, xj) &&
            std::min(yi, yj) <= point.second && point.second <= std::max(yi, yj)) {
            return true; // Point is on boundary
        }

        // 1e-9 guard prevents divide-by-zero on horizontal edges
        if (((yi > point.second) != (yj > point.second)) &&
            (point.first < (xj - xi) * (point.second - yi) / (yj - yi + 1e-9) + xi)) {
            inside = !inside;
        }
    }

    return inside;
}

std::pair<double, double> SafetyZoneDetector::polarToCartesian(double range, double angle) {
    return {range * std::cos(angle), range * std::sin(angle)};
}

} // namespace safety_demo