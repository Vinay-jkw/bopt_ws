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

PreparedPolygon SafetyZoneDetector::preparePolygon(
    const std::vector<std::pair<double, double>>& polygon) const {

    PreparedPolygon prepared;
    const size_t n = polygon.size();
    if (n == 0) {
        return prepared;  // max_radius_sq stays 0, so every beam is rejected
    }

    prepared.edges.resize(n);

    double min_x = polygon[0].first, max_x = polygon[0].first;
    double min_y = polygon[0].second, max_y = polygon[0].second;
    double max_radius_sq = 0.0;

    // Edge i closes vertex i back to vertex i-1, matching the (i, j) walk of the
    // vector overload so both produce the same crossing parity.
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = polygon[i].first,  yi = polygon[i].second;
        const double xj = polygon[j].first,  yj = polygon[j].second;

        PolygonEdge& e = prepared.edges[i];
        e.xi = xi;
        e.yi = yi;
        e.xj = xj;
        e.yj = yj;
        e.dx = xj - xi;
        e.dy = yj - yi;
        // Divide once per scan here rather than once per point in the test loop
        e.inv_dy = 1.0 / (yj - yi + 1e-9);

        e.min_x = std::min(xi, xj);
        e.max_x = std::max(xi, xj);
        e.min_y = std::min(yi, yj);
        e.max_y = std::max(yi, yj);

        min_x = std::min(min_x, xi);
        max_x = std::max(max_x, xi);
        min_y = std::min(min_y, yi);
        max_y = std::max(max_y, yi);

        // Radius is measured from the sensor origin because that is where the
        // beams are measured from — see withinReach()
        max_radius_sq = std::max(max_radius_sq, xi * xi + yi * yi);
    }

    prepared.min_x = min_x - kRejectionMargin;
    prepared.max_x = max_x + kRejectionMargin;
    prepared.min_y = min_y - kRejectionMargin;
    prepared.max_y = max_y + kRejectionMargin;

    const double max_radius = std::sqrt(max_radius_sq) + kRejectionMargin;
    prepared.max_radius_sq = max_radius * max_radius;

    return prepared;
}

} // namespace safety_demo