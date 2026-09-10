#include "safety_demo/safety_zone_detector.hpp"

#include <cmath>
#include <algorithm>

namespace safety_demo {

std::vector<std::pair<double, double>>
SafetyZoneDetector::defineFieldPolygon(
    const std::string& shape_type,
    const ZoneParameters& params)
{
    std::vector<std::pair<double, double>> polygon;

    /*
     * IMPORTANT:
     *
     * These coordinates are ALWAYS in the LiDAR's own LaserScan frame.
     *
     * ROS LaserScan convention:
     *   +X : angle = 0 rad
     *   +Y : positive angle direction
     *
     * Physical sensor mounting is NOT handled here.
     * SafetyNode transforms this polygon through TF:
     *
     *       lidar_frame -> base_link
     */

    if (shape_type == "Rectangle") {

        /*
         * Rectangle:
         *
         *             +X (scan forward)
         *              ^
         *              |
         *        +-----+-----+
         *        |           |
         *        |           |
         *        |   ZONE    |
         *        |           |
         *        +-----------+
         *
         * a = width across Y
         * b = length along +X
         *
         * Start at sensor origin so the safety field extends
         * outward from the LiDAR.
         */
        polygon = {
            {0.0,       -params.a / 2.0},
            {params.b,  -params.a / 2.0},
            {params.b,   params.a / 2.0},
            {0.0,        params.a / 2.0}
        };

    } else if (shape_type == "L-Shape") {

        /*
         * Left-side sensor field.
         *
         * The field starts at the LiDAR origin and extends
         * primarily in +X while keeping the shape asymmetric.
         *
         * Parameters are interpreted in metres.
         */
        polygon = {
            {0.0,        0.0},
            {params.i1,  0.0},
            {params.i1,  params.o1},
            {-params.o2, params.o1},
            {-params.o2, -params.i2},
            {0.0,       -params.i2}
        };

    } else if (shape_type == "Mirror L-Shape") {

        /*
         * Mirrored right-side field.
         */
        polygon = {
            {0.0,        0.0},
            {-params.i1, 0.0},
            {-params.i1, params.o1},
            {params.o2,  params.o1},
            {params.o2, -params.i2},
            {0.0,       -params.i2}
        };

    } else {

        /*
         * Conservative fallback.
         *
         * Unknown geometry must never produce an invalid polygon.
         */
        polygon = {
            {0.0,       -params.a / 2.0},
            {params.b,  -params.a / 2.0},
            {params.b,   params.a / 2.0},
            {0.0,        params.a / 2.0}
        };
    }

    return polygon;
}


bool SafetyZoneDetector::isPointInPolygon(
    const std::vector<std::pair<double, double>>& polygon,
    std::pair<double, double> point)
{
    if (polygon.size() < 3) {
        return false;
    }

    bool inside = false;
    const size_t n = polygon.size();

    for (size_t i = 0, j = n - 1; i < n; j = i++) {

        const double xi = polygon[i].first;
        const double yi = polygon[i].second;

        const double xj = polygon[j].first;
        const double yj = polygon[j].second;

        /*
         * Explicit boundary test.
         *
         * Safety-wise, being exactly on the boundary is considered
         * an intrusion.
         */
        const double cross =
            (yj - yi) * (point.first - xi) -
            (xj - xi) * (point.second - yi);

        if (std::abs(cross) < 1e-9 &&
            point.first >= std::min(xi, xj) &&
            point.first <= std::max(xi, xj) &&
            point.second >= std::min(yi, yj) &&
            point.second <= std::max(yi, yj)) {
            return true;
        }

        /*
         * Standard ray casting.
         */
        const bool crosses =
            ((yi > point.second) != (yj > point.second));

        if (crosses) {
            const double x_intersection =
                (xj - xi) *
                (point.second - yi) /
                (yj - yi) +
                xi;

            if (point.first < x_intersection) {
                inside = !inside;
            }
        }
    }

    return inside;
}


std::pair<double, double>
SafetyZoneDetector::polarToCartesian(
    double range,
    double angle)
{
    return {
        range * std::cos(angle),
        range * std::sin(angle)
    };
}

} // namespace safety_demo
