// Polygon geometry utilities — zone construction and point-in-polygon detection for LiDAR scan processing.
#ifndef SAFETY_DEMO_SAFETY_ZONE_DETECTOR_HPP
#define SAFETY_DEMO_SAFETY_ZONE_DETECTOR_HPP

#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <cstddef>
#include "safety_demo/database_manager.hpp"

namespace safety_demo {

/**
 * @brief One polygon edge with every quantity the point test would otherwise
 *        recompute per beam.
 *
 * Kept together in one struct rather than split into parallel arrays: a zone has
 * a handful of edges, the test loop exits early on a boundary hit so it will not
 * vectorize anyway, and this way an edge is one cache line rather than eleven
 * separate streams.
 */
struct PolygonEdge {
    double xi, yi;      ///< Edge start vertex
    double xj, yj;      ///< Edge end vertex (the preceding vertex in the walk)
    double dx, dy;      ///< Edge deltas, xj - xi and yj - yi
    double inv_dy;      ///< 1 / (dy + 1e-9) — the ray-cast denominator, divided once
    double min_x, max_x, min_y, max_y;  ///< Edge extents used by the boundary test
};

/**
 * @brief A zone polygon flattened for the many point tests of a single scan.
 *
 * The polygon changes once per scan while the point test runs once per LiDAR
 * beam, so everything that depends only on the polygon is hoisted in here: the
 * edge data above, plus a bounding box and outer radius that let most beams be
 * discarded before any edge — or even any trigonometry — is touched.
 *
 * Build one with SafetyZoneDetector::preparePolygon().
 */
struct PreparedPolygon {
    std::vector<PolygonEdge> edges;  ///< One allocation per prepared polygon

    // Conservative rejection bounds, padded (see kRejectionMargin).
    double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
    double max_radius_sq = 0.0;  ///< Squared distance to the outermost vertex, from the sensor origin

    bool empty() const { return edges.empty(); }
};

/**
 * @brief Class for detecting obstacles in safety zones
 */
class SafetyZoneDetector {
public:
    /**
     * @brief Padding applied to the cheap rejection bounds, in metres.
     *
     * The exact test treats a point as inside when it lands within 1e-9 (in
     * cross-product units) of an edge, so the bounding box and radius are grown
     * by a margin comfortably above that before they are allowed to reject a
     * point. 1 mm is far below LiDAR range noise, and padding is the safe
     * direction regardless: it can only send more points to the exact test,
     * never fewer.
     */
    static constexpr double kRejectionMargin = 1e-3;

    /**
     * @brief Constructor
     */
    SafetyZoneDetector() = default;

    /**
     * @brief Define a safety zone polygon based on shape type and parameters
     * @param shape_type Type of shape ("Rectangle", "L-Shape", "Mirror L-Shape")
     * @param params Zone parameters
     * @param theta Rotation angle in radians
     * @param theta_N Additional angle parameter
     * @param x_offset X offset from base
     * @param y_offset Y offset from base
     * @return Vector of polygon points (x, y coordinates)
     */
    std::vector<std::pair<double, double>> defineFieldPolygon(
        const std::string& shape_type,
        const ZoneParameters& params,
        float theta,
        float theta_N,
        float x_offset,
        float y_offset);

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

    /**
     * @brief Flatten a polygon into the form the per-point test consumes
     * @param polygon Vector of polygon points, as returned by defineFieldPolygon
     * @return Prepared polygon; empty in, empty out
     *
     * Call once per scan, then reuse the result for every point.
     */
    PreparedPolygon preparePolygon(const std::vector<std::pair<double, double>>& polygon) const;

    /**
     * @brief Cheap radial pre-test, valid before the polar-to-Cartesian conversion
     * @param polygon Prepared polygon
     * @param range Beam range in metres
     * @return False only when the beam is provably outside the polygon
     *
     * A polygon lies inside the disc centred on the sensor origin that touches
     * its outermost vertex, so any beam longer than that radius is outside it —
     * which is most of a scan, since safety zones are metres and scanners reach
     * tens of metres. Rejecting here skips the sin/cos as well as the edge loop.
     */
    bool withinReach(const PreparedPolygon& polygon, double range) const {
        return range * range <= polygon.max_radius_sq;
    }

    /**
     * @brief Ray-casting point-in-polygon test against a prepared polygon
     * @param polygon Prepared polygon
     * @param x Point x coordinate
     * @param y Point y coordinate
     * @return True if the point is inside or on the boundary
     *
     * Same algorithm and same boundary handling as the vector overload above.
     * Defined inline so it folds into the caller's scan loop instead of being a
     * cross-translation-unit call per point.
     */
    bool isPointInPolygon(const PreparedPolygon& polygon, double x, double y) const {
        // Outside the bounding box is outside the polygon — the common case.
        if (x < polygon.min_x || x > polygon.max_x ||
            y < polygon.min_y || y > polygon.max_y) {
            return false;
        }

        bool inside = false;

        for (const PolygonEdge& e : polygon.edges) {
            // Boundary hit counts as inside — avoids ambiguity at zone edges
            if (std::abs(e.dy * (x - e.xi) - e.dx * (y - e.yi)) < 1e-9 &&
                e.min_x <= x && x <= e.max_x &&
                e.min_y <= y && y <= e.max_y) {
                return true;  // Point is on boundary
            }

            // Crossing test; inv_dy carries the 1e-9 guard against horizontal edges
            if (((e.yi > y) != (e.yj > y)) &&
                (x < e.dx * (y - e.yi) * e.inv_dy + e.xi)) {
                inside = !inside;
            }
        }

        return inside;
    }

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