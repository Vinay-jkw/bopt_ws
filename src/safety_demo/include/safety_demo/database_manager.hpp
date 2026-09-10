// Defines the DatabaseManager class and the core data structures (Policy, ZoneParameters, LiDARConfig) shared across the package.
#ifndef SAFETY_DEMO_DATABASE_MANAGER_HPP
#define SAFETY_DEMO_DATABASE_MANAGER_HPP

#include <vector>
#include <string>
#include <map>
#include <set>
#include <memory>
#include <sqlite3.h>

namespace safety_demo {

/**
 * @brief Structure representing zone parameters for safety fields
 */
struct ZoneParameters {
    float a;   ///< Width parameter
    float b;   ///< Length parameter
    float i1;  ///< Inner parameter 1
    float i2;  ///< Inner parameter 2
    float o1;  ///< Outer parameter 1
    float o2;  ///< Outer parameter 2
};

/**
 * @brief Structure representing a safety policy
 */
struct Policy {
    int id;                           ///< Policy ID
    std::string field_type;           ///< "Warning" or "Danger"
    std::string description;          ///< Policy description
    float max_speed;                  ///< Maximum linear speed
    float angular_max_speed;          ///< Maximum angular speed
    std::string direction;            ///< Movement direction
    std::string angular_direction;    ///< Angular direction
    ZoneParameters warning_zone;      ///< Warning zone parameters
    ZoneParameters danger_zone;       ///< Danger zone parameters

    /**
     * @brief Comparison operator for sorting policies
     */
    bool operator<(const Policy& other) const {
        if (max_speed != other.max_speed) return max_speed < other.max_speed;
        if (angular_max_speed != other.angular_max_speed) return angular_max_speed < other.angular_max_speed;
        if (direction != other.direction) return direction < other.direction;
        return angular_direction < other.angular_direction;
    }
};

/**
 * @brief Structure representing LiDAR configuration
 */
struct LiDARConfig {
    int lidar_id;              ///< LiDAR ID
    std::string topic;         ///< ROS topic name
    std::string shape_type;    ///< Shape type ("Rectangle", "L-Shape", etc.)
    float x_offset;            ///< X offset from base
    float y_offset;            ///< Y offset from base
    float theta;               ///< Rotation angle in radians
    float theta_N;             ///< Additional angle parameter
    std::string description;   ///< Configuration description
};

/**
 * @brief Database manager class for loading safety policies and LiDAR configurations
 */
class DatabaseManager {
public:
    /**
     * @brief Constructor
     * @param db_path Path to the SQLite database file
     * @throws std::runtime_error if database cannot be opened
     */
    explicit DatabaseManager(const std::string& db_path);

    /**
     * @brief Destructor - closes database connection
     */
    ~DatabaseManager();

    /**
     * @brief Load all safety policies from database
     * @return Map of LiDAR ID to set of policies
     */
    std::map<int, std::multiset<Policy>> loadPolicies();

    /**
     * @brief Load all LiDAR configurations from database
     * @return Map of topic name to LiDAR configuration
     */
    std::map<std::string, LiDARConfig> loadLiDARConfigs();

    /**
     * @brief Get list of LiDAR topics from database
     * @return Vector of topic names
     */
    std::vector<std::string> getLidarTopics();

private:
    sqlite3* db_;  ///< SQLite database handle

    /**
     * @brief Check database operation result and throw on error
     * @param result SQLite result code
     * @param operation Description of the operation for error messages
     */
    void checkResult(int result, const std::string& operation) const;
};

} // namespace safety_demo

#endif // SAFETY_DEMO_DATABASE_MANAGER_HPP