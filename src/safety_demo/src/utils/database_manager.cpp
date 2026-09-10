// Owns the SQLite connection and loads safety policies and LiDAR configurations at startup.
#include "safety_demo/database_manager.hpp"
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace safety_demo {

DatabaseManager::DatabaseManager(const std::string& db_path) {
    int rc = sqlite3_open(db_path.c_str(), &db_);
    if (rc != SQLITE_OK) {
        std::string error_msg = "Can't open database: " + std::string(sqlite3_errmsg(db_));
        sqlite3_close(db_);
        throw std::runtime_error(error_msg);
    }
}

DatabaseManager::~DatabaseManager() {
    if (db_) {
        sqlite3_close(db_);
    }
}

void DatabaseManager::checkResult(int result, const std::string& operation) const {
    if (result != SQLITE_OK) {
        std::string error_msg = operation + " failed: " + std::string(sqlite3_errmsg(db_));
        throw std::runtime_error(error_msg);
    }
}

std::map<int, std::multiset<Policy>> DatabaseManager::loadPolicies() {
    // Grouped by lidar_id; multiset keeps them sorted by max_speed via Policy::operator<
    std::map<int, std::multiset<Policy>> policies;

    sqlite3_stmt* stmt = nullptr;
    // Only load Warning and Danger — other types (e.g. Info) are not used by the safety logic
    const char* sql = "SELECT policy_id, lidar_id, field_type, description, max_speed, "
                      "angular_max_speed, direction, angular_direction, a, b, i1, i2, o1, o2 "
                      "FROM policy WHERE field_type IN ('Warning', 'Danger')";

    checkResult(sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr), "Prepare policy statement");

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        Policy policy;
        policy.id = sqlite3_column_int(stmt, 0);
        int lidar_id = sqlite3_column_int(stmt, 1);

        // sqlite3_column_text returns NULL for SQL NULL — guard against that
        const char* field_type_raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        policy.field_type = field_type_raw ? field_type_raw : "";

        const char* desc_raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        policy.description = desc_raw ? desc_raw : "";

        policy.max_speed = static_cast<float>(sqlite3_column_double(stmt, 4));
        policy.angular_max_speed = static_cast<float>(sqlite3_column_double(stmt, 5));

        const char* dir_raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6));
        policy.direction = dir_raw ? dir_raw : "";

        const char* ang_dir_raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 7));
        policy.angular_direction = ang_dir_raw ? ang_dir_raw : "";

        // DB stores zone dimensions in cm; convert to meters here
        double a = sqlite3_column_double(stmt, 8);
        double b = sqlite3_column_double(stmt, 9);
        double i1 = sqlite3_column_double(stmt, 10);
        double i2 = sqlite3_column_double(stmt, 11);
        double o1 = sqlite3_column_double(stmt, 12);
        double o2 = sqlite3_column_double(stmt, 13);

        if (policy.field_type == "Danger") {
            policy.danger_zone = {static_cast<float>(a * 0.01), static_cast<float>(b * 0.01),
                                  static_cast<float>(i1 * 0.01), static_cast<float>(i2 * 0.01),
                                  static_cast<float>(o1 * 0.01), static_cast<float>(o2 * 0.01)};
        } else if (policy.field_type == "Warning") {
            policy.warning_zone = {static_cast<float>(a * 0.01), static_cast<float>(b * 0.01),
                                   static_cast<float>(i1 * 0.01), static_cast<float>(i2 * 0.01),
                                   static_cast<float>(o1 * 0.01), static_cast<float>(o2 * 0.01)};
        }

        policies[lidar_id].insert(policy);
    }

    sqlite3_finalize(stmt);
    return policies;
}

std::map<std::string, LiDARConfig> DatabaseManager::loadLiDARConfigs() {
    // Keyed by exact ROS topic name so SafetyNode can do a fast lookup when a scan arrives.
    std::map<std::string, LiDARConfig> configs;

    sqlite3_stmt* stmt = nullptr;
    const char* sql = "SELECT lidar_id, topic, shape_type, x_offset, y_offset, theta, theta_N, description "
                      "FROM lidar";

    checkResult(sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr), "Prepare LiDAR config statement");

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        LiDARConfig config;
        config.lidar_id = sqlite3_column_int(stmt, 0);

        const char* topic_raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        config.topic = topic_raw ? topic_raw : "";

        const char* shape_raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        config.shape_type = shape_raw ? shape_raw : "";

        config.x_offset = static_cast<float>(sqlite3_column_double(stmt, 3));
        config.y_offset = static_cast<float>(sqlite3_column_double(stmt, 4));
        config.theta = static_cast<float>(sqlite3_column_double(stmt, 5)) * M_PI / 180.0f; // DB stores degrees
        config.theta_N = static_cast<float>(sqlite3_column_double(stmt, 6));

        const char* desc_raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 7));
        config.description = desc_raw ? desc_raw : "";

        configs[config.topic] = config;
    }

    sqlite3_finalize(stmt);
    return configs;
}

std::vector<std::string> DatabaseManager::getLidarTopics() {
    std::vector<std::string> topics;

    sqlite3_stmt* stmt = nullptr;
    const char* sql = "SELECT topic FROM lidar";

    checkResult(sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr), "Prepare topic statement");

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        const char* topic_raw = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        if (topic_raw) {
            topics.push_back(topic_raw);
        }
    }

    sqlite3_finalize(stmt);
    return topics;
}

} // namespace safety_demo