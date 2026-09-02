// Picks the best-matching danger and warning policy for the robot's current speed, direction, and operational mode.
#include "safety_demo/policy_selector.hpp"
#include <algorithm>

namespace safety_demo {

PolicySelector::PolicySelector(const std::map<int, std::multiset<Policy>>& policies)
    : policies_(policies) {}

std::pair<Policy, Policy> PolicySelector::selectPolicies(
    float current_linear_speed,
    float current_angular_speed,
    const std::string& direction,
    const std::string& angular_direction,
    int lidar_id,
    bool in_parking) {

    Policy danger_policy = selectSinglePolicy("Danger", current_linear_speed, direction,
                                             angular_direction, lidar_id, in_parking);
    Policy warning_policy = selectSinglePolicy("Warning", current_linear_speed, direction,
                                              angular_direction, lidar_id, in_parking);

    return std::make_pair(danger_policy, warning_policy);
}

Policy PolicySelector::selectSinglePolicy(
    const std::string& field_type,
    float current_linear_speed,
    const std::string& direction,
    const std::string& angular_direction,
    int lidar_id,
    bool in_parking) {

    Policy selected_policy;
    selected_policy.id = -1;  // Sentinel value for invalid policy

    // Check if policies exist for this LiDAR
    auto lidar_it = policies_.find(lidar_id);
    if (lidar_it == policies_.end() || lidar_it->second.empty()) {
        return selected_policy;
    }

    const auto& lidar_policies = lidar_it->second;

    // Collect applicable policies
    std::vector<Policy> applicable_policies;
    for (const auto& policy : lidar_policies) {
        // Parking uses a fixed narrow policy; normal operation excludes it entirely
        if (in_parking && policy.description != "PRKNG_POLICY") {
            continue;
        }
        if (!in_parking && policy.description == "PRKNG_POLICY") {
            continue;
        }

        // Filter by field type and direction
        if (policy.field_type != field_type || policy.direction != direction) {
            continue;
        }

        // angular_max_speed == -1 means this policy applies to any rotation direction
        if (policy.angular_max_speed != -1 && policy.angular_direction != angular_direction) {
            continue;
        }

        applicable_policies.push_back(policy);
    }

    if (applicable_policies.empty()) {
        return selected_policy;
    }

    std::sort(applicable_policies.begin(), applicable_policies.end(),
              [](const Policy& a, const Policy& b) {
                  return a.max_speed < b.max_speed;
              });

    // Pick the tightest policy whose max_speed still covers the current speed
    const Policy* prev_policy = nullptr;
    for (const auto& policy : applicable_policies) {
        if (current_linear_speed <= policy.max_speed) {
            if (!prev_policy || current_linear_speed > prev_policy->max_speed) {
                selected_policy = policy;
                break;
            }
        }
        prev_policy = &policy;
    }

    // If speed exceeds all thresholds, use the widest policy we have
    if (selected_policy.id == -1) {
        selected_policy = applicable_policies.back();
    }

    return selected_policy;
}

} // namespace safety_demo