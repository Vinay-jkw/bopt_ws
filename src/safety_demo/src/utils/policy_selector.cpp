#include "safety_demo/policy_selector.hpp"

#include <algorithm>

namespace safety_demo {

PolicySelector::PolicySelector(
    const std::map<int, std::multiset<Policy>>& policies)
    : policies_(policies)
{
}

std::pair<Policy, Policy> PolicySelector::selectPolicies(
    float current_linear_speed,
    float current_angular_speed,
    const std::string& direction,
    const std::string& angular_direction,
    int lidar_id,
    bool in_parking,
    bool pickdrop_mode)
{
    (void)current_angular_speed;

    Policy danger_policy = selectSinglePolicy(
        "Danger",
        current_linear_speed,
        direction,
        angular_direction,
        lidar_id,
        in_parking,
        pickdrop_mode);

    Policy warning_policy = selectSinglePolicy(
        "Warning",
        current_linear_speed,
        direction,
        angular_direction,
        lidar_id,
        in_parking,
        pickdrop_mode);

    return std::make_pair(danger_policy, warning_policy);
}


Policy PolicySelector::selectSinglePolicy(
    const std::string& field_type,
    float current_linear_speed,
    const std::string& direction,
    const std::string& angular_direction,
    int lidar_id,
    bool in_parking,
    bool pickdrop_mode)
{
    Policy selected_policy;
    selected_policy.id = -1;

    auto lidar_it = policies_.find(lidar_id);

    if (lidar_it == policies_.end() ||
        lidar_it->second.empty()) {
        return selected_policy;
    }

    const auto& lidar_policies = lidar_it->second;

    std::vector<Policy> applicable_policies;

    for (const auto& policy : lidar_policies) {

        /*
         * Parking policy:
         *
         * Allowed only while robot is in parking mode.
         */
        if (policy.description == "PRKNG_POLICY") {

            if (!in_parking) {
                continue;
            }

        } else {

            /*
             * Normal operation:
             * PRKNG_POLICY must not participate.
             */
            if (in_parking) {
                continue;
            }
        }

        /*
         * Pickdrop policy:
         *
         * "Pickdrop Zone" is a special operational policy.
         * It must NEVER be selected during normal operation.
         */
        if (policy.description == "Pickdrop Zone") {

            if (!pickdrop_mode) {
                continue;
            }

            /*
             * Pickdrop mode:
             * allow the special policy only for the intended
             * LiDAR/policy combination.
             */
        } else {

            /*
             * During pickdrop mode we still want the normal
             * safety policies to remain active unless a separately
             * validated special policy supersedes them.
             *
             * Therefore we do NOT remove normal policies here.
             */
        }

        /*
         * Field type and travel direction must match.
         */
        if (policy.field_type != field_type ||
            policy.direction != direction) {
            continue;
        }

        /*
         * angular_max_speed == -1 means all angular directions.
         */
        if (policy.angular_max_speed != -1.0f &&
            policy.angular_direction != angular_direction) {
            continue;
        }

        applicable_policies.push_back(policy);
    }

    if (applicable_policies.empty()) {
        return selected_policy;
    }

    /*
     * Select the tightest policy whose speed limit still covers
     * current speed.
     */
    std::sort(
        applicable_policies.begin(),
        applicable_policies.end(),
        [](const Policy& a, const Policy& b) {

            if (a.max_speed != b.max_speed) {
                return a.max_speed < b.max_speed;
            }

            return a.id < b.id;
        });

    for (const auto& policy : applicable_policies) {

        if (current_linear_speed <= policy.max_speed + 1e-6f) {

            selected_policy = policy;

            break;
        }
    }

    /*
     * If current speed is above every configured limit,
     * use the widest/highest-speed policy.
     */
    if (selected_policy.id == -1) {
        selected_policy = applicable_policies.back();
    }

    return selected_policy;
}

} // namespace safety_demo
