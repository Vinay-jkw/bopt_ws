// Selects the appropriate danger and warning policies from the loaded DB data
// based on speed, direction, and mode.
#ifndef SAFETY_DEMO_POLICY_SELECTOR_HPP
#define SAFETY_DEMO_POLICY_SELECTOR_HPP

#include "safety_demo/database_manager.hpp"
#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace safety_demo {

/**
 * @brief Class for selecting appropriate safety policies based on current
 * conditions
 */
class PolicySelector {
public:
  /**
   * @brief Constructor
   * @param policies Map of LiDAR ID to set of policies
   */
  explicit PolicySelector(const std::map<int, std::multiset<Policy>> &policies);

  /**
   * @brief Select danger and warning policies for given conditions
   * @param current_linear_speed Current linear speed (absolute value)
   * @param current_angular_speed Current angular speed (absolute value)
   * @param direction Movement direction ("Forward" or "Reverse")
   * @param angular_direction Angular direction ("Clockwise" or
   * "Counter-Clockwise")
   * @param lidar_id LiDAR ID
   * @param in_parking Whether robot is in parking area
   * @return Pair of (danger_policy, warning_policy)
   */
  std::pair<Policy, Policy> selectPolicies(float current_linear_speed,
                                           float current_angular_speed,
                                           const std::string &direction,
                                           const std::string &angular_direction,
                                           int lidar_id, bool in_parking,
                                           bool pickdrop_mode);

private:
  const std::map<int, std::multiset<Policy>>
      &policies_; ///< Reference to policies map

  /**
   * @brief Select a single policy for given field type and conditions
   * @param field_type "Danger" or "Warning"
   * @param current_linear_speed Current linear speed
   * @param direction Movement direction
   * @param angular_direction Angular direction
   * @param lidar_id LiDAR ID
   * @param in_parking Whether in parking area
   * @return Selected policy (id = -1 if none found)
   */
  Policy selectSinglePolicy(const std::string &field_type,
                            float current_linear_speed,
                            const std::string &direction,
                            const std::string &angular_direction, int lidar_id,
                            bool in_parking, bool pickdrop_mode);
};

} // namespace safety_demo

#endif // SAFETY_DEMO_POLICY_SELECTOR_HPP