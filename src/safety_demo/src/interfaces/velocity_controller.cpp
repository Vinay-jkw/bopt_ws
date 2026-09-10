// Applies safety limits to velocity commands — passes through, scales, or zeroes them based on current safety status.
#include "safety_demo/velocity_controller.hpp"
#include <algorithm>

namespace safety_demo {

VelocityController::VelocityController(rclcpp::Node* node) : node_(node) {
    // Downstream motion stack subscribes to these remapped topics, not the raw ones
    cmd_vel_remapped_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_remapped", 10);

    velocity_remapped_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
        "/velocity_remapped", 10);
}

void VelocityController::updateCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_cmd_vel_ = *msg;
    RCLCPP_INFO(node_->get_logger(),
                ">>> [STEP 1 - INPUT cmd_vel]  linear.x=%.4f  linear.y=%.4f  angular.z=%.4f",
                msg->linear.x, msg->linear.y, msg->angular.z);
}

void VelocityController::updateVelocity(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(velocity_msg_mutex_);
    current_velocity_msg_ = *msg;
    RCLCPP_INFO(node_->get_logger(),
                ">>> [STEP 1 - INPUT velocity]  data=%.4f", msg->data);
}

void VelocityController::publishModifiedVelocities(const std::string& safety_status, bool safety_turn_off) {
    // Snapshot both velocities under lock before any modification
    std::lock_guard<std::mutex> cmd_lock(cmd_vel_mutex_);
    std::lock_guard<std::mutex> vel_lock(velocity_msg_mutex_);

    auto output_cmd_vel = std::make_shared<geometry_msgs::msg::Twist>(current_cmd_vel_);
    auto output_velocity = std::make_shared<std_msgs::msg::Float64>(current_velocity_msg_);

    RCLCPP_INFO(node_->get_logger(),
                ">>> [STEP 2 - PROCESSING]  safety_status=%s  safety_turnoff=%s"
                "  input: linear.x=%.4f  angular.z=%.4f  velocity=%.4f",
                safety_status.c_str(), safety_turn_off ? "ON" : "OFF",
                current_cmd_vel_.linear.x, current_cmd_vel_.angular.z,
                current_velocity_msg_.data);

    if (safety_turn_off) {
        // Safety disabled by operator — pass through commands unchanged
        RCLCPP_INFO(node_->get_logger(),
                    ">>> [STEP 3 - OUTPUT]  safety OFF — passing through unchanged"
                    "  linear.x=%.4f  angular.z=%.4f  velocity=%.4f",
                    output_cmd_vel->linear.x, output_cmd_vel->angular.z, output_velocity->data);
        cmd_vel_remapped_publisher_->publish(*output_cmd_vel);
        velocity_remapped_publisher_->publish(*output_velocity);
        return;
    }

    if (safety_status == "danger" || safety_status == "broker_disconnected" ||
        safety_status == "localization_lost") {
        // Hard stop — obstacle in danger zone, lost MQTT connection, or the robot
        // no longer knows where it is
        output_cmd_vel->linear.x = 0.0;
        output_cmd_vel->linear.y = 0.0;
        output_cmd_vel->linear.z = 0.0;
        output_cmd_vel->angular.x = 0.0;
        output_cmd_vel->angular.y = 0.0;
        output_cmd_vel->angular.z = 0.0;
        output_velocity->data = 0.0;

    } else if (safety_status == "warning") {
        // Slow down to 1/3 — enough to give the operator time to react
        constexpr double REDUCTION_FACTOR = 1.0 / 3.0;

        output_cmd_vel->linear.x *= REDUCTION_FACTOR;
        output_cmd_vel->linear.y *= REDUCTION_FACTOR;
        output_cmd_vel->linear.z *= REDUCTION_FACTOR;
        output_cmd_vel->angular.x *= REDUCTION_FACTOR;
        output_cmd_vel->angular.y *= REDUCTION_FACTOR;
        output_cmd_vel->angular.z *= REDUCTION_FACTOR;
        output_velocity->data *= REDUCTION_FACTOR;
    }

    RCLCPP_INFO(node_->get_logger(),
                ">>> [STEP 3 - OUTPUT]  status=%s"
                "  linear.x=%.4f  angular.z=%.4f  velocity=%.4f",
                safety_status.c_str(),
                output_cmd_vel->linear.x, output_cmd_vel->angular.z, output_velocity->data);

    // Both topics are always published together to keep them in sync
    cmd_vel_remapped_publisher_->publish(*output_cmd_vel);
    velocity_remapped_publisher_->publish(*output_velocity);
}

float VelocityController::roundToTwoDecimals(float value) {
    return std::round(value * 100.0f) / 100.0f;
}

} // namespace safety_demo