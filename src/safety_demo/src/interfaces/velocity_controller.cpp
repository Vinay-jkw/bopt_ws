// Applies safety limits to velocity commands — passes through, scales, or
// zeroes them based on current safety status.
#include "safety_demo/velocity_controller.hpp"
#include <algorithm>

namespace safety_demo {

VelocityController::VelocityController(rclcpp::Node *node) : node_(node) {
  cmd_vel_publisher_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  cmd_vel_remapped_publisher_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_remapped",
                                                         10);

  velocity_remapped_publisher_ =
      node_->create_publisher<std_msgs::msg::Float64>("/velocity_remapped", 10);

  bopt_cmd_publisher_ =
      node_->create_publisher<bopt_interfaces::msg::BoptCommandStamped>(
          "bopt/relay_cmd", 10);

  bopt_key_cmd_publisher_ =
      node_->create_publisher<bopt_interfaces::msg::BoptCommandStamped>(
          "bopt/key_cmd", 10);

  bopt_nmpc_cmd_publisher_ =
      node_->create_publisher<bopt_interfaces::msg::BoptCommand>(
          "bopt/nmpc_cmd", 10);

  // 50 Hz timer (every 20ms) matches bopt_twist_relay frequency to guarantee
  // override of stop commands
  control_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&VelocityController::controlTimerCallback, this));
}

void VelocityController::updateCmdVel(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  current_cmd_vel_ = *msg;
  RCLCPP_INFO(node_->get_logger(),
              ">>> [STEP 1 - INPUT cmd_vel]  linear.x=%.4f  linear.y=%.4f  "
              "angular.z=%.4f",
              msg->linear.x, msg->linear.y, msg->angular.z);
}

void VelocityController::updateVelocity(
    const std_msgs::msg::Float64::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(velocity_msg_mutex_);
  current_velocity_msg_ = *msg;
  RCLCPP_INFO(node_->get_logger(), ">>> [STEP 1 - INPUT velocity]  data=%.4f",
              msg->data);
}

void VelocityController::updateBoptCommand(
    const bopt_interfaces::msg::BoptCommandStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(bopt_cmd_mutex_);
  current_bopt_cmd_ = *msg;
}

void VelocityController::publishModifiedVelocities(
    const std::string &safety_status, bool safety_turn_off) {
  // Update safety state for 50 Hz control loop
  {
    std::lock_guard<std::mutex> state_lock(safety_state_mutex_);
    current_safety_status_ = safety_status;
    current_safety_turn_off_ = safety_turn_off;
  }

  // Trigger immediate publication
  controlTimerCallback();
}

void VelocityController::controlTimerCallback() {
  std::string status;
  bool turn_off;
  {
    std::lock_guard<std::mutex> state_lock(safety_state_mutex_);
    status = current_safety_status_;
    turn_off = current_safety_turn_off_;
  }

  // If safety is turned off or status is safe, do not override
  if (turn_off || status == "safe") {
    return;
  }

  std::lock_guard<std::mutex> cmd_lock(cmd_vel_mutex_);
  std::lock_guard<std::mutex> vel_lock(velocity_msg_mutex_);
  std::lock_guard<std::mutex> bopt_lock(bopt_cmd_mutex_);

  geometry_msgs::msg::Twist output_cmd_vel = current_cmd_vel_;
  std_msgs::msg::Float64 output_velocity = current_velocity_msg_;
  bopt_interfaces::msg::BoptCommandStamped output_bopt_stamped =
      current_bopt_cmd_;
  bopt_interfaces::msg::BoptCommand output_bopt_nmpc;

  output_bopt_stamped.header.stamp = node_->now();
  output_bopt_stamped.header.frame_id = "base_link";

  if (status == "danger" || status == "broker_disconnected" ||
      status == "localization_lost") {
    // Hard stop — zero out all velocities
    output_cmd_vel.linear.x = 0.0;
    output_cmd_vel.linear.y = 0.0;
    output_cmd_vel.linear.z = 0.0;
    output_cmd_vel.angular.x = 0.0;
    output_cmd_vel.angular.y = 0.0;
    output_cmd_vel.angular.z = 0.0;
    output_velocity.data = 0.0;

    output_bopt_stamped.traction_velocity = 0.0;
    output_bopt_nmpc.traction_velocity = 0.0;
    output_bopt_nmpc.steering_angle = current_bopt_cmd_.steering_angle;

  } else if (status == "warning") {
    // Slow down to 1/3
    constexpr double REDUCTION_FACTOR = 1.0 / 2.0;

    output_cmd_vel.linear.x *= REDUCTION_FACTOR;
    output_cmd_vel.linear.y *= REDUCTION_FACTOR;
    output_cmd_vel.linear.z *= REDUCTION_FACTOR;
    output_cmd_vel.angular.x *= REDUCTION_FACTOR;
    output_cmd_vel.angular.y *= REDUCTION_FACTOR;
    output_cmd_vel.angular.z *= REDUCTION_FACTOR;
    output_velocity.data *= REDUCTION_FACTOR;

    output_bopt_stamped.traction_velocity *= REDUCTION_FACTOR;
    output_bopt_nmpc.traction_velocity = output_bopt_stamped.traction_velocity;
    output_bopt_nmpc.steering_angle = current_bopt_cmd_.steering_angle;
  }

  // Publish continuous override to all motion command topics at 50 Hz
  bopt_cmd_publisher_->publish(output_bopt_stamped);
  bopt_key_cmd_publisher_->publish(output_bopt_stamped);
  bopt_nmpc_cmd_publisher_->publish(output_bopt_nmpc);
  cmd_vel_publisher_->publish(output_cmd_vel);
  cmd_vel_remapped_publisher_->publish(output_cmd_vel);
  velocity_remapped_publisher_->publish(output_velocity);
}

float VelocityController::roundToTwoDecimals(float value) {
  return std::round(value * 100.0f) / 100.0f;
}

} // namespace safety_demo