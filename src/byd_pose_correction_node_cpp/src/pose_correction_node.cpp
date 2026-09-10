#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <mutex>

class OrientRobot : public rclcpp::Node
{
public:
  OrientRobot() : Node("orient_robot")
  {
    // ---- Mandatory parameters ----
    rcl_interfaces::msg::ParameterDescriptor z_desc;
    z_desc.description = "Target quaternion z component (REQUIRED)";
    rcl_interfaces::msg::ParameterDescriptor w_desc;
    w_desc.description = "Target quaternion w component (REQUIRED)";

    this->declare_parameter("target_z", rclcpp::PARAMETER_DOUBLE, z_desc);
    this->declare_parameter("target_w", rclcpp::PARAMETER_DOUBLE, w_desc);

    rclcpp::Parameter z_param, w_param;
    if (!this->get_parameter("target_z", z_param) ||
        z_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_FATAL(this->get_logger(), "Required parameter 'target_z' not provided.");
      throw std::runtime_error("Missing required parameter: target_z");
    }
    if (!this->get_parameter("target_w", w_param) ||
        w_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
      RCLCPP_FATAL(this->get_logger(), "Required parameter 'target_w' not provided.");
      throw std::runtime_error("Missing required parameter: target_w");
    }

    target_z_ = z_param.as_double();
    target_w_ = w_param.as_double();

    double norm = std::sqrt(target_z_ * target_z_ + target_w_ * target_w_);
    if (std::abs(norm - 1.0) > 1e-3) {
      RCLCPP_WARN(this->get_logger(),
        "(target_z, target_w) not unit-norm (norm=%.4f). Normalizing.", norm);
      target_z_ /= norm;
      target_w_ /= norm;
    }

    // ---- Tuning parameters (UNCHANGED — these are working well) ----
    this->declare_parameter<double>("max_angular_speed", 0.4);
    this->declare_parameter<double>("kp", 0.8);
    this->declare_parameter<double>("tolerance", 0.03);
    this->declare_parameter<double>("slowdown_zone", 0.785398);
    this->declare_parameter<double>("min_w", 0.08);
    this->declare_parameter<double>("min_w_zone", 0.12);
    this->declare_parameter<double>("hold_duration_sec", 0.3);
    this->declare_parameter<double>("control_rate_hz", 20.0);
    this->declare_parameter<double>("pose_timeout_sec", 2.0);
    this->declare_parameter<int>("max_oscillations", 3);

    // ---- NEW: escape-from-loop parameters ----
    this->declare_parameter<double>("total_timeout_sec", 15.0);   // hard time bound
    this->declare_parameter<int>("max_tolerance_entries", 3);     // accept after N near-misses

    max_w_speed_        = this->get_parameter("max_angular_speed").as_double();
    kp_                 = this->get_parameter("kp").as_double();
    tolerance_          = this->get_parameter("tolerance").as_double();
    slowdown_zone_      = this->get_parameter("slowdown_zone").as_double();
    min_w_              = this->get_parameter("min_w").as_double();
    min_w_zone_         = this->get_parameter("min_w_zone").as_double();
    hold_duration_      = this->get_parameter("hold_duration_sec").as_double();
    pose_timeout_       = this->get_parameter("pose_timeout_sec").as_double();
    max_oscillations_   = this->get_parameter("max_oscillations").as_int();
    total_timeout_      = this->get_parameter("total_timeout_sec").as_double();
    max_tol_entries_    = this->get_parameter("max_tolerance_entries").as_int();
    double rate_hz      = this->get_parameter("control_rate_hz").as_double();

    target_yaw_ = quatToYaw(0.0, 0.0, target_z_, target_w_);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                      .reliable()
                      .transient_local();

    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", amcl_qos,
      std::bind(&OrientRobot::amclCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OrientRobot::controlLoop, this));

    node_start_time_ = this->now();   // NEW

    RCLCPP_INFO(this->get_logger(),
                "Target z=%.3f w=%.3f -> yaw=%.3f rad (%.1f deg). Control @ %.1f Hz.",
                target_z_, target_w_, target_yaw_, target_yaw_ * 180.0 / M_PI, rate_hz);
    RCLCPP_INFO(this->get_logger(), "Waiting for /amcl_pose ...");
  }

private:
  double quatToYaw(double x, double y, double z, double w)
  {
    tf2::Quaternion q(x, y, z, w);
    q.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalizeAngle(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    const auto & q = msg->pose.pose.orientation;
    latest_yaw_ = quatToYaw(q.x, q.y, q.z, q.w);
    last_pose_time_ = this->now();
    pose_received_ = true;
  }

  void stopAndShutdown(const std::string & reason)
  {
    geometry_msgs::msg::Twist stop;
    for (int i = 0; i < 5; ++i) {
      cmd_pub_->publish(stop);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_INFO(this->get_logger(), "%s Shutting down.", reason.c_str());
    control_timer_->cancel();
    rclcpp::shutdown();
  }

  void controlLoop()
  {
    double current_yaw;
    rclcpp::Time last_time;
    bool have_pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      current_yaw = latest_yaw_;
      last_time   = last_pose_time_;
      have_pose   = pose_received_;
    }

    auto now = this->now();

    // ---- NEW: hard timeout — guaranteed escape ----
    double elapsed = (now - node_start_time_).seconds();
    if (elapsed > total_timeout_) {
      char buf[128];
      std::snprintf(buf, sizeof(buf),
        "Total timeout %.1fs reached. Accepting current orientation.", total_timeout_);
      stopAndShutdown(buf);
      return;
    }

    if (!have_pose) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "No /amcl_pose received yet.");
      return;
    }

    double age = (now - last_time).seconds();
    if (age > pose_timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Pose is stale (%.2fs old). Stopping for safety.", age);
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      in_tolerance_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }

    double error    = normalizeAngle(target_yaw_ - current_yaw);
    double abs_err  = std::abs(error);
    geometry_msgs::msg::Twist cmd;

    if (abs_err > tolerance_) {
      // Detect oscillation: sign of error flipped
      if (last_error_sign_ != 0 && error * last_error_sign_ < 0) {
        oscillation_count_++;
        RCLCPP_WARN(this->get_logger(),
          "Overshoot detected (count=%d). Current err=%.4f", oscillation_count_, error);

        if (oscillation_count_ >= max_oscillations_) {
          stopAndShutdown(
            "Max oscillations reached — accepting current orientation.");
          return;
        }
      }
      last_error_sign_ = (error > 0) ? 1 : -1;

      // --- Speed-scaled approach (UNCHANGED) ---
      double dynamic_max = max_w_speed_;
      if (abs_err < slowdown_zone_) {
        dynamic_max = max_w_speed_ * (abs_err / slowdown_zone_);
        dynamic_max = std::max(dynamic_max, 0.05);
      }

      double w = kp_ * error;
      w = std::clamp(w, -dynamic_max, dynamic_max);

      if (abs_err > min_w_zone_ && std::abs(w) < min_w_) {
        w = std::copysign(min_w_, error);
      }

      cmd.angular.z = w;
      in_tolerance_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                  "yaw=%.3f target=%.3f err=%.4f w=%.3f (max=%.2f)",
                  current_yaw, target_yaw_, error, w, dynamic_max);
      cmd_pub_->publish(cmd);
    } else {
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);

      // ---- Tolerance-entry counting (NEW) ----
      // A "tolerance entry" = transitioning from outside-tol to inside-tol.
      // If we keep entering tolerance but bouncing back out, the robot is
      // physically close enough — just accept it on the Nth entry.
      if (in_tolerance_since_.nanoseconds() == 0) {
        in_tolerance_since_ = now;
        tolerance_entries_++;
        RCLCPP_INFO(this->get_logger(),
          "Within tolerance (err=%.4f, entry #%d/%d). Holding for %.2fs...",
          error, tolerance_entries_, max_tol_entries_, hold_duration_);

        if (tolerance_entries_ >= max_tol_entries_) {
          stopAndShutdown(
            "Reached tolerance multiple times — accepting current orientation.");
          return;
        }
      }

      double held_for = (now - in_tolerance_since_).seconds();
      if (held_for >= hold_duration_) {
        stopAndShutdown("Orientation corrected.");
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double target_z_, target_w_, target_yaw_;
  double max_w_speed_, kp_, tolerance_, slowdown_zone_;
  double min_w_, min_w_zone_, pose_timeout_, hold_duration_;
  double total_timeout_;                 // NEW
  int max_oscillations_;
  int max_tol_entries_;                  // NEW

  std::mutex pose_mutex_;
  double latest_yaw_ = 0.0;
  rclcpp::Time last_pose_time_;
  rclcpp::Time node_start_time_;         // NEW
  bool pose_received_ = false;
  rclcpp::Time in_tolerance_since_{0, 0, RCL_ROS_TIME};

  int last_error_sign_ = 0;
  int oscillation_count_ = 0;
  int tolerance_entries_ = 0;            // NEW
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OrientRobot>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("orient_robot"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <cmath>
// #include <algorithm>
// #include <stdexcept>
// #include <mutex>

// class OrientRobot : public rclcpp::Node
// {
// public:
//   OrientRobot() : Node("orient_robot")
//   {
//     // ---- Mandatory parameters ----
//     rcl_interfaces::msg::ParameterDescriptor z_desc;
//     z_desc.description = "Target quaternion z component (REQUIRED)";
//     rcl_interfaces::msg::ParameterDescriptor w_desc;
//     w_desc.description = "Target quaternion w component (REQUIRED)";

//     this->declare_parameter("target_z", rclcpp::PARAMETER_DOUBLE, z_desc);
//     this->declare_parameter("target_w", rclcpp::PARAMETER_DOUBLE, w_desc);

//     rclcpp::Parameter z_param, w_param;
//     if (!this->get_parameter("target_z", z_param) ||
//         z_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
//     {
//       RCLCPP_FATAL(this->get_logger(), "Required parameter 'target_z' not provided.");
//       throw std::runtime_error("Missing required parameter: target_z");
//     }
//     if (!this->get_parameter("target_w", w_param) ||
//         w_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
//     {
//       RCLCPP_FATAL(this->get_logger(), "Required parameter 'target_w' not provided.");
//       throw std::runtime_error("Missing required parameter: target_w");
//     }

//     target_z_ = z_param.as_double();
//     target_w_ = w_param.as_double();

//     double norm = std::sqrt(target_z_ * target_z_ + target_w_ * target_w_);
//     if (std::abs(norm - 1.0) > 1e-3) {
//       RCLCPP_WARN(this->get_logger(),
//         "(target_z, target_w) not unit-norm (norm=%.4f). Normalizing.", norm);
//       target_z_ /= norm;
//       target_w_ /= norm;
//     }

//     // ---- Tuning parameters ----
//     this->declare_parameter<double>("max_angular_speed", 0.4);
//     this->declare_parameter<double>("kp", 0.8);
//     this->declare_parameter<double>("tolerance", 0.03);          // ~1.7°
//     this->declare_parameter<double>("slowdown_zone", 0.785398);      // start slowing ~45° out
//     this->declare_parameter<double>("min_w", 0.08);
//     this->declare_parameter<double>("min_w_zone", 0.12);         // only boost when error > this
//     this->declare_parameter<double>("hold_duration_sec", 0.3);
//     this->declare_parameter<double>("control_rate_hz", 20.0);
//     this->declare_parameter<double>("pose_timeout_sec", 2.0);
//     this->declare_parameter<int>("max_oscillations", 3);         // give up after N overshoots

//     max_w_speed_    = this->get_parameter("max_angular_speed").as_double();
//     kp_             = this->get_parameter("kp").as_double();
//     tolerance_      = this->get_parameter("tolerance").as_double();
//     slowdown_zone_  = this->get_parameter("slowdown_zone").as_double();
//     min_w_          = this->get_parameter("min_w").as_double();
//     min_w_zone_     = this->get_parameter("min_w_zone").as_double();
//     hold_duration_  = this->get_parameter("hold_duration_sec").as_double();
//     pose_timeout_   = this->get_parameter("pose_timeout_sec").as_double();
//     max_oscillations_ = this->get_parameter("max_oscillations").as_int();
//     double rate_hz  = this->get_parameter("control_rate_hz").as_double();

//     target_yaw_ = quatToYaw(0.0, 0.0, target_z_, target_w_);

//     cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

//     auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(10))
//                       .reliable()
//                       .transient_local();

//     amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//       "/amcl_pose", amcl_qos,
//       std::bind(&OrientRobot::amclCallback, this, std::placeholders::_1));

//     auto period = std::chrono::duration<double>(1.0 / rate_hz);
//     control_timer_ = this->create_wall_timer(
//       std::chrono::duration_cast<std::chrono::nanoseconds>(period),
//       std::bind(&OrientRobot::controlLoop, this));

//     RCLCPP_INFO(this->get_logger(),
//                 "Target z=%.3f w=%.3f -> yaw=%.3f rad (%.1f deg). Control @ %.1f Hz.",
//                 target_z_, target_w_, target_yaw_, target_yaw_ * 180.0 / M_PI, rate_hz);
//     RCLCPP_INFO(this->get_logger(), "Waiting for /amcl_pose ...");
//   }

// private:
//   double quatToYaw(double x, double y, double z, double w)
//   {
//     tf2::Quaternion q(x, y, z, w);
//     q.normalize();
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//     return yaw;
//   }

//   double normalizeAngle(double a)
//   {
//     while (a >  M_PI) a -= 2.0 * M_PI;
//     while (a < -M_PI) a += 2.0 * M_PI;
//     return a;
//   }

//   void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
//   {
//     std::lock_guard<std::mutex> lock(pose_mutex_);
//     const auto & q = msg->pose.pose.orientation;
//     latest_yaw_ = quatToYaw(q.x, q.y, q.z, q.w);
//     last_pose_time_ = this->now();
//     pose_received_ = true;
//   }

//   void stopAndShutdown(const std::string & reason)
//   {
//     geometry_msgs::msg::Twist stop;
//     for (int i = 0; i < 5; ++i) {
//       cmd_pub_->publish(stop);
//       rclcpp::sleep_for(std::chrono::milliseconds(20));
//     }
//     RCLCPP_INFO(this->get_logger(), "%s Shutting down.", reason.c_str());
//     control_timer_->cancel();
//     rclcpp::shutdown();
//   }

//   void controlLoop()
//   {
//     double current_yaw;
//     rclcpp::Time last_time;
//     bool have_pose;
//     {
//       std::lock_guard<std::mutex> lock(pose_mutex_);
//       current_yaw = latest_yaw_;
//       last_time   = last_pose_time_;
//       have_pose   = pose_received_;
//     }

//     if (!have_pose) {
//       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                            "No /amcl_pose received yet.");
//       return;
//     }

//     double age = (this->now() - last_time).seconds();
//     if (age > pose_timeout_) {
//       RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//         "Pose is stale (%.2fs old). Stopping for safety.", age);
//       geometry_msgs::msg::Twist stop;
//       cmd_pub_->publish(stop);
//       in_tolerance_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
//       return;
//     }

//     double error    = normalizeAngle(target_yaw_ - current_yaw);
//     double abs_err  = std::abs(error);
//     geometry_msgs::msg::Twist cmd;

//     if (abs_err > tolerance_) {
//       // Detect oscillation: sign of error flipped
//       if (last_error_sign_ != 0 && error * last_error_sign_ < 0) {
//         oscillation_count_++;
//         RCLCPP_WARN(this->get_logger(),
//           "Overshoot detected (count=%d). Current err=%.4f", oscillation_count_, error);

//         if (oscillation_count_ >= max_oscillations_) {
//           stopAndShutdown(
//             "Max oscillations reached — accepting current orientation.");
//           return;
//         }
//       }
//       last_error_sign_ = (error > 0) ? 1 : -1;

//       // --- Speed-scaled approach: shrink max speed near target ---
//       // This is the key fix for overshoot from rotational inertia.
//       double dynamic_max = max_w_speed_;
//       if (abs_err < slowdown_zone_) {
//         dynamic_max = max_w_speed_ * (abs_err / slowdown_zone_);
//         dynamic_max = std::max(dynamic_max, 0.05);
//       }

//       double w = kp_ * error;
//       w = std::clamp(w, -dynamic_max, dynamic_max);

//       // Only apply min_w when error is large enough to actually need
//       // a friction-breaking push. Near the target, let w be tiny so the
//       // robot coasts in without overshoot.
//       if (abs_err > min_w_zone_ && std::abs(w) < min_w_) {
//         w = std::copysign(min_w_, error);
//       }

//       cmd.angular.z = w;
//       in_tolerance_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

//       RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 300,
//                   "yaw=%.3f target=%.3f err=%.4f w=%.3f (max=%.2f)",
//                   current_yaw, target_yaw_, error, w, dynamic_max);
//       cmd_pub_->publish(cmd);
//     } else {
//       cmd.angular.z = 0.0;
//       cmd_pub_->publish(cmd);

//       auto now = this->now();
//       if (in_tolerance_since_.nanoseconds() == 0) {
//         in_tolerance_since_ = now;
//         RCLCPP_INFO(this->get_logger(),
//           "Within tolerance (err=%.4f). Holding for %.2fs to confirm...",
//           error, hold_duration_);
//       }

//       double held_for = (now - in_tolerance_since_).seconds();
//       if (held_for >= hold_duration_) {
//         stopAndShutdown("Orientation corrected.");
//       }
//     }
//   }

//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
//   rclcpp::TimerBase::SharedPtr control_timer_;

//   double target_z_, target_w_, target_yaw_;
//   double max_w_speed_, kp_, tolerance_, slowdown_zone_;
//   double min_w_, min_w_zone_, pose_timeout_, hold_duration_;
//   int max_oscillations_;

//   std::mutex pose_mutex_;
//   double latest_yaw_ = 0.0;
//   rclcpp::Time last_pose_time_;
//   bool pose_received_ = false;
//   rclcpp::Time in_tolerance_since_{0, 0, RCL_ROS_TIME};

//   int last_error_sign_ = 0;
//   int oscillation_count_ = 0;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   try {
//     auto node = std::make_shared<OrientRobot>();
//     rclcpp::spin(node);
//   } catch (const std::exception & e) {
//     RCLCPP_FATAL(rclcpp::get_logger("orient_robot"), "Startup failed: %s", e.what());
//     rclcpp::shutdown();
//     return 1;
//   }
//   rclcpp::shutdown();
//   return 0;
// }