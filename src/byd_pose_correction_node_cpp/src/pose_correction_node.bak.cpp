#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PoseCorrectionNode: public rclcpp::Node {
  public: explicit PoseCorrectionNode(const geometry_msgs::msg::Pose & goal_pose);

  bool is_success() const;

  private: void amcl_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pose_correction();
  void send_stop_command();
  std::map < std::string, double > calculate_yaw_control_signal(const std::map < std::string, double > & error);
  std::map < std::string, double > calculate_error();
  double limit_velocity(double velocity, double max_velocity);
  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q);

  geometry_msgs::msg::Pose goal_pose_;
  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription < geometry_msgs::msg::PoseStamped > ::SharedPtr pose_sub_;
  geometry_msgs::msg::Pose amcl_pose_;
  bool amcl_pose_received_;

  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  bool success_;

  rclcpp::TimerBase::SharedPtr timer_;

  double prev_error_x_;
  double prev_error_y_;
  double prev_error_yaw_;
  double integral_error_x_;
  double integral_error_y_;
  double integral_error_yaw_;
  double prev_time_;
  double max_angular_velocity_;
};

PoseCorrectionNode::PoseCorrectionNode(const geometry_msgs::msg::Pose & goal_pose): rclcpp::Node("pose_correction_node"),
  goal_pose_(goal_pose),
  amcl_pose_received_(false),
  xy_goal_tolerance_(0.05),
  yaw_goal_tolerance_(0.015),
  success_(false),
  prev_error_x_(0.0),
  prev_error_y_(0.0),
  prev_error_yaw_(0.0),
  integral_error_x_(0.0),
  integral_error_y_(0.0),
  integral_error_yaw_(0.0),
  prev_time_(rclcpp::Clock().now().seconds()),
  max_angular_velocity_(0.2) {
    cmd_vel_pub_ = this -> create_publisher < geometry_msgs::msg::Twist > ("/cmd_vel", 10);
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);

    pose_sub_ = this -> create_subscription < geometry_msgs::msg::PoseStamped > (
      "/current_pose", qos, std::bind( & PoseCorrectionNode::amcl_pose_callback, this, std::placeholders::_1));

    auto timer_callback = std::bind( & PoseCorrectionNode::pose_correction, this);
    timer_ = this -> create_wall_timer(std::chrono::milliseconds(50), timer_callback);
  }

bool PoseCorrectionNode::is_success() const {
  return success_;
}


void PoseCorrectionNode::amcl_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  amcl_pose_ = msg -> pose;
  amcl_pose_received_ = true;
  // RCLCPP_INFO(this -> get_logger(), "Received pose");
}

void PoseCorrectionNode::pose_correction() {
  if (!amcl_pose_received_) {
    RCLCPP_INFO(this -> get_logger(), "Waiting for initial pose...");
  } else {
    // RCLCPP_INFO(this -> get_logger(), "Pose correction running...");

    auto error = calculate_error();
    auto control_signal = calculate_yaw_control_signal(error);

    if (std::abs(error["yaw"]) < yaw_goal_tolerance_) {
      success_ = true;
      RCLCPP_INFO(this -> get_logger(), "Yaw correction successful.");
      send_stop_command();
      timer_ -> cancel();
    } else {
      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.angular.z = limit_velocity(control_signal["angular"], max_angular_velocity_);
      cmd_vel_pub_ -> publish(cmd_vel_msg);
    }
  }
}

void PoseCorrectionNode::send_stop_command() {
  geometry_msgs::msg::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.angular.z = 0.0;
  cmd_vel_pub_ -> publish(stop_msg);
}

std::map < std::string, double > PoseCorrectionNode::calculate_yaw_control_signal(const std::map < std::string, double > & error) {
  double yaw_error = error.at("yaw");

  if (yaw_error > M_PI) {
    yaw_error -= 2 * M_PI;
  } else if (yaw_error < -M_PI) {
    yaw_error += 2 * M_PI;
  }

  double angular = yaw_error * 1.75;
  return {
    {
      "angular",
      angular
    }
  };
}

std::map < std::string, double > PoseCorrectionNode::calculate_error() {
  std::map < std::string, double > error = {
    {
      "x", 0.0
    },
    {
      "y", 0.0
    },
    {
      "yaw", 0.0
    }
  };

  double goal_yaw = quaternion_to_yaw(goal_pose_.orientation);
  double current_yaw = quaternion_to_yaw(amcl_pose_.orientation);
  double yaw_error = goal_yaw - current_yaw;

  if (yaw_error > M_PI) {
    yaw_error -= 2 * M_PI;
  } else if (yaw_error < -M_PI) {
    yaw_error += 2 * M_PI;
  }

  error["yaw"] = yaw_error;
  // RCLCPP_INFO(this -> get_logger(), "Error: %f", yaw_error);
  return error;
}

double PoseCorrectionNode::limit_velocity(double velocity, double max_velocity) {
  if (velocity > max_velocity) {
    return max_velocity;
  } else if (velocity < -max_velocity) {
    return -max_velocity;
  } else {
    return velocity;
  }
}

double PoseCorrectionNode::quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q) {
  tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char * argv[]) {
   if (argc < 5) {
    std::cerr << "Usage: ros2 run <your_package_name> pose_correction_node x y z w" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = std::stod(argv[1]);
  goal_pose.position.y = std::stod(argv[2]);
  goal_pose.orientation.z = std::stod(argv[3]);
  goal_pose.orientation.w = std::stod(argv[4]);

  auto pose_correction_node = std::make_shared < PoseCorrectionNode > (goal_pose);

  while (rclcpp::ok() && !pose_correction_node -> is_success()) {
    rclcpp::spin_some(pose_correction_node);
  }

  rclcpp::shutdown();
  return 0;
}