#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class YawAdjustmentNode: public rclcpp::Node {
  public:
    explicit YawAdjustmentNode(double yaw_correction);

    bool is_success() const;

  private:
    void amcl_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void adjust_yaw();
    void send_stop_command();
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q);
    double wrap_angle(double angle);
    void check_timeout();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    bool amcl_pose_received_;
    bool success_;
    double yaw_correction_;
    double yaw_goal_;
    geometry_msgs::msg::Pose amcl_pose_;
    rclcpp::Time start_time_; 

    rclcpp::TimerBase::SharedPtr timer_;
    const rclcpp::Duration timeout_duration_ = rclcpp::Duration::from_seconds(34); // 45 seconds timeout
};

YawAdjustmentNode::YawAdjustmentNode(double yaw_correction) 
  : rclcpp::Node("yaw_adjustment_node"), yaw_correction_(yaw_correction), amcl_pose_received_(false), success_(false) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/current_pose", qos, std::bind(&YawAdjustmentNode::amcl_pose_callback, this, std::placeholders::_1));

    auto timer_callback = std::bind(&YawAdjustmentNode::adjust_yaw, this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);

    start_time_ = this->now(); // Initialize start time
}

bool YawAdjustmentNode::is_success() const {
  return success_;
}

void YawAdjustmentNode::amcl_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    amcl_pose_ = msg->pose; // Add this line
    if(!amcl_pose_received_) {
        double current_yaw = quaternion_to_yaw(msg->pose.orientation);
        yaw_goal_ = wrap_angle(current_yaw + yaw_correction_);
        amcl_pose_received_ = true;
    }
}

void YawAdjustmentNode::adjust_yaw() {
    if (!amcl_pose_received_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for initial pose...");
        return;
    }

    // Check for timeout
    if (this->now() - start_time_ > timeout_duration_) {
        // RCLCPP_ERROR(this->get_logger(), "Yaw adjustment timed out.");
        success_ = true;
        send_stop_command();
        timer_->cancel();
        return; // Optionally, set `success_` to false here to indicate timeout
    }

    double current_yaw = quaternion_to_yaw(amcl_pose_.orientation);
    double error_yaw = wrap_angle(yaw_goal_ - current_yaw);

    if (std::abs(error_yaw) < 0.01) {  // some tolerance
        success_ = true;
        RCLCPP_INFO(this->get_logger(), "Yaw adjustment successful.");
        send_stop_command();
        timer_->cancel();
    } else {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.angular.z = 1.0 * error_yaw; // proportional control
        cmd_vel_pub_->publish(cmd_vel_msg);
    }
}

void YawAdjustmentNode::send_stop_command() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_msg);
}

double YawAdjustmentNode::quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q) {
    tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return yaw;
}

double YawAdjustmentNode::wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, char * argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: ros2 run <your_package_name> yaw_adjustment_node yaw_correction" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    double yaw_correction = std::stod(argv[1]);

    auto node = std::make_shared<YawAdjustmentNode>(yaw_correction);

    while (rclcpp::ok() && !node->is_success()) {
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
