#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class LastAMCLPosePublisher : public rclcpp::Node
{
public:
  LastAMCLPosePublisher() : Node("last_amcl_pose_publisher")
  {
    last_amcl_pose_ = nullptr;
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&LastAMCLPosePublisher::amcl_pose_callback, this, std::placeholders::_1));
    last_amcl_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/last_amcl_pose", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                      std::bind(&LastAMCLPosePublisher::publish_last_amcl_pose, this));
  }

private:
  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    last_amcl_pose_ = msg;
  }

  void publish_last_amcl_pose()
  {
    if (last_amcl_pose_ != nullptr)
    {
      RCLCPP_INFO(this->get_logger(), "Publishing AMCL pose: %s", last_amcl_pose_->pose.pose.position.x);
      last_amcl_pose_pub_->publish(*last_amcl_pose_);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr last_amcl_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_pose_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LastAMCLPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
