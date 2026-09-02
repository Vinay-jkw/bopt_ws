#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>

class CurrentPosePublisher : public rclcpp::Node
{
public:
    CurrentPosePublisher()
    : Node(
        "current_pose_publisher",
        rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)),
      buffer_(this->get_clock()),
      listener_(buffer_)
    {
        publisher_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CurrentPosePublisher::publishPose, this));

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Current Pose Publisher Started");
    }

private:

    void publishPose()
    {
        auto now = this->now();

        geometry_msgs::msg::TransformStamped transformStamped;

        try
        {
            transformStamped =
                buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "TF unavailable: %s",
                ex.what());

            checkTimeout(now);
            return;
        }

        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = now;
        pose.header.frame_id = "map";

        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;

        pose.pose.orientation = transformStamped.transform.rotation;

        publisher_->publish(pose);

        last_time_ = now;
    }

    void checkTimeout(rclcpp::Time now)
    {
        if ((now - last_time_).seconds() > 0.3)
        {
            geometry_msgs::msg::Twist stop;
            cmd_vel_publisher_->publish(stop);

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "No pose update — stopping robot");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CurrentPosePublisher>());

    rclcpp::shutdown();
    return 0;
}