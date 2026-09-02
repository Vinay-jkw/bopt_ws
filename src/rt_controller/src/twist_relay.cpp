#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rt_interfaces/msg/rt_command.hpp"
#include "rt_interfaces/msg/rt_command_stamped.hpp"

class TwistRelayNode : public rclcpp::Node {
public:
    TwistRelayNode() : Node("twist_relay")
    {
        controller_sub_ = this->create_subscription<rt_interfaces::msg::RtCommand>(
            "/rt_controller/cmd_vel_unstamped",
            10,
            std::bind(&TwistRelayNode::controller_twist_callback, this, std::placeholders::_1)
        );
        controller_pub_ = this->create_publisher<rt_interfaces::msg::RtCommandStamped>(
            "/rt_controller/cmd_vel", 10);
        joy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_remapped",
            10,
            std::bind(&TwistRelayNode::joy_twist_callback, this, std::placeholders::_1)
        );
        joy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/input_key/cmd_vel", 10);
    }

private:
    void controller_twist_callback(const rt_interfaces::msg::RtCommand::SharedPtr msg)
    {
        rt_interfaces::msg::RtCommandStamped twist_stamped;
        twist_stamped.header.stamp = this->now();
        twist_stamped.traction_velocity = msg->traction_velocity;
        twist_stamped.steering_angle= msg->steering_angle;
        twist_stamped.lift_height= msg->lift_height;
        twist_stamped.reach_distance= msg->reach_distance;
        twist_stamped.tilt_angle= msg->tilt_angle;
        // RCLCPP_DEBUG(this->get_logger(), "Received RtCommand: traction_velocity=%.2f, steering_angle=%.2f, lift_height=%.2f, reach_distance=%.2f, tilt_angle=%.2f",
        //              msg->traction_velocity, msg->steering_angle, msg->lift_height, msg->reach_distance, msg->tilt_angle);
        std::cout << "Received RtCommand: traction_velocity=" << msg->traction_velocity
                  << ", steering_angle=" << msg->steering_angle
                  << ", lift_height=" << msg->lift_height
                  << ", reach_distance=" << msg->reach_distance
                  << ", tilt_angle=" << msg->tilt_angle << std::endl;
        controller_pub_->publish(twist_stamped);
    }

    void joy_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist;
        twist = *msg;
        joy_pub_->publish(twist);
        // RCLCPP_DEBUG(this->get_logger(), "Received Twist: linear.x=%.2f, angular.z=%.2f",
        //             msg->linear.x, msg->angular.z);
        std::cout << "Received Twist: linear.x=" << msg->linear.x
                  << ", angular.z=" << msg->angular.z << std::endl;
    }

    rclcpp::Subscription<rt_interfaces::msg::RtCommand>::SharedPtr controller_sub_;
    rclcpp::Publisher<rt_interfaces::msg::RtCommandStamped>::SharedPtr controller_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}