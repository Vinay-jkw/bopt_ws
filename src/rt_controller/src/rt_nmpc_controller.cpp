#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "rt_interfaces/msg/rt_command.hpp"

class NmpcCommandBridge : public rclcpp::Node
{
public:
    NmpcCommandBridge() : Node("nmpc_command_bridge")
    {
        wheelbase_ = this->declare_parameter<double>("wheelbase", 1.375);
        wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.115);
        max_steer_ = this->declare_parameter<double>("max_steering", M_PI_2);
        max_speed_ = this->declare_parameter<double>("max_speed", 5.0);

        RCLCPP_INFO(this->get_logger(), "NMPC Command Bridge Node Started");

        vel_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/velocity_remapped", 10,
            std::bind(&NmpcCommandBridge::velCallback, this, std::placeholders::_1));

        steer_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/steering_angle", 10,
            std::bind(&NmpcCommandBridge::steerCallback, this, std::placeholders::_1));

        state_sub = this->create_subscription<std_msgs::msg::String>(
            "/state", 10,
            std::bind(&NmpcCommandBridge::stateCallback, this, std::placeholders::_1));

        cmd_pub = this->create_publisher<rt_interfaces::msg::RtCommand>(
            "/rt_controller/cmd_vel_unstamped", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&NmpcCommandBridge::publishCmd, this));

        RCLCPP_INFO(this->get_logger(), "Subscriptions and publisher initialized");
    }

private:

    /*------------------- Robot Parameters -------------------*/

    double wheelbase_;
    double wheel_radius_;
    double max_steer_;
    double max_speed_;

    /*------------------- Internal Variables -------------------*/

    double velocity_ = 0.0;
    double steering_ = 0.0;
    std::string current_state_ = "";

    bool stop_published_ = false;

    /*------------------- Callbacks -------------------*/

    void velCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        velocity_ = msg->data / wheel_radius_;
        // if (msg->data < 0){
        //     velocity_ *= -1;
        // }

        RCLCPP_DEBUG(
            this->get_logger(),
            "Velocity received: %.3f",
            velocity_);
    }

    void steerCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        steering_ = msg->data;

        RCLCPP_DEBUG(
            this->get_logger(),
            "Steering received: %.3f",
            steering_);
    }

    void stateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_state_ = msg->data;

        RCLCPP_DEBUG(
            this->get_logger(),
            "State received: %s",
            current_state_.c_str());
    }

    /*------------------- Steering Mapping -------------------*/

    double map_steering_to_rad(double angle)
    {
        // Fold angles greater than 90°
        if (angle > 90 && angle <= 180)
        {
            angle = 180 - angle;
        }

        // Fold angles less than -90°
        if (angle < -90 && angle >= -180)
        {
            angle = -180 - angle;
        }

        // Convert to radians
        return angle * M_PI / 180.0;
    }

    /*------------------- Command Publisher -------------------*/

    void publishCmd()
    {
        rt_interfaces::msg::RtCommand msg;

        if (current_state_ == "Custom")
        {
            stop_published_ = false;

            msg.traction_velocity = velocity_;
            msg.steering_angle = map_steering_to_rad(steering_);


            RCLCPP_INFO(
                this->get_logger(),
                "Publishing RT Command -> Velocity: %.3f | Steering: %.3f | Lift: %.3f",
                msg.traction_velocity,
                msg.steering_angle,
                msg.lift_height);

            cmd_pub->publish(msg);
        }
        else
        {
            if (!stop_published_)
            {
                msg.traction_velocity = 0.0;
                msg.steering_angle = 0.0; 
                cmd_pub->publish(msg);

                stop_published_ = true;

                RCLCPP_INFO(this->get_logger(), "State not Custom -> STOP command published");
            }
        }
    }

    /*------------------- ROS Interfaces -------------------*/

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;

    rclcpp::Publisher<rt_interfaces::msg::RtCommand>::SharedPtr cmd_pub;

    rclcpp::TimerBase::SharedPtr timer_;
};

/*------------------- Main -------------------*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting NMPC Command Bridge...");

    rclcpp::spin(std::make_shared<NmpcCommandBridge>());

    rclcpp::shutdown();

    return 0;
}