#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rt_interfaces/msg/rt_command.hpp"

class RobotListener : public rclcpp::Node
{
public:
    RobotListener()
    : Node("robot_listener")
    {
                // Parameters
        wheelbase_ = this->declare_parameter<double>("wheelbase", 1.515);
        wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.165);
        max_steer_ = this->declare_parameter<double>("max_steering", M_PI_2);
        max_speed_ = this->declare_parameter<double>("max_speed", 15.0);

        // RCLCPP_INFO(this->get_logger(),
        //             "Wheelbase: %.3f  Radius: %.3f  MaxSteer: %.3f",
        //             wheelbase_, wheel_radius_, max_steer_);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/input_key/cmd_vel",
            10,
            std::bind(&RobotListener::cmdCallback, this, std::placeholders::_1));

        command_pub_ = this->create_publisher<rt_interfaces::msg::RtCommand>(
            "/rt_controller/cmd_vel_unstamped",
            10);
    }

private:
    void computeSteeringAndTraction(
        double &V,
        double &W,
        double L,
        double R,
        double &steering,
        double &traction)
    {
        const double EPS = 1e-4;

        // ===== Wheel feasibility scaling (IMPORTANT) =====
        double max_wheel_linear = max_speed_ * R;

        double wheel_linear =
            std::sqrt(V * V + (W * L) * (W * L));

        if (wheel_linear > max_wheel_linear && wheel_linear > EPS)
        {
            double scale = max_wheel_linear / wheel_linear;
            V *= scale;
            W *= scale;
        }

        // ===== Dead stop =====
        if (std::abs(V) < EPS && std::abs(W) < EPS)
        {
            steering = last_steering_;
            traction = 0.0;
            return;
        }

        // ===== Pure translation =====
        if (std::abs(W) < EPS)
        {
            steering = 0.0;
            traction = V / R;
            return;
        }

        // ===== Pivot rotation (V ≈ 0) =====
        if (std::abs(V) < 0.05)
        {
            steering = (W > 0 ? 1.0 : -1.0) * max_steer_;

            double wheel_lin = std::abs(W) * L;
            traction = wheel_lin / R;


            return;
        }

        // ===== General motion =====
        steering = std::atan((W * L) / V);

        steering =
            std::clamp(steering, -max_steer_, max_steer_);

        double wheel_lin =
            std::sqrt(V * V + (W * L) * (W * L));

        traction = wheel_lin / R;

        if (V < 0)
            traction *= -1.0;
    }
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double V = msg->linear.x;
        double W = msg->angular.z;
        
        // Clamp linear speed
        
        // RCLCPP_INFO(this->get_logger(),
        //             "Linear: %.3f, Angular: %.3f",
        //             V, W);
        double steering_angle = 0.0;
        double traction_velocity = 0.0;
        computeSteeringAndTraction(
                V,
                W,
                wheelbase_,
                wheel_radius_,
                steering_angle,
                traction_velocity);
        // traction_velocity = 1.0;             // For testing 1 m/s linear speed
        publishCommand(traction_velocity, steering_angle);
        last_steering_ = steering_angle;
    }
    void publishCommand(double traction_velocity,
                    double steering_angle)
    {
        rt_interfaces::msg::RtCommand msg;

        msg.traction_velocity = traction_velocity;
        msg.steering_angle = steering_angle;
        command_pub_->publish(msg);

    }
    double wheelbase_;
    double wheel_radius_;
    double max_steer_;
    double max_speed_;
    double last_steering_ = 0.0;
    // ROS Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<rt_interfaces::msg::RtCommand>::SharedPtr command_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotListener>());
    rclcpp::shutdown();
    return 0;
}