#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"  // Include for Bool message
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class VelocityAndSteeringPublisher : public rclcpp::Node {
private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr field_status_sub;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer;
    
    // Variables
    double velocity;
    double steering_angle;
    double prev_x, prev_y;
    double distance, target_distance;
    std::string state = "Custom";
    bool timer_alive = true;
    double max_velocity = 0.6;
    double min_velocity = 0.1;
    double direction = 1.0;
    
    // Field status flag
    bool field_status = false;

public:
    VelocityAndSteeringPublisher() : rclcpp::Node("velocity_and_steering_pub") {
        // Declare and get parameters
        this->declare_parameter<double>("initial_velocity", 0.0);
        this->declare_parameter<double>("initial_steering_angle", 0.0);
        this->declare_parameter<double>("target_distance", 0.0);

        this->get_parameter("initial_velocity", velocity);
        this->get_parameter("initial_steering_angle", steering_angle);
        this->get_parameter("target_distance", target_distance);
        
        // Determine direction based on initial velocity
        direction = (velocity >= 0) ? 1.0 : -1.0;
        velocity = std::abs(velocity);  // Ensure velocity is positive for calculations

        // Initialize publishers
        velocity_pub = this->create_publisher<std_msgs::msg::Float64>("/velocity", 10);
        steering_pub = this->create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);
        state_pub = this->create_publisher<std_msgs::msg::String>("/state", 10);
        
        // Initialize subscribers
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current_pose", qos,
            std::bind(&VelocityAndSteeringPublisher::pose_callback, this, std::placeholders::_1));
        
        field_status_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/sft_field_status", qos,
            std::bind(&VelocityAndSteeringPublisher::field_status_callback, this, std::placeholders::_1));
        
        // Initialize timer to publish messages at 25 Hz (40 ms)
        timer = this->create_wall_timer(std::chrono::milliseconds(40),
            std::bind(&VelocityAndSteeringPublisher::publish_messages, this));

        // Initialize previous positions
        prev_x = prev_y = std::numeric_limits<double>::max();
    }

    // Callback for PoseStamped messages to calculate distance traveled
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        
        if (prev_x == std::numeric_limits<double>::max() && prev_y == std::numeric_limits<double>::max()) {
            prev_x = x;
            prev_y = y;
            return;
        }

        double diff_x = x - prev_x;
        double diff_y = y - prev_y;
        distance += std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
        prev_x = x;
        prev_y = y;

        // Optional: Log current distance
        // RCLCPP_INFO(this->get_logger(), "Current distance: %f", distance);
    }

    // Callback for Field Status messages
    void field_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        field_status = msg->data;
        if (field_status) {
            RCLCPP_INFO(this->get_logger(), "Field status TRUE: Stopping the vehicle.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Field status FALSE: Resuming normal operation.");
        }
    }

    // Function to publish velocity, steering, and state messages
    void publish_messages() {
        if (field_status) {
            // If field status is true, stop the vehicle by setting velocity to zero
            std_msgs::msg::Float64 zero_velocity_msg;
            zero_velocity_msg.data = 0.0;
            velocity_pub->publish(zero_velocity_msg);

            // **Do not modify the steering angle**
            std_msgs::msg::Float64 steering_msg;
            steering_msg.data = steering_angle;  // Keep the current steering angle
            steering_pub->publish(steering_msg);

            // Publish state as "Stopped due to Field Status"
            std_msgs::msg::String state_msg;
            state_msg.data = "Stopped due to Field Status";
            state_pub->publish(state_msg);
            return;  // Early exit since we're stopping
        }

        // Normal operation: Check if target distance is reached
        if (distance >= target_distance) {  // Change this to your desired distance
            if (timer_alive) {
                timer->cancel();
                timer_alive = false;
                state = "Stop";

                // Publish zero for velocity before stopping
                std_msgs::msg::Float64 zero_velocity_msg;
                zero_velocity_msg.data = 0.0;
                velocity_pub->publish(zero_velocity_msg);

                // Publish current steering angle without modification
                std_msgs::msg::Float64 steering_msg;
                steering_msg.data = steering_angle;
                steering_pub->publish(steering_msg);

                // Publish state as "Stop"
                std_msgs::msg::String state_msg;
                state_msg.data = state;
                state_pub->publish(state_msg);
            }
            return;
        }

        // Adjust velocity based on the distance
        if (distance < target_distance / 2) {
            velocity = min_velocity + (max_velocity - min_velocity) * (distance * 2 / target_distance);
        } else {
            // Corrected the velocity calculation formula
            velocity = max_velocity - (max_velocity - min_velocity) * ((distance - target_distance / 2) / (target_distance / 2));
        }

        // Apply direction
        velocity *= direction;

        // Publish velocity
        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = velocity;
        velocity_pub->publish(velocity_msg);

        // Publish steering angle
        std_msgs::msg::Float64 steering_msg;
        steering_msg.data = steering_angle;
        steering_pub->publish(steering_msg);

        // Publish state
        std_msgs::msg::String state_msg;
        state_msg.data = state;
        state_pub->publish(state_msg);
    }

    // Function to check if the timer is still active
    bool get_timer_alive() const {
        return timer_alive;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto pub_node = std::make_shared<VelocityAndSteeringPublisher>();

    // Spin the node until the timer is no longer active
    while (rclcpp::ok() && pub_node->get_timer_alive()) {
        rclcpp::spin_some(pub_node);
    }

    // Optionally, continue spinning to handle field status changes even after stopping
    // If you want the node to respond to field status changes even after stopping the timer,
    // you can uncomment the following lines:

    /*
    if (rclcpp::ok()) {
        rclcpp::spin(pub_node);
    }
    */

    pub_node.reset();
    rclcpp::shutdown();
    return 0;
}
