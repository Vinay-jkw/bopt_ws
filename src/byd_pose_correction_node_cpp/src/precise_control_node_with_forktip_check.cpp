#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"  // Include Bool message type
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class VelocityAndSteeringPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pallet_detected_sub;  // Subscription for pallet detection
    rclcpp::TimerBase::SharedPtr timer;
    double velocity;
    double steering_angle;
    double prev_x, prev_y;
    double distance, target_distance;
    std::string state = "Custom";
    bool timer_alive = true;
    double max_velocity = 0.6;
    double min_velocity = 0.1;
    double direction = 1.0;
    bool pallet_detected = false;
    bool stop_due_to_pallet = false;  // New flag to indicate stop reason

public:
    VelocityAndSteeringPublisher() : rclcpp::Node("velocity_and_steering_pub") {
        this->declare_parameter<double>("initial_velocity", 0.0);
        this->declare_parameter<double>("initial_steering_angle", 0.0);
        this->declare_parameter<double>("target_distance", 0.0);

        this->get_parameter("initial_velocity", velocity);
        this->get_parameter("initial_steering_angle", steering_angle);
        this->get_parameter("target_distance", target_distance);
        if (velocity > 0) {
            direction = 1.0;
        } else {
            direction = -1.0;
        }

        velocity_pub = this->create_publisher<std_msgs::msg::Float64>("/velocity", 10);
        steering_pub = this->create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);
        state_pub = this->create_publisher<std_msgs::msg::String>("/state", 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&VelocityAndSteeringPublisher::publish_messages, this));

        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/current_pose", qos, std::bind(&VelocityAndSteeringPublisher::pose_callback, this, std::placeholders::_1));
        pallet_detected_sub = this->create_subscription<std_msgs::msg::Bool>("/forktip_status", qos, std::bind(&VelocityAndSteeringPublisher::pallet_detected_callback, this, std::placeholders::_1));

        prev_x = prev_y = std::numeric_limits<double>::max();
    }

    void pallet_detected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Pallet detection callback received: %s", msg->data ? "true" : "false");
        if (msg->data) {  // If pallet is detected
            pallet_detected = true;
            stop_due_to_pallet = true;  // Set flag when pallet is detected
            RCLCPP_INFO(this->get_logger(), "\ntrue\n");
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        
        if(prev_x == std::numeric_limits<double>::max() && prev_y == std::numeric_limits<double>::max()) {
            prev_x = x;
            prev_y = y;
        }

        double diff_x = x - prev_x;
        double diff_y = y - prev_y;
        distance += sqrt(pow(diff_x, 2) + pow(diff_y, 2));
        prev_x = x;
        prev_y = y;
        // RCLCPP_INFO(this->get_logger(), "Current x: %f, y: %f", x, y);
        // RCLCPP_INFO(this->get_logger(), "Current distance: %f", distance);
    
    }

    void publish_messages() {
        if (pallet_detected || distance >= target_distance) {
            timer->cancel();
            timer_alive = false;
            state = "Stop";

            std_msgs::msg::Float64 zero_velocity_msg;
            zero_velocity_msg.data = 0.0;
            velocity_pub->publish(zero_velocity_msg);

            std_msgs::msg::Float64 zero_steering_msg;
            zero_steering_msg.data = 0.0;
            steering_pub->publish(zero_steering_msg);

            std_msgs::msg::String state_msg;
            state_msg.data = state;
            state_pub->publish(state_msg);

            if (!pallet_detected) {
                stop_due_to_pallet = false;  // Ensure flag is correctly set for distance case
            }

            return;
        }

        if (distance < target_distance/2){
                    velocity = min_velocity + (max_velocity - min_velocity) * distance*2/target_distance;      
                }
                else{
                    velocity = max_velocity + (distance - target_distance/2) * (min_velocity - max_velocity/(target_distance/2));
                }
                if (direction < 0){
                    velocity = -velocity;
                }  
                std_msgs::msg::Float64 velocity_msg;
                velocity_msg.data = velocity;
                velocity_pub->publish(velocity_msg);

                std_msgs::msg::Float64 steering_msg;
                steering_msg.data = steering_angle;
                steering_pub->publish(steering_msg);

                std_msgs::msg::String state_msg;
                state_msg.data = state;
                state_pub->publish(state_msg);
        }    

    bool get_timer_alive() const {
        return timer_alive;
    }
    bool get_stop_due_to_pallet() const {
        return stop_due_to_pallet;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto pub_node = std::make_shared<VelocityAndSteeringPublisher>();
    while (rclcpp::ok() && pub_node->get_timer_alive()) {
        rclcpp::spin_some(pub_node);
    }
    bool result = pub_node->get_stop_due_to_pallet();
    pub_node.reset();
    rclcpp::shutdown();
    return result ? EXIT_SUCCESS : EXIT_FAILURE;  // Return true or false based on stop reason
}
