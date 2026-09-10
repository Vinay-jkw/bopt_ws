#include <cmath>
#include <limits>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

struct Arc {
    double distance;
    double steering_angle;
    double velocity; // Add a velocity field
};


class VelocityAndSteeringPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer;
    double velocity;
    double steering_angle;
    double prev_x, prev_y;
    double distance, target_distance;
    std::vector<Arc> arcs;
    size_t current_arc_index = 0;
    std::string state = "Custom";
    bool timer_alive = true;
    double max_velocity = 0.3;
    double min_velocity = 0.2;
    double direction = 1.0;

public:
    VelocityAndSteeringPublisher() : rclcpp::Node("velocity_and_steering_pub") {
              
        // New part: Declare and read arcs from parameters
        this->declare_parameter<std::vector<double>>("arcs", std::vector<double>());
        std::vector<double> arcs_params;
        this->get_parameter("arcs", arcs_params);
        
        // Assuming arcs_params is a flat list [distance1, angle1, velocity1, distance2, angle2, velocity2, ...]
        for (size_t i = 0; i < arcs_params.size(); i += 3) {
            if (i + 2 < arcs_params.size()) { // Make sure we have triples
                Arc arc;
                arc.distance = arcs_params[i];
                arc.steering_angle = arcs_params[i + 1];
                arc.velocity = arcs_params[i + 2]; // Set the velocity for the arc
                arcs.push_back(arc);
            }
        }
        
        // Make sure to check if arcs is not empty before using its values
        if (!arcs.empty()) {
            target_distance = arcs[0].distance;
            steering_angle = arcs[0].steering_angle;
        } else {
            RCLCPP_ERROR(this->get_logger(), "No arcs provided.");
            // Handle error: Maybe set some default or exit
        }

        
        // Initialize the first arc's target distance and steering angle.
        target_distance = arcs[0].distance;
        steering_angle = arcs[0].steering_angle;
        velocity = arcs[0].velocity;
        if (velocity > 0){
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

        prev_x = prev_y = std::numeric_limits<double>::max();
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
    }

    void publish_messages() {
        if (distance >= target_distance && current_arc_index < arcs.size() - 1) {  
            // Move to the next arc
            current_arc_index++;
            distance = 0; // Reset distance for the next arc
            target_distance = arcs[current_arc_index].distance;
            steering_angle = arcs[current_arc_index].steering_angle;
            velocity = arcs[current_arc_index].velocity;
            if (velocity > 0){
                direction = 1.0;
            } else {            
                direction = -1.0;
            }
            RCLCPP_INFO(this->get_logger(), "dist: '%f', steer: '%f', vel: '%f' ", target_distance, steering_angle, velocity);


        } else if (distance >= target_distance && current_arc_index == arcs.size() - 1) {
            // Last arc completed, stop the robot
            timer->cancel();
            timer_alive = false;
            state = "Stop";

            // Publish zero for velocity and steering before stopping
            std_msgs::msg::Float64 zero_velocity_msg;
            zero_velocity_msg.data = 0.0;
            velocity_pub->publish(zero_velocity_msg);

            std_msgs::msg::Float64 zero_steering_msg;
            zero_steering_msg.data = 0.0;
            steering_pub->publish(zero_steering_msg);

            std_msgs::msg::String state_msg;
            state_msg.data = state;
            state_pub->publish(state_msg);
            return;
        }

        // Update velocity based on the current segment of the arc
        if (distance < target_distance/2){
            velocity = min_velocity + (max_velocity - min_velocity) * distance*2/target_distance;      
        } else {
            velocity = max_velocity + (distance - target_distance/2) * (min_velocity - max_velocity/(target_distance/2));
        }
        if (direction < 0){
            velocity = -velocity;
        }  

        RCLCPP_INFO(this->get_logger(), "vel: '%f', steer: '%f'", velocity, steering_angle);

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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto pub_node = std::make_shared<VelocityAndSteeringPublisher>();

    while (rclcpp::ok() && pub_node->get_timer_alive()) {
        rclcpp::spin_some(pub_node);
    }

    pub_node.reset();
    rclcpp::shutdown();
    return 0;
}

