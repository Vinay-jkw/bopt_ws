#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class ForkliftOdometry : public rclcpp::Node
{
public:
    ForkliftOdometry()
        : Node(
              "forklift_odometry",
              rclcpp::NodeOptions().append_parameter_override("use_sim_time", true))
    {
        wheel_radius_ = declare_parameter("wheel_radius", 0.165);
        wheelbase_ = declare_parameter("wheelbase", 1.515);

        steering_joint_ =
            declare_parameter("steering_joint", "steer_wheel_joint");

        traction_joint_ =
            declare_parameter("traction_joint", "drive_wheel_joint");

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            50,
            std::bind(&ForkliftOdometry::jointCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
        wv_publisher_ = this->create_publisher<std_msgs::msg::Float64>("wheel_velocity", 10);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(static_cast<rclcpp::Node *>(this));

        last_time_ = this->get_clock()->now();
    }

private:

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        double steering = 0.0;
        double wheel_vel = 0.0;

        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == steering_joint_)
                steering = msg->position[i];

            if (msg->name[i] == traction_joint_)
                wheel_vel = msg->velocity[i];

        }
        // RCLCPP_INFO(
        //         this->get_logger(),
        //         "Publishing ST ODOM -> Velocity: %.3f | Steering: %.3f",
        //         wheel_vel,
        //         steering);

        updateOdometry(wheel_vel, steering);
    }

    void updateOdometry(double wheel_vel, double steering)
    {
        rclcpp::Time now_time = this->get_clock()->now();
        double dt = (now_time - last_time_).seconds();
        last_time_ = now_time;

        if (dt <= 0.0)
            return;

        // wheel linear velocity
        double Vw = wheel_vel * wheel_radius_;
        // RCLCPP_INFO(
        //         this->get_logger(),
        //         "Publishing RT ODOM -> Velocity: %.3f",
        //         Vw);
        // publish wheel velocity
        std_msgs::msg::Float64 wv_msg;
        wv_msg.data = Vw;
        wv_publisher_ -> publish(wv_msg);
        // robot forward velocity
        double V = Vw * std::cos(steering);

        // robot angular velocity
        double W = (Vw  * std::sin(-steering))/ wheelbase_;

        // integrate pose
        x_ += V * std::cos(theta_) * dt;
        y_ += V * std::sin(theta_) * dt;
        theta_ += W * dt;

        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        publishOdometry(V, W, now_time);
    }

    void publishOdometry(double V, double W, rclcpp::Time stamp)
    {
        nav_msgs::msg::Odometry odom;

        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = V;
        odom.twist.twist.angular.z = W;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_footprint";

        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;

        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wv_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string steering_joint_;
    std::string traction_joint_;

    double wheel_radius_;
    double wheelbase_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForkliftOdometry>());
    rclcpp::shutdown();
    return 0;
}