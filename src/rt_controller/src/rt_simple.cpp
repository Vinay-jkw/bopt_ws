#include <cmath>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "rt_interfaces/msg/rt_command_stamped.hpp"

class ForkliftController : public rclcpp::Node
{
public:
    ForkliftController()
    : Node("forklift_controller")
    {
        // ---------------- PARAMETERS ----------------

        this->declare_parameter<double>("carriage_max", 1.6);
        this->declare_parameter<double>("mast_max", 2.0);

        this->declare_parameter<double>("max_velocity", 1.6);
        this->declare_parameter<double>("min_velocity", -1.6);

        this->declare_parameter<double>("max_steer_angle", 1.57);
        this->declare_parameter<double>("min_steer_angle", -1.57);

        this->declare_parameter<double>("max_reach", 0.8);
        this->declare_parameter<double>("min_reach", 0.0);

        this->declare_parameter<double>("max_tilt", 0.05236);
        this->declare_parameter<double>("min_tilt", -0.08727);

        this->declare_parameter<double>("max_height", 5.6);
        this->declare_parameter<double>("min_height", 0.0);

        this->get_parameter("carriage_max", carriage_max_);
        this->get_parameter("mast_max", mast_max_);

        this->get_parameter("max_velocity", max_velocity_);
        this->get_parameter("min_velocity", min_velocity_);

        this->get_parameter("max_steer_angle", max_steer_angle_);
        this->get_parameter("min_steer_angle", min_steer_angle_);

        this->get_parameter("max_reach", max_reach_);
        this->get_parameter("min_reach", min_reach_);

        this->get_parameter("max_tilt", max_tilt_);
        this->get_parameter("min_tilt", min_tilt_);

        this->get_parameter("max_height", max_height_);
        this->get_parameter("min_height", min_height_);

        // ---------------- SUBSCRIBER ----------------

        cmd_sub_ =
            this->create_subscription<rt_interfaces::msg::RtCommandStamped>(
                "/rt_controller/cmd_vel",
                10,
                std::bind(
                    &ForkliftController::cmdCallback,
                    this,
                    std::placeholders::_1));

        // ---------------- TRACTION ----------------

        traction_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/traction_joint_controller/commands",
                10);

        // ---------------- STEERING ----------------

        steering_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/steering_joint_controller/commands",
                10);

        // ---------------- REACH ----------------

        reach_pub_ =
            this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/reach_joint_controller/joint_trajectory",
                10);

        // ---------------- MAST 1 ----------------

        mast_pub_ =
            this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/mast_joint_controller/joint_trajectory",
                10);

        // ---------------- MAST 2 ----------------

        mast_02_pub_ =
            this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/mast_02_joint_controller/joint_trajectory",
                10);

        // ---------------- CARRIAGE ----------------

        carriage_pub_ =
            this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/carriage_joint_controller/joint_trajectory",
                10);

        // ---------------- TILT ----------------

        tilt_pub_ =
            this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/carriage_rotation_joint_controller/joint_trajectory",
                10);
    }

private:

    // =========================================================
    // SUBSCRIBERS
    // =========================================================

    rclcpp::Subscription<rt_interfaces::msg::RtCommandStamped>::SharedPtr
        cmd_sub_;

    // =========================================================
    // PUBLISHERS
    // =========================================================

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        traction_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        steering_pub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        reach_pub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        mast_pub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        mast_02_pub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        carriage_pub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        tilt_pub_;

    // =========================================================
    // PARAMETERS
    // =========================================================

    double carriage_max_;
    double mast_max_;

    double max_velocity_;
    double min_velocity_;

    double max_steer_angle_;
    double min_steer_angle_;

    double max_reach_;
    double min_reach_;

    double max_tilt_;
    double min_tilt_;

    double max_height_;
    double min_height_;

    // =========================================================
    // UNIVERSAL TRAJECTORY PUBLISHER
    // =========================================================

    void publishTrajectory(
        const std::string & joint_name,
        double position,
        double velocity,
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub)
    {
        trajectory_msgs::msg::JointTrajectory traj_msg;

        traj_msg.joint_names.push_back(joint_name);

        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.positions.push_back(position);
        point.velocities.push_back(velocity);

        point.time_from_start.sec = 3;

        traj_msg.points.push_back(point);

        pub->publish(traj_msg);
    }

    // =========================================================
    // TRACTION
    // =========================================================

    void publishTraction(double velocity)
    {
        velocity =
            std::clamp(
                velocity,
                min_velocity_,
                max_velocity_);

        std_msgs::msg::Float64MultiArray msg;

        msg.data = {velocity};

        traction_pub_->publish(msg);
    }

    // =========================================================
    // STEERING
    // =========================================================

    void publishSteering(double angle)
    {
        angle =
            std::clamp(
                angle,
                min_steer_angle_,
                max_steer_angle_);

        std_msgs::msg::Float64MultiArray msg;

        msg.data = {-angle};

        steering_pub_->publish(msg);
    }

    // =========================================================
    // REACH
    // =========================================================

    void publishReach(double reach_distance)
    {
        reach_distance =
            std::clamp(
                reach_distance,
                min_reach_,
                max_reach_);

        publishTrajectory(
            "reach_joint",
            reach_distance,
            0.04,
            reach_pub_);
    }

    // =========================================================
    // TILT
    // =========================================================

    void publishTilt(double tilt_angle)
    {
        tilt_angle =
            std::clamp(
                tilt_angle,
                min_tilt_,
                max_tilt_);

        publishTrajectory(
            "carriage_rotation_joint",
            tilt_angle,
            0.05,
            tilt_pub_);
    }

    // =========================================================
    // LIFT LOGIC
    // =========================================================

    void publishLift(double lift_height)
    {
        lift_height =
            std::clamp(
                lift_height,
                min_height_,
                max_height_);

        double carriage_cmd = 0.0;
        double mast_cmd = 0.0;
        double mast_02_cmd = 0.0;

        // -----------------------------------------------------
        // PHASE 1
        // FREE LIFT
        // -----------------------------------------------------

        if (lift_height <= carriage_max_)
        {
            carriage_cmd = lift_height;

            mast_cmd = 0.0;
            mast_02_cmd = 0.0;
        }

        // -----------------------------------------------------
        // PHASE 2
        // MAST EXTENSION
        // mast_02 follows mast 1:1
        // -----------------------------------------------------

        else
        {
            carriage_cmd = carriage_max_;

            double remaining_height =
                lift_height - carriage_max_;

            mast_cmd =
                remaining_height / 2.0;

            mast_cmd =
                std::clamp(
                    mast_cmd,
                    0.0,
                    mast_max_);

            mast_02_cmd = mast_cmd;
        }

        // -----------------------------------------------------
        // PUBLISH CARRIAGE
        // -----------------------------------------------------

        publishTrajectory(
            "carriage_joint",
            carriage_cmd,
            0.03,
            carriage_pub_);

        // -----------------------------------------------------
        // PUBLISH MAST 1
        // -----------------------------------------------------

        publishTrajectory(
            "mast_joint",
            mast_cmd,
            0.03,
            mast_pub_);

        // -----------------------------------------------------
        // PUBLISH MAST 2
        // -----------------------------------------------------

        publishTrajectory(
            "mast_02_joint",
            mast_02_cmd,
            0.03,
            mast_02_pub_);
    }

    // =========================================================
    // COMMAND CALLBACK
    // =========================================================

    void cmdCallback(
        const rt_interfaces::msg::RtCommandStamped::SharedPtr msg)
    {
        publishTraction(msg->traction_velocity);

        publishSteering(msg->steering_angle);

        publishLift(msg->lift_height);

        publishReach(msg->reach_distance);

        publishTilt(msg->tilt_angle);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ForkliftController>());

    rclcpp::shutdown();

    return 0;
}