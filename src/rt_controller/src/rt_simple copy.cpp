
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rt_interfaces/msg/rt_command_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class ForkliftController : public rclcpp::Node
{
public:
    ForkliftController()
    : Node("forklift_controller")
    {
        this->declare_parameter<double>("carriage_max", 2.0);
        this->declare_parameter<double>("mast_max", 2.0);
        this->declare_parameter<double>("max_velocity", 1.6);
        this->declare_parameter<double>("min_velocity", -1.6);
        this->declare_parameter<double>("max_steer_angle", 1.57);
        this->declare_parameter<double>("min_steer_angle", -1.57);
        this->declare_parameter<double>("max_reach", 0.8);
        this->declare_parameter<double>("min_reach", 0);
        this->declare_parameter<double>("max_tilt", 0.08);
        this->declare_parameter<double>("min_tilt", -0.03);
        this->declare_parameter<double>("max_height", 6.0);
        this->declare_parameter<double>("min_height", 0);
        this->get_parameter("carriage_max", carriage_max);
        this->get_parameter("mast_max", mast_max);
        this->get_parameter("max_velocity", max_velocity);
        this->get_parameter("min_velocity", min_velocity);
        this->get_parameter("max_steer_angle", max_steer_angle);
        this->get_parameter("min_steer_angle", min_steer_angle);
        this->get_parameter("max_reach", max_reach);
        this->get_parameter("min_reach", min_reach);
        this->get_parameter("max_tilt", max_tilt);
        this->get_parameter("min_tilt", min_tilt);
        this->get_parameter("max_height", max_height);
        this->get_parameter("min_height", min_height);

        cmd_sub_ = create_subscription<rt_interfaces::msg::RtCommandStamped>(
            "/rt_controller/cmd_vel", 10,
            std::bind(&ForkliftController::cmdCallback, this, std::placeholders::_1));
        // joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10,
        //     std::bind(&ForkliftController::jointStateCallback, this, std::placeholders::_1));
        vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/traction_joint_controller/commands",
            10);

        steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/steering_joint_controller/commands",
            10);
        mast_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/mast_joint_controller/commands",
            10);
        reach_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/reach_joint_controller/joint_trajectory",
            10);
        carriage_tilt_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/carriage_rotation_joint_controller/commands",
            10);
        carriage_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/carriage_joint_controller/commands",
            10);
                // ---------------- CONTROL LOOP ----------------
        // control_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(20),
        //     std::bind(&ForkliftController::controlLoop, this));
    }

private:
    rclcpp::Subscription<rt_interfaces::msg::RtCommandStamped>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mast_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr reach_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr carriage_tilt_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr carriage_pub_;
    // rclcpp::TimerBase::SharedPtr control_timer_;
        // ---------------- PARAMETERS ----------------

    double carriage_max;
    double mast_max;
    double max_velocity;
    double min_velocity;
    double max_reach;
    double min_reach;
    double max_tilt;
    double min_tilt;
    double max_height;
    double min_height;
    double max_steer_angle;
    double min_steer_angle;

    // ---------------- STATE ----------------
    double target_reach_ = 0.0;
    double current_reach_ = 0.0;
    double current_velocity_ = 0.0;

    // ---------------- GAINS ----------------
    double kp_ = 3000.0;
    double kd_ = 400.0;
    double lift_height = 0.0;
    double tilt_angle = 0.0;
    double reach_distance = 0.0;

    // ---------------- TRACTION ----------------

    void publishTraction(double velocity)
    {   
        velocity = std::clamp(velocity, min_velocity, max_velocity);
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {velocity};

        vel_pub_->publish(msg);

        // RCLCPP_INFO(this->get_logger(),
        //             "Published traction: %.3f",
        //             velocity);
    }

    // ---------------- STEERING ----------------

    void publishSteering(double angle)
    {
        angle = std::clamp(angle, min_steer_angle, max_steer_angle);
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {-angle};

        steer_pub_->publish(msg);

        // RCLCPP_INFO(this->get_logger(),
        //             "Published steering: %.3f",
        //             angle);
    }

        // ---------------- REACH ----------------

    void publishReach(double reach_distance)
    {
        reach_distance = std::clamp(reach_distance, min_reach, max_reach);
        trajectory_msgs::msg::JointTrajectory traj_msg;

        traj_msg.joint_names.push_back("reach_joint");

        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.positions.push_back(reach_distance);
        point.velocities.push_back(0.04);

        point.time_from_start.sec = 3;

        traj_msg.points.push_back(point);

        reach_pub_->publish(traj_msg);
    }

        // ---------------- TILT ----------------

    void publishTilt(double tilt_angle)
    {
        tilt_angle = std::clamp(tilt_angle, min_tilt, max_tilt);
        std_msgs::msg::Float64MultiArray tilt_msg;
        tilt_msg.data = {tilt_angle};
        carriage_tilt_pub_->publish(tilt_msg);
    }

        // ---------------- Lift ----------------

    void publishLift(double lift_height)
    {   
        lift_height = std::clamp(lift_height, min_height, max_height);
        double carriage_cmd = 0.0;
        double mast_cmd = 0.0;

        if (lift_height <= carriage_max)
        {
            carriage_cmd = lift_height;
            mast_cmd = 0.0;
        }
        else
        {
            carriage_cmd = carriage_max;
            mast_cmd = (lift_height - carriage_max)/2;

            // clamp mast
            if (mast_cmd > mast_max)
                mast_cmd = mast_max;
        }

        std_msgs::msg::Float64MultiArray carriage_msg;
        carriage_msg.data = {carriage_cmd};
        carriage_pub_->publish(carriage_msg);

        // ---- publish mast ----
        std_msgs::msg::Float64MultiArray mast_msg;
        mast_msg.data = {mast_cmd};
        mast_pub_->publish(mast_msg);
    }


    // ---------------- CMD CALLBACK ----------------

    void cmdCallback(const rt_interfaces::msg::RtCommandStamped::SharedPtr msg)
    { // degrees for CAN node
        // RCLCPP_INFO(this->get_logger(),
        //             "Published traction command: %.3f",
        //             msg->traction_velocity);
        // RCLCPP_INFO(this->get_logger(),
        //             "Published steering command: %.3f",
        //             msg->steering_angle);
        publishTraction(msg->traction_velocity);
        publishSteering(msg->steering_angle);
        publishLift(msg -> lift_height);
        publishReach(msg->reach_distance);
        publishTilt(msg->tilt_angle);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "mast_joint")
            {
                current_reach_ = msg->position[i];
                current_velocity_ = msg->velocity[i];
                break;
            }
        }
    }

    // void controlLoop()
    // {
    //     double error = target_reach_ - current_reach_;
    //     double velocity = current_velocity_;

    //     // -------- BASE PD --------
    //     double effort = kp_ * error - kd_ * velocity;

    //     // -------- NONLINEAR BOOST --------
    //     if (std::abs(error) > 0.2)
    //     {
    //         effort *= 2.0;
    //     }

    //     // -------- HOLD MODE (ANTI-DRIFT) --------
    //     if (std::abs(error) < 0.01)
    //     {
    //         effort = -kd_ * velocity * 3.0;
    //     }

    //     // -------- CLAMP --------
    //     effort = std::clamp(effort, -10000.0, 10000.0);

    //     std_msgs::msg::Float64MultiArray msg;
    //     msg.data = {effort};
    //     reach_pub_->publish(msg);

    //     // -------- DEBUG --------
    //     RCLCPP_INFO(this->get_logger(),
    //         "Target: %.3f Current: %.3f Vel: %.3f Effort: %.2f",
    //         target_reach_, current_reach_, velocity, effort);
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForkliftController>());
    rclcpp::shutdown();
    return 0;
}