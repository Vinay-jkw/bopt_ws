#include "rclcpp/rclcpp.hpp"

#include "rt_interfaces/msg/rt_command.hpp"
#include "rt_interfaces/srv/set_hydraulic_value.hpp"

using SetHydraulicValue = rt_interfaces::srv::SetHydraulicValue;

class ManipulatorServiceNode : public rclcpp::Node
{
public:
    ManipulatorServiceNode()
    : Node("manipulator_service_node")
    {
        cmd_pub_ = this->create_publisher<rt_interfaces::msg::RtCommand>(
            "/rt_controller/cmd_vel_unstamped",
            10);

        // Services
        height_srv_ = create_service<SetHydraulicValue>(
            "/set_height",
            std::bind(
                &ManipulatorServiceNode::heightCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        reach_srv_ = create_service<SetHydraulicValue>(
            "/set_reach",
            std::bind(
                &ManipulatorServiceNode::reachCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        tilt_srv_ = create_service<SetHydraulicValue>(
            "/set_tilt",
            std::bind(
                &ManipulatorServiceNode::tiltCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "Manipulator Service Node Started");
    }

private:
    rclcpp::Publisher<rt_interfaces::msg::RtCommand>::SharedPtr cmd_pub_;

    rclcpp::Service<SetHydraulicValue>::SharedPtr height_srv_;
    rclcpp::Service<SetHydraulicValue>::SharedPtr reach_srv_;
    rclcpp::Service<SetHydraulicValue>::SharedPtr tilt_srv_;

    // Store latest values
    double current_height_ = 0.0;
    double current_reach_ = 0.0;
    double current_tilt_ = 0.0;

    void publishCommand()
    {
        rt_interfaces::msg::RtCommand msg;

        msg.traction_velocity = 0.0;        
        msg.steering_angle = 0.0;

        msg.lift_height = current_height_;
        msg.reach_distance = current_reach_;
        msg.tilt_angle = current_tilt_;

        cmd_pub_->publish(msg);

        RCLCPP_INFO(
            get_logger(),
            "Published -> Height: %.2f Reach: %.2f Tilt: %.2f",
            current_height_,
            current_reach_,
            current_tilt_);
    }

    // HEIGHT SERVICE
    void heightCallback(
        const std::shared_ptr<SetHydraulicValue::Request> request,
        std::shared_ptr<SetHydraulicValue::Response> response)
    {
        current_height_ = request->value;

        publishCommand();

        response->success = true;
        response->message = "Height updated";
    }

    // REACH SERVICE
    void reachCallback(
        const std::shared_ptr<SetHydraulicValue::Request> request,
        std::shared_ptr<SetHydraulicValue::Response> response)
    {
        current_reach_ = request->value;

        publishCommand();

        response->success = true;
        response->message = "Reach updated";
    }

    // TILT SERVICE
    void tiltCallback(
        const std::shared_ptr<SetHydraulicValue::Request> request,
        std::shared_ptr<SetHydraulicValue::Response> response)
    {
        current_tilt_ = request->value;

        publishCommand();

        response->success = true;
        response->message = "Tilt updated";
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<ManipulatorServiceNode>());

    rclcpp::shutdown();

    return 0;
}