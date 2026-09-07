#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>

using namespace std::chrono_literals;

class GroundTruthNode : public rclcpp::Node
{
public:

    GroundTruthNode()
        : Node("ground_truth_node")
    {
        pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/ground_truth/pose",
                10);

        ignition_topic_ =
            "/world/empty_world/pose/info";

        if (!ignition_node_.Subscribe(
                ignition_topic_,
                &GroundTruthNode::poseCallback,
                this))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to subscribe to Ignition topic: %s",
                ignition_topic_.c_str());

            throw std::runtime_error(
                "Failed to subscribe to Ignition Pose_V");
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Ground truth node started");

        RCLCPP_INFO(
            this->get_logger(),
            "Ignition topic: %s",
            ignition_topic_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Tracking model: JKW_BOPT");

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: /ground_truth/pose");
    }

private:

    void poseCallback(
        const ignition::msgs::Pose_V &_msg)
    {
        for (int i = 0; i < _msg.pose_size(); ++i)
        {
            const auto &pose = _msg.pose(i);

            if (pose.name() != "JKW_BOPT")
            {
                continue;
            }

            geometry_msgs::msg::PoseStamped ros_pose;

            /*
             * Use ROS time for the outgoing message.
             */
            ros_pose.header.stamp =
                this->get_clock()->now();

            ros_pose.header.frame_id =
                "world";

            /*
             * Position
             */
            ros_pose.pose.position.x =
                pose.position().x();

            ros_pose.pose.position.y =
                pose.position().y();

            ros_pose.pose.position.z =
                pose.position().z();

            /*
             * Orientation
             */
            ros_pose.pose.orientation.x =
                pose.orientation().x();

            ros_pose.pose.orientation.y =
                pose.orientation().y();

            ros_pose.pose.orientation.z =
                pose.orientation().z();

            ros_pose.pose.orientation.w =
                pose.orientation().w();

            /*
             * Publish
             */
            pose_pub_->publish(ros_pose);

            /*
             * Optional terminal output.
             *
             * Print at approximately 1 Hz instead of
             * flooding the terminal.
             */
            static auto last_print =
                std::chrono::steady_clock::now();

            auto now =
                std::chrono::steady_clock::now();

            if (now - last_print >= 1s)
            {
                double yaw =
                    std::atan2(
                        2.0 *
                        (
                            pose.orientation().w() *
                            pose.orientation().z()
                            +
                            pose.orientation().x() *
                            pose.orientation().y()
                        ),
                        1.0 -
                        2.0 *
                        (
                            pose.orientation().y() *
                            pose.orientation().y()
                            +
                            pose.orientation().z() *
                            pose.orientation().z()
                        )
                    );

                constexpr double RAD_TO_DEG =
                    180.0 / M_PI;

                RCLCPP_INFO(
                    this->get_logger(),
                    "GT | x=%.4f y=%.4f yaw=%.2f deg",
                    pose.position().x(),
                    pose.position().y(),
                    yaw * RAD_TO_DEG);

                last_print = now;
            }

            break;
        }
    }

    ignition::transport::Node ignition_node_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::string ignition_topic_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<GroundTruthNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

