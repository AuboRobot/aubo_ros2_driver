#include "robot_state_helper.h"

namespace aubo_driver {

RobotStateHelper::RobotStateHelper(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
    node_->declare_parameter("robot_ip", "127.0.0.1");
    node_->declare_parameter("controller_manager_name", "/controller_manager");
    robot_ip_ = node_->get_parameter("robot_ip").as_string();
    controller_manager_name_ =
        node_->get_parameter("controller_manager_name").as_string();

    RCLCPP_INFO(node_->get_logger(),
                "robot_state_helper is reserved for robot state orchestration; "
                "IO/status ROS APIs are provided by ros2_control interfaces.");
}

} // namespace aubo_driver
