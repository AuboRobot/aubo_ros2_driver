#ifndef CONTROLLER_STOPPER_H
#define CONTROLLER_STOPPER_H

#include <memory>
#include <string>
#include <vector>

#include "aubo_msgs/msg/robot_manage_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aubo_driver {

class ControllerStopper
{
public:
    explicit ControllerStopper(const rclcpp::Node::SharedPtr &node);
    ControllerStopper() = delete;
    ~ControllerStopper() = default;

private:
    void robotManageStateCallback(
        const aubo_msgs::msg::RobotManageState::ConstSharedPtr message);
    void findAndStopMotionControllers();
    void restartStoppedControllers();
    bool claimsMotionInterface(
        const controller_manager_msgs::msg::ControllerState &controller) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<aubo_msgs::msg::RobotManageState>::SharedPtr
        robot_manage_state_sub_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
        controller_list_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
        controller_switch_client_;
    std::vector<std::string> stopped_controllers_;
    bool handguide_enabled_{ false };
};

} // namespace aubo_driver

#endif
