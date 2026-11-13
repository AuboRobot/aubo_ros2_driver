#ifndef CONTROLLER_STOPPER_H
#define CONTROLLER_STOPPER_H

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "aubo_msgs/msg/robot_manage_state.hpp"
#include "aubo_msgs/srv/json_rpc.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aubo_driver {

class ControllerStopper
{
public:
    explicit ControllerStopper(const rclcpp::Node::SharedPtr &node);
    ControllerStopper() = delete;
    ~ControllerStopper() = default;

    static bool isRunningMode(const std::string &jsonrpc_result);

private:
    void robotManageStateCallback(
        const aubo_msgs::msg::RobotManageState::ConstSharedPtr message);
    void findAndStopMotionControllers();
    void restartStoppedControllers();
    bool claimsMotionInterface(
        const controller_manager_msgs::msg::ControllerState &controller) const;

    void pollHardware();
    void onRobotMode(bool running);
    void onHardwareState(bool running, uint8_t state_id);
    void rememberActiveControllers();
    void setHardwareState(uint8_t target_id, const std::string &label);
    void activateControllersAfterHardware();
    void finishPoll();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<aubo_msgs::msg::RobotManageState>::SharedPtr
        robot_manage_state_sub_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
        controller_list_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
        controller_switch_client_;
    std::vector<std::string> stopped_controllers_;
    bool handguide_enabled_{ false };

    rclcpp::Client<aubo_msgs::srv::JsonRpc>::SharedPtr jsonrpc_client_;
    rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr
        hardware_list_client_;
    rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
        hardware_state_client_;
    rclcpp::TimerBase::SharedPtr hardware_timer_;
    std::string hardware_component_;
    std::vector<std::string> restore_controllers_;
    bool restore_pending_{ false };
    bool poll_busy_{ false };
    std::chrono::steady_clock::time_point poll_started_{};
    int last_logged_state_{ -1 };
};

} // namespace aubo_driver

#endif
