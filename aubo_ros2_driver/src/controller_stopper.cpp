#include "controller_stopper.h"

#include <algorithm>

namespace aubo_driver {

ControllerStopper::ControllerStopper(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
    robot_manage_state_sub_ =
        node_->create_subscription<aubo_msgs::msg::RobotManageState>(
            "/robot_manage/state", 1,
            [this](const aubo_msgs::msg::RobotManageState::ConstSharedPtr
                       message) { robotManageStateCallback(message); });

    controller_list_client_ =
        node_->create_client<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers");
    controller_switch_client_ =
        node_->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
}

void ControllerStopper::robotManageStateCallback(
    const aubo_msgs::msg::RobotManageState::ConstSharedPtr message)
{
    if (message->handguide_enabled && !handguide_enabled_) {
        findAndStopMotionControllers();
    } else if (!message->handguide_enabled && handguide_enabled_) {
        restartStoppedControllers();
    }
    handguide_enabled_ = message->handguide_enabled;
}

void ControllerStopper::findAndStopMotionControllers()
{
    if (!controller_list_client_->service_is_ready() ||
        !controller_switch_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "controller_manager services are not ready.");
        return;
    }

    auto request =
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    controller_list_client_->async_send_request(
        request,
        [this](rclcpp::Client<
               controller_manager_msgs::srv::ListControllers>::SharedFuture
                   future) {
            stopped_controllers_.clear();
            for (const auto &controller : future.get()->controller) {
                if (controller.state == "active" &&
                    claimsMotionInterface(controller)) {
                    stopped_controllers_.push_back(controller.name);
                }
            }

            if (stopped_controllers_.empty()) {
                return;
            }

            auto switch_request = std::make_shared<
                controller_manager_msgs::srv::SwitchController::Request>();
            switch_request->deactivate_controllers = stopped_controllers_;
            switch_request->strictness =
                controller_manager_msgs::srv::SwitchController::Request::STRICT;
            switch_request->activate_asap = true;
            switch_request->timeout.sec = 2;

            controller_switch_client_->async_send_request(
                switch_request,
                [this](rclcpp::Client<
                       controller_manager_msgs::srv::SwitchController>::
                           SharedFuture switch_future) {
                    if (!switch_future.get()->ok) {
                        RCLCPP_ERROR(
                            node_->get_logger(),
                            "Failed to deactivate motion controllers before "
                            "handguide mode.");
                    }
                });
        });
}

void ControllerStopper::restartStoppedControllers()
{
    if (stopped_controllers_.empty()) {
        return;
    }
    if (!controller_switch_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "controller_manager switch service is not ready.");
        return;
    }

    auto request =
        std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = stopped_controllers_;
    request->strictness =
        controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 2;

    controller_switch_client_->async_send_request(
        request,
        [this](
            rclcpp::Client<controller_manager_msgs::srv::SwitchController>::
                SharedFuture future) {
            if (!future.get()->ok) {
                RCLCPP_ERROR(node_->get_logger(),
                             "Failed to reactivate motion controllers after "
                             "handguide mode.");
                return;
            }
            stopped_controllers_.clear();
        });
}

bool ControllerStopper::claimsMotionInterface(
    const controller_manager_msgs::msg::ControllerState &controller) const
{
    return std::any_of(
        controller.claimed_interfaces.begin(),
        controller.claimed_interfaces.end(), [](const std::string &name) {
            return name.size() >= 9 &&
                   (name.compare(name.size() - 9, 9, "/position") == 0 ||
                    name.compare(name.size() - 9, 9, "/velocity") == 0);
        });
}

} // namespace aubo_driver
