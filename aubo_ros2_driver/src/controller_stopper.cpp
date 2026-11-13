#include "controller_stopper.h"

#include <algorithm>

#include "lifecycle_msgs/msg/state.hpp"

namespace aubo_driver {

using lifecycle_msgs::msg::State;

namespace {
constexpr std::chrono::seconds POLL_STALE_AFTER{ 10 };

std::string stateLabel(uint8_t id)
{
    switch (id) {
    case State::PRIMARY_STATE_UNCONFIGURED:
        return "unconfigured";
    case State::PRIMARY_STATE_INACTIVE:
        return "inactive";
    case State::PRIMARY_STATE_ACTIVE:
        return "active";
    case State::PRIMARY_STATE_FINALIZED:
        return "finalized";
    default:
        return "state " + std::to_string(id);
    }
}
} // namespace

ControllerStopper::ControllerStopper(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
    const bool auto_activate =
        node_->declare_parameter<bool>("auto_activate_hardware", false);
    hardware_component_ =
        node_->declare_parameter<std::string>("hardware_component", "aubo");
    const double poll_period =
        node_->declare_parameter<double>("hardware_poll_period", 1.0);
    // Until the hardware has been active once, these are the controllers to restore.
    restore_controllers_ = node_->declare_parameter<std::vector<std::string>>(
        "activate_controllers", std::vector<std::string>{});

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

    if (!auto_activate) {
        return;
    }
    jsonrpc_client_ =
        node_->create_client<aubo_msgs::srv::JsonRpc>("/jsonrpc_service");
    hardware_list_client_ = node_->create_client<
        controller_manager_msgs::srv::ListHardwareComponents>(
        "/controller_manager/list_hardware_components");
    hardware_state_client_ = node_->create_client<
        controller_manager_msgs::srv::SetHardwareComponentState>(
        "/controller_manager/set_hardware_component_state");
    hardware_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(std::max(poll_period, 0.2)),
        [this]() { pollHardware(); });
    RCLCPP_INFO(node_->get_logger(),
                "Hardware '%s' is activated automatically once the robot "
                "reaches Running.",
                hardware_component_.c_str());
}

bool ControllerStopper::isRunningMode(const std::string &jsonrpc_result)
{
    std::string value;
    for (char c : jsonrpc_result) {
        if (c != '"' && c != ' ') value.push_back(c);
    }
    return value == "Running" || value == "8";
}

void ControllerStopper::pollHardware()
{
    if (poll_busy_) {
        if (std::chrono::steady_clock::now() - poll_started_ < POLL_STALE_AFTER) {
            return;
        }
        jsonrpc_client_->prune_pending_requests();
        hardware_list_client_->prune_pending_requests();
        hardware_state_client_->prune_pending_requests();
        controller_list_client_->prune_pending_requests();
        controller_switch_client_->prune_pending_requests();
    }
    if (!jsonrpc_client_->service_is_ready() ||
        !hardware_list_client_->service_is_ready() ||
        !hardware_state_client_->service_is_ready()) {
        return;
    }
    poll_busy_ = true;
    poll_started_ = std::chrono::steady_clock::now();

    auto request = std::make_shared<aubo_msgs::srv::JsonRpc::Request>();
    request->cls = "RobotState";
    request->func = "getRobotModeType";
    request->params = "[]";
    jsonrpc_client_->async_send_request(
        request,
        [this](rclcpp::Client<aubo_msgs::srv::JsonRpc>::SharedFuture future) {
            const auto response = future.get();
            onRobotMode(response->error == "None" &&
                        isRunningMode(response->result));
        });
}

void ControllerStopper::onRobotMode(bool running)
{
    auto request = std::make_shared<
        controller_manager_msgs::srv::ListHardwareComponents::Request>();
    hardware_list_client_->async_send_request(
        request,
        [this, running](
            rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::
                SharedFuture future) {
            for (const auto &component : future.get()->component) {
                if (component.name == hardware_component_) {
                    onHardwareState(running, component.state.id);
                    return;
                }
            }
            if (last_logged_state_ != -2) {
                RCLCPP_WARN(node_->get_logger(),
                            "Hardware component '%s' is not loaded.",
                            hardware_component_.c_str());
                last_logged_state_ = -2;
            }
            finishPoll();
        });
}

void ControllerStopper::onHardwareState(bool running, uint8_t state_id)
{
    if (state_id == State::PRIMARY_STATE_ACTIVE) {
        last_logged_state_ = state_id;
        // Spawners may still be loading controllers after the hardware came up.
        if (restore_pending_) {
            activateControllersAfterHardware();
        } else {
            rememberActiveControllers();
        }
        return;
    }
    if (!running) {
        if (last_logged_state_ != state_id) {
            RCLCPP_INFO(node_->get_logger(),
                        "Hardware '%s' is %s; waiting for the robot to reach "
                        "Running.",
                        hardware_component_.c_str(),
                        stateLabel(state_id).c_str());
            last_logged_state_ = state_id;
        }
        finishPoll();
        return;
    }
    if (state_id == State::PRIMARY_STATE_UNCONFIGURED) {
        setHardwareState(State::PRIMARY_STATE_INACTIVE, "inactive");
        return;
    }
    if (state_id == State::PRIMARY_STATE_INACTIVE) {
        setHardwareState(State::PRIMARY_STATE_ACTIVE, "active");
        return;
    }
    finishPoll();
}

void ControllerStopper::rememberActiveControllers()
{
    if (!controller_list_client_->service_is_ready()) {
        finishPoll();
        return;
    }
    auto request =
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    controller_list_client_->async_send_request(
        request,
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::
                   SharedFuture future) {
            restore_controllers_.clear();
            for (const auto &controller : future.get()->controller) {
                if (controller.state == "active") {
                    restore_controllers_.push_back(controller.name);
                }
            }
            for (const auto &name : stopped_controllers_) {
                if (std::find(restore_controllers_.begin(),
                              restore_controllers_.end(),
                              name) == restore_controllers_.end()) {
                    restore_controllers_.push_back(name);
                }
            }
            finishPoll();
        });
}

void ControllerStopper::setHardwareState(uint8_t target_id,
                                         const std::string &label)
{
    auto request = std::make_shared<
        controller_manager_msgs::srv::SetHardwareComponentState::Request>();
    request->name = hardware_component_;
    request->target_state.id = target_id;
    request->target_state.label = label;
    hardware_state_client_->async_send_request(
        request,
        [this, target_id](
            rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::
                SharedFuture future) {
            const auto response = future.get();
            if (!response->ok) {
                RCLCPP_WARN(node_->get_logger(),
                            "Failed to set hardware '%s' to %s (now %s); "
                            "retrying on the next poll.",
                            hardware_component_.c_str(),
                            stateLabel(target_id).c_str(),
                            stateLabel(response->state.id).c_str());
                last_logged_state_ = response->state.id;
                finishPoll();
                return;
            }
            if (target_id == State::PRIMARY_STATE_INACTIVE) {
                setHardwareState(State::PRIMARY_STATE_ACTIVE, "active");
                return;
            }
            RCLCPP_INFO(node_->get_logger(),
                        "Robot is Running; hardware '%s' activated.",
                        hardware_component_.c_str());
            restore_pending_ = true;
            activateControllersAfterHardware();
        });
}

void ControllerStopper::activateControllersAfterHardware()
{
    if (!controller_list_client_->service_is_ready() ||
        !controller_switch_client_->service_is_ready()) {
        finishPoll();
        return;
    }
    auto request =
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    controller_list_client_->async_send_request(
        request,
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::
                   SharedFuture future) {
            std::vector<std::string> targets;
            std::size_t active = 0;
            for (const auto &controller : future.get()->controller) {
                if (std::find(restore_controllers_.begin(),
                              restore_controllers_.end(),
                              controller.name) == restore_controllers_.end()) {
                    continue;
                }
                if (controller.state == "active") {
                    ++active;
                } else if (controller.state == "inactive") {
                    targets.push_back(controller.name);
                }
            }
            if (active == restore_controllers_.size()) {
                restore_pending_ = false;
            }
            if (targets.empty()) {
                finishPoll();
                return;
            }

            auto switch_request = std::make_shared<
                controller_manager_msgs::srv::SwitchController::Request>();
            switch_request->activate_controllers = targets;
            switch_request->strictness =
                controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
            switch_request->activate_asap = true;
            switch_request->timeout.sec = 2;
            controller_switch_client_->async_send_request(
                switch_request,
                [this, targets](
                    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::
                        SharedFuture switch_future) {
                    std::string names;
                    for (const auto &name : targets) {
                        names += (names.empty() ? "" : ", ") + name;
                    }
                    if (switch_future.get()->ok) {
                        RCLCPP_INFO(node_->get_logger(),
                                    "Activated controllers: %s", names.c_str());
                    } else {
                        RCLCPP_ERROR(node_->get_logger(),
                                     "Failed to activate controllers: %s",
                                     names.c_str());
                    }
                    finishPoll();
                });
        });
}

void ControllerStopper::finishPoll()
{
    poll_busy_ = false;
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
