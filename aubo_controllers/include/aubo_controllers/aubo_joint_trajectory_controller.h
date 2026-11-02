#pragma once
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

namespace aubo_driver {
// Extends Humble JTC with hardware execution-state propagation to its action.
// motion_status/execution_state: 0 ready, 1 paused, 2 latched hardware fault.
class AuboJointTrajectoryController
    : public joint_trajectory_controller::JointTrajectoryController {
public:
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
private:
    const hardware_interface::LoanedStateInterface *execution_state_interface_{nullptr};
};
}  // namespace aubo_driver
