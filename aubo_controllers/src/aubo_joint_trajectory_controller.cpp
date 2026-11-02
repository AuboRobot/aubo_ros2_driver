#include "aubo_controllers/aubo_joint_trajectory_controller.h"
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace aubo_driver {
controller_interface::InterfaceConfiguration
AuboJointTrajectoryController::state_interface_configuration() const
{
    auto config = JointTrajectoryController::state_interface_configuration();
    config.names.emplace_back("motion_status/execution_state");
    return config;
}

controller_interface::CallbackReturn AuboJointTrajectoryController::on_activate(
    const rclcpp_lifecycle::State &state)
{
    execution_state_interface_ = nullptr;
    for (const auto &interface : state_interfaces_) {
        if (interface.get_name() == "motion_status/execution_state") {
            execution_state_interface_ = &interface;
            break;
        }
    }
    if (!execution_state_interface_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing motion_status/execution_state interface");
        return controller_interface::CallbackReturn::ERROR;
    }
    return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type AuboJointTrajectoryController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        return controller_interface::return_type::OK;
    if (execution_state_interface_ && execution_state_interface_->get_value() == 0.0)
        return JointTrajectoryController::update(time, period);

    // Do this before the base controller can declare the last point successful.
    // Humble ControllerManager::write() does not propagate hardware failures to
    // JTC's action. Hardware state survives on_error()/SDK cleanup for this check.
    const auto active_goal = *rt_active_goal_.readFromRT();
    if (active_goal) {
        auto result = std::make_shared<FollowJTrajAction::Result>();
        result->error_code = FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = execution_state_interface_ &&
            execution_state_interface_->get_value() == 1.0
            ? "AUBO motion paused by safety state or unavailable feedback; submit a new trajectory after recovery"
            : "AUBO hardware execution fault or invalid motion status";
        active_goal->setAborted(result);
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
        rt_has_pending_goal_ = false;
        RCLCPP_WARN(get_node()->get_logger(), "%s", result->error_string.c_str());
    }

    read_state_from_state_interfaces(state_current_);
    // Do not overwrite a new action while its acceptance callback is still
    // installing it. It will be aborted by the next blocked update if needed.
    if (!rt_has_pending_goal_) {
        // Reuse the preallocated buffers while blocked; reset() reallocates.
        traj_msg_external_point_ptr_.initRT(set_hold_position());
    }
    for (std::size_t i = 0; i < dof_; ++i) {
        if (has_position_command_interface_)
            joint_command_interface_[0][i].get().set_value(state_current_.positions[i]);
        if (has_velocity_command_interface_)
            joint_command_interface_[1][i].get().set_value(0.0);
        if (has_acceleration_command_interface_)
            joint_command_interface_[2][i].get().set_value(0.0);
        if (has_effort_command_interface_)
            joint_command_interface_[3][i].get().set_value(0.0);
    }
    last_commanded_state_ = state_current_;
    std::fill(last_commanded_state_.velocities.begin(), last_commanded_state_.velocities.end(), 0.0);
    std::fill(last_commanded_state_.accelerations.begin(), last_commanded_state_.accelerations.end(), 0.0);
    return controller_interface::return_type::OK;
}
}  // namespace aubo_driver
PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboJointTrajectoryController,
                       controller_interface::ControllerInterface)
