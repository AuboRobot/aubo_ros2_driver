#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H

// System
#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>
#include <limits>

#include <algorithm>
#include <utility>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <aubo_dashboard_msgs/msg/robot_mode.h>

#include <aubo/robot/robot_state.h>
#include "aubo_sdk/rtde.h"
#include "aubo_sdk/rpc.h"
#include "serviceinterface.h"

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
using RtdeRecipeMap =
    std::unordered_map<int, arcs::common_interface::RtdeRecipe>;

namespace aubo_driver {
class AuboHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(AuboHardwareInterface);
    virtual ~AuboHardwareInterface();

    bool OnActive();
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state);
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &system_info) final;
    std::vector<hardware_interface::StateInterface> export_state_interfaces()
        final;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() final;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) override;

    hardware_interface::return_type read(const rclcpp::Time &time,
                                         const rclcpp::Duration &period) final;
    hardware_interface::return_type write(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) final;

    void readActualQ();

    void setInput(RtdeClientPtr cli);

    bool isServoModeStart();

    bool ServoModeStart();

    int startServoMode();

    int stopServoMode();

    int Servoj(const std::array<double, 6> joint_position_command);

    int speedServo(const std::array<double, 6> joint_velocity_command);

    void configSubscribe(RtdeClientPtr cli);

private:
    bool connectRpcClient();
    bool connectRtdeClient();
    bool probeIoControlLayout();
    void initAsyncSdkCommands();
    void readIoControlConfigStates();
    void readRobotManageStates();
    void readRobotConfigStates();
    void checkAsyncSdkCommands();

    std::shared_ptr<RpcClient> rpc_client_{ nullptr };
    std::shared_ptr<RtdeClient> rtde_client_{ nullptr };
    std::vector<std::string> joint_names_;
    std::mutex rtde_mtx_;
    std::mutex sdk_command_mtx_;
    std::string robot_ip_;
    std::string robot_name_;

    std::array<double, 6> aubo_position_commands_{};
    std::array<double, 6> aubo_velocity_commands_{};
    double speed_scaling_combined_;
    bool controllers_initialized_;
    bool servo_mode_start_{ false };
    int servo_mode_{ 1 };
    bool initialized_;
    bool motion_command_paused_{ false };

    std::atomic<bool> robot_program_running_;
    std::atomic<bool> controller_reset_necessary_{ false };
    //    uint32_t runtime_state_;
    std::atomic<bool> position_controller_running_{ false };
    std::atomic<bool> velocity_controller_running_{ false };
    std::atomic<bool> handguide_mode_active_{ false };
    std::atomic<bool> joint_forward_controller_running_;
    std::atomic<bool> cartesian_forward_controller_running_;
    std::atomic<bool> twist_controller_running_;
    std::atomic<bool> pose_controller_running_;

    // topic1
    int line_{ -1 };
    std::vector<double> actual_q_{ std::vector<double>(6, 0) };
    std::vector<double> joint_velocity_{ std::vector<double>(6, 0) };
    std::array<double, 6> actual_q_copy_;
    std::array<double, 6> joint_velocity_copy_;
    std::vector<double> actual_qd_{ std::vector<double>(6, 0) };
    std::vector<double> target_q_{ std::vector<double>(6, 0) };
    std::vector<double> target_qd_{ std::vector<double>(6, 0) };

    std::vector<double> actual_current_{ std::vector<double>(6, 0.1) };
    std::vector<double> actual_current_e{ std::vector<double>(6, 0) };
    std::vector<double> actual_TCP_pose_{ std::vector<double>(6, 0.) };
    std::vector<double> actual_TCP_speed_{ std::vector<double>(6, 0.) };
    std::vector<double> actual_TCP_force_{ std::vector<double>(6, 0.) };
    std::vector<double> target_TCP_pose_{ std::vector<double>(6, 0.) };
    std::vector<double> target_TCP_speed_{ std::vector<double>(6, 0.) };
    std::vector<double> actual_tool_pose_{ std::vector<double>(6, 0.) };
    Payload actual_payload_{ 0.0, std::vector<double>(3, 0.0),
                             std::vector<double>(3, 0.0),
                             std::vector<double>(6, 0.0) };

    std::uint64_t standard_digital_input_bits_{ 0 };
    std::uint64_t standard_digital_output_bits_{ 0 };
    std::uint64_t configurable_digital_input_bits_{ 0 };
    std::uint64_t configurable_digital_output_bits_{ 0 };
    std::uint64_t tool_digital_input_bits_{ 0 };
    std::uint64_t tool_digital_output_bits_{ 0 };
    std::vector<double> standard_analog_input_values_;
    std::vector<double> standard_analog_output_values_;
    std::vector<double> tool_analog_input_values_;
    std::vector<double> tool_analog_output_values_;
    bool tool_button_status_{ false };
    std::uint64_t handle_status_{ 0 };
    int handle_dev_state_{ 0 };

    std::vector<double> standard_digital_input_states_;
    std::vector<double> standard_digital_output_states_;
    std::vector<double> configurable_digital_input_states_;
    std::vector<double> configurable_digital_output_states_;
    std::vector<double> tool_digital_input_states_;
    std::vector<double> tool_digital_output_states_;
    std::vector<double> standard_analog_input_states_;
    std::vector<double> standard_analog_output_states_;
    std::vector<double> tool_analog_input_states_;
    std::vector<double> tool_analog_output_states_;
    std::vector<double> tool_io_input_states_;
    std::vector<double> tool_digital_input_action_states_;
    std::vector<double> tool_digital_output_runstate_states_;
    std::vector<double> tool_analog_input_domain_states_;
    std::vector<double> tool_analog_output_domain_states_;
    std::vector<double> tool_analog_output_runstate_states_;
    double tool_voltage_output_domain_state_{ 0.0 };
    double tool_button_status_state_{ 0.0 };
    double handle_io_status_state_{ 0.0 };
    double handle_dev_state_state_{ 0.0 };

    std::array<double, 6> tcp_pose_state_{};
    std::array<double, 6> tcp_speed_state_{};
    std::array<double, 6> tcp_force_state_{};
    std::array<double, 6> tool_pose_state_{};

    std::array<double, 6> tcp_offset_state_{};
    double payload_mass_state_{ 0.0 };
    std::array<double, 3> payload_cog_state_{};

    double handguide_enabled_state_{ 0.0 };
    std::array<double, 5> handguide_free_axis_state_{ 1.0, 1.0, 1.0, 1.0,
                                                     1.0 };
    std::array<double, 6> handguide_feature_state_{};

    double digital_output_type_command_;
    double digital_output_pin_command_;
    double digital_output_value_command_;
    double digital_output_trigger_command_;
    double analog_output_type_command_;
    double analog_output_pin_command_;
    double analog_output_value_command_;
    double analog_output_trigger_command_;
    double tool_voltage_output_domain_command_;
    double tool_io_input_pin_command_;
    double tool_io_input_value_command_;
    double tool_io_input_trigger_command_;
    double tool_io_config_type_command_;
    double tool_io_config_pin_command_;
    double tool_io_config_value_command_;
    double tool_io_config_trigger_command_;
    std::array<double, 6> tcp_offset_commands_{};
    double payload_mass_command_;
    std::array<double, 3> payload_cog_commands_{};
    double handguide_enable_command_;
    std::array<double, 5> handguide_free_axis_commands_{};
    std::array<double, 6> handguide_feature_commands_{};

    RobotModeType robot_mode_ = RobotModeType::NoController;
    SafetyModeType safety_mode_ = SafetyModeType::Normal;
    RuntimeState runtime_state_ = RuntimeState::Stopped;
};
} // namespace aubo_driver

#endif
