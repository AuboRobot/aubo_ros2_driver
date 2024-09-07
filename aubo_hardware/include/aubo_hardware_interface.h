#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <bitset>
#include <algorithm>
#include <utility>
#include <fstream>
// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

// ROS
#include "rclcpp/macros.hpp"
#include <rclcpp/logger.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

//#include <aubo_dashboard_msgs/msg/robot_mode.h>
#include <aubo/robot/robot_state.h>
#include "aubo_sdk/rtde.h"
#include "aubo_sdk/rpc.h"
#include "serviceinterface.h"
#include "thread"

using namespace arcs::common_interface;
using namespace arcs::aubo_sdk;
using RtdeRecipeMap =
    std::unordered_map<int, arcs::common_interface::RtdeRecipe>;

namespace aubo_hardware {
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

    hardware_interface::return_type read(const rclcpp::Time &time,
                                         const rclcpp::Duration &period) final;
    hardware_interface::return_type write(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) final;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

    void readActualQ();
    void readIO();
    void getToolIOType(RpcClientPtr rpc_cli);
    void setInput(RtdeClientPtr cli);

    bool isServoModeStart();

    bool ServoModeStart();

    int startServoMode();

    int stopServoMode();
    bool check_command_changed(const std::array<double, 6> prev_command, const std::array<double, 6> current_command, double threshold_min_ = 0.0001,double threshold_max_ = 0.16);
    int Servoj(const std::array<double, 6> joint_position_command);

    void configSubscribe(RtdeClientPtr cli);
    void asyncThread();
    static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();

private:
    void initAsyncIO();
    void checkAsyncIO();    
    void updateNonDoubleValues();
    std::shared_ptr<RpcClient> rpc_client_{ nullptr };
    std::shared_ptr<RtdeClient> rtde_client_{ nullptr };
    std::vector<std::string> joint_names_;
    std::mutex rtde_mtx_;
    std::string robot_ip_;
    std::string robot_name_;
    std::string rtu_device_name = "";

    std::array<double, 6> aubo_position_commands_;
    std::array<double, 6> aubo_velocity_commands_;
    double speed_scaling_combined_;
    bool controllers_initialized_;
    bool servo_mode_start_{ false };
    bool initialized_;
    double system_interface_initialized_;
    bool async_thread_shutdown_;

    double servoj_interval_time = 0.05;//运行时间，单位 s值越大,机器臂运动越慢,反之，运动越快; 该参数最优值为连续调用 servoJoint 接口的间隔时间。最大为0.252

    std::atomic<bool> robot_program_running_;
    std::atomic<bool> controller_reset_necessary_{ false };
    //    uint32_t runtime_state_;
    std::atomic<bool> position_controller_running_;
    std::atomic<bool> velocity_controller_running_;
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

    //topic io
    std::vector<double> standard_analog_input_values_{ std::vector<double>(2, 0.) };
    std::vector<double> tool_analog_input_values_{ std::vector<double>(2, 0.) };
    std::vector<double> standard_analog_output_values_{ std::vector<double>(2,
                                                                            0.) };
    uint64_t standard_digital_input_ = 0;
    uint64_t standard_digital_output_ = 0;
    uint64_t tool_digital_input_ = 0;
    uint64_t tool_digital_output_ = 0;

    std::bitset<12> actual_dig_out_bits_;
    std::bitset<12> actual_dig_in_bits_;
    std::array<double, 2> standard_analog_input_;
    std::array<double, 2> standard_analog_output_;
    std::array<double, 2> tool_analog_input_;

    //tool digital type
    std::vector<bool> tool_is_input_{ std::vector<bool>(4, false) };
    int tool_digital_num_ = 4;
    // asynchronous commands
    std::array<double, 12> standard_dig_out_bits_cmd_;
    std::array<double, 2> standard_analog_output_cmd_;
    double io_async_success_;
    std::shared_ptr<std::thread> async_thread_; 
    // payload stuff
    std::array<double, 3> payload_center_of_gravity_;
    double payload_mass_;
    double payload_async_success_;

    //modbus stuff
    // std::string modbus_device_info_;
    double modbus_slave_number_;
    double modbus_signal_address_;
    double modbus_signal_type_;
    double modbus_signal_name_a_d_;
    double modbus_signal_name_get_;
    double modbus_signal_name_set_;
    double modbus_set_signal_value_;
    double modbus_async_success_;

    // copy of non double values
    std::array<double, 12> actual_dig_out_bits_copy_;
    std::array<double, 12> actual_dig_in_bits_copy_;



    RobotModeType robot_mode_ = RobotModeType::NoController;
    SafetyModeType safety_mode_ = SafetyModeType::Normal;
    RuntimeState runtime_state_ = RuntimeState::Stopped;

    // Store time between update loops
    rclcpp::Clock clock_;
    rclcpp::Time prev_read_time_;
    rclcpp::Time curr_read_time_;
    rclcpp::Time curr_write_time_;
    rclcpp::Time prev_write_time_;
    
    std::array<double, 6> prev_position_command_;
};
} // namespace aubo_hardware

#endif
