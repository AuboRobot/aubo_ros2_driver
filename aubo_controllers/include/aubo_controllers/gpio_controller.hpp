// Copyright (c) 2024 AUBO LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-02-20
 *
 */
//----------------------------------------------------------------------

#ifndef aubo_controllers__GPIO_CONTROLLER_HPP_
#define aubo_controllers__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "std_srvs/srv/trigger.hpp"

#include "controller_interface/controller_interface.hpp"
#include "aubo_msgs/msg/io_states.hpp"
#include "aubo_msgs/msg/tool_data_msg.hpp"
#include "aubo_msgs/srv/set_io.hpp"
#include "aubo_msgs/srv/set_speed_slider_fraction.hpp"
#include "aubo_msgs/srv/set_payload.hpp"
#include "aubo_msgs/srv/add_signal.hpp"
#include "aubo_msgs/srv/delete_signal.hpp"
#include "aubo_msgs/srv/get_signal_status.hpp"
#include "aubo_msgs/srv/set_output_signal.hpp"
#include "aubo_msgs/srv/set_signal_update_frequency.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/bool.hpp"
#include "gpio_controller_parameters.hpp"

namespace aubo_controllers
{
enum CommandInterfaces
{
  DIGITAL_OUTPUTS_CMD = 0u,
  ANALOG_OUTPUTS_CMD = 12,
  // TOOL_VOLTAGE_CMD = 14,
  IO_ASYNC_SUCCESS = 14,
  // TARGET_SPEED_FRACTION_CMD = 22,
  // TARGET_SPEED_FRACTION_ASYNC_SUCCESS = 23,
  // RESEND_ROBOT_PROGRAM_CMD = 24,
  // RESEND_ROBOT_PROGRAM_ASYNC_SUCCESS = 25,
  PAYLOAD_MASS = 15,
  PAYLOAD_COG_X = 16,
  PAYLOAD_COG_Y = 17,
  PAYLOAD_COG_Z = 18,
  PAYLOAD_ASYNC_SUCCESS = 19,
  // MODBUS_DEVICE_INFO = 20,
  MODBUS_SLAVE_NUMBER = 20,
  MODBUD_SIGNAL_ADDRESS = 21,
  MODBUS_SIGNAL_TYPE = 22,
  MODBUS_SIGNAL_NAME_A_D = 23,
  MODBUS_SIGNAL_NAME_GET = 24,
  MODBUS_SIGNAL_NAME_SET = 25,
  MODBUS_SET_SIGNAL_VALUE = 26,
  MODBUS_ASYNC_SUCCESS = 27,

};

enum StateInterfaces
{
  DIGITAL_OUTPUTS = 0u,
  DIGITAL_INPUTS = 12,
  ANALOG_OUTPUTS = 24,
  ANALOG_INPUTS = 26,
  // TOOL_OUTPUT_VOLTAGE = 28,
  TOOL_ANALOG_INPUTS = 28,
  // ROBOT_MODE = 31,
  // SAFETY_MODE = 32,
  INITIALIZED_FLAG = 30,
  // PROGRAM_RUNNING = 33,
};

class GPIOController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  bool setIO(aubo_msgs::srv::SetIO::Request::SharedPtr req, aubo_msgs::srv::SetIO::Response::SharedPtr resp);

  // bool setSpeedSlider(aubo_msgs::srv::SetSpeedSliderFraction::Request::SharedPtr req,
  //                     aubo_msgs::srv::SetSpeedSliderFraction::Response::SharedPtr resp);

  // bool resendRobotProgram(std_srvs::srv::Trigger::Request::SharedPtr req,
  //                         std_srvs::srv::Trigger::Response::SharedPtr resp);

  // bool handBackControl(std_srvs::srv::Trigger::Request::SharedPtr req,
  //                      std_srvs::srv::Trigger::Response::SharedPtr resp);

  bool setPayload(const aubo_msgs::srv::SetPayload::Request::SharedPtr req,
                  aubo_msgs::srv::SetPayload::Response::SharedPtr resp);

  bool modbusAddSignal(const aubo_msgs::srv::AddSignal::Request::SharedPtr req,
                  aubo_msgs::srv::AddSignal::Response::SharedPtr resq);
  bool modbusDeleteSignal(const aubo_msgs::srv::DeleteSignal::Request::SharedPtr req,
                  aubo_msgs::srv::DeleteSignal::Response::SharedPtr resq);
  int32_t modbusGetSignalStatus(const aubo_msgs::srv::GetSignalStatus::Request::SharedPtr req,
                  aubo_msgs::srv::GetSignalStatus::Response::SharedPtr resq);
  bool modbusSetOutputSignal(const aubo_msgs::srv::SetOutputSignal::Request::SharedPtr req,
                  aubo_msgs::srv::SetOutputSignal::Response::SharedPtr resq);             

  
  void publishIO();

  void publishToolData();

  // void publishRobotMode();

  // void publishSafetyMode();

  // void publishProgramRunning();

protected:
  void initMsgs();

  bool first_pass_;

  // internal commands
  std::array<double, 12> standard_digital_output_cmd_;
  std::array<double, 12> standard_analog_output_cmd_;
  double target_speed_fraction_cmd_;

  // services
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resend_robot_program_srv_;
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hand_back_control_srv_;
  // rclcpp::Service<aubo_msgs::srv::SetSpeedSliderFraction>::SharedPtr set_speed_slider_srv_;
  rclcpp::Service<aubo_msgs::srv::SetIO>::SharedPtr set_io_srv_;
  rclcpp::Service<aubo_msgs::srv::SetPayload>::SharedPtr set_payload_srv_;
  rclcpp::Service<aubo_msgs::srv::AddSignal>::SharedPtr modbus_add_signal_srv_;
  rclcpp::Service<aubo_msgs::srv::DeleteSignal>::SharedPtr modbus_delete_signal_srv_;
  rclcpp::Service<aubo_msgs::srv::GetSignalStatus>::SharedPtr modbus_get_signal_status_srv_;
  rclcpp::Service<aubo_msgs::srv::SetOutputSignal>::SharedPtr modbus_set_output_signal_srv_;
  rclcpp::Service<aubo_msgs::srv::SetSignalUpdateFrequency>::SharedPtr modbus_set_update_frequency_srv_;


  std::shared_ptr<rclcpp::Publisher<aubo_msgs::msg::IOStates>> io_pub_;
  std::shared_ptr<rclcpp::Publisher<aubo_msgs::msg::ToolDataMsg>> tool_data_pub_;
  // std::shared_ptr<rclcpp::Publisher<aubo_dashboard_msgs::msg::RobotMode>> robot_mode_pub_;
  // std::shared_ptr<rclcpp::Publisher<aubo_dashboard_msgs::msg::SafetyMode>> safety_mode_pub_;
  // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> program_state_pub_;

  aubo_msgs::msg::IOStates io_msg_;
  aubo_msgs::msg::ToolDataMsg tool_data_msg_;
  // aubo_dashboard_msgs::msg::RobotMode robot_mode_msg_;
  // aubo_dashboard_msgs::msg::SafetyMode safety_mode_msg_;
  // std_msgs::msg::Bool program_running_msg_;

  // Parameters from ROS for gpio_controller
  std::shared_ptr<gpio_controller::ParamListener> param_listener_;
  gpio_controller::Params params_;

  static constexpr double ASYNC_WAITING = 2.0;
  // TODO(anyone) publishers to add: tcp_pose_pub_
  // TODO(anyone) subscribers to add: script_command_sub_
  // TODO(anyone) service servers to add: resend_robot_program_srv_, deactivate_srv_, set_payload_srv_, tare_sensor_srv_

  /**
   * @brief wait until a command interface isn't in state ASYNC_WAITING anymore or until the parameter maximum_retries
   * have been reached
   */
  bool waitForAsyncCommand(std::function<double(void)> get_value);
};
}  // namespace aubo_controllers

#endif  // aubo_controllers__GPIO_CONTROLLER_HPP_
