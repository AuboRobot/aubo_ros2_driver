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

#include "aubo_controllers/gpio_controller.hpp"

#include <string>
namespace aubo_controllers
{
controller_interface::CallbackReturn GPIOController::on_init()
{
  try {
    initMsgs();
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<gpio_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::string tf_prefix = params_.tf_prefix;
  for (size_t i = 0; i < 12; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/standard_digital_output_cmd_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/standard_analog_output_cmd_" + std::to_string(i));
  }
  // config.names.emplace_back(tf_prefix + "gpio/tool_voltage_cmd");

  config.names.emplace_back(tf_prefix + "gpio/io_async_success");

  // config.names.emplace_back(tf_prefix + "speed_scaling/target_speed_fraction_cmd");

  // config.names.emplace_back(tf_prefix + "speed_scaling/target_speed_fraction_async_success");

  // config.names.emplace_back(tf_prefix + "resend_robot_program/resend_robot_program_cmd");

  // config.names.emplace_back(tf_prefix + "resend_robot_program/resend_robot_program_async_success");

  // payload stuff
  config.names.emplace_back(tf_prefix + "payload/mass");
  config.names.emplace_back(tf_prefix + "payload/cog.x");
  config.names.emplace_back(tf_prefix + "payload/cog.y");
  config.names.emplace_back(tf_prefix + "payload/cog.z");
  config.names.emplace_back(tf_prefix + "payload/payload_async_success");
  //modbus stuff 
  // config.names.emplace_back(tf_prefix + "modbus/device_info");
  config.names.emplace_back(tf_prefix + "modbus/slave_number");
  config.names.emplace_back(tf_prefix + "modbus/signal_address");
  config.names.emplace_back(tf_prefix + "modbus/signal_type");
  config.names.emplace_back(tf_prefix + "modbus/signal_name_a_d");
  config.names.emplace_back(tf_prefix + "modbus/signal_name_get");
  config.names.emplace_back(tf_prefix + "modbus/signal_name_set");
  config.names.emplace_back(tf_prefix + "modbus/set_signal_value");
  config.names.emplace_back(tf_prefix + "modbus/modbus_async_success");


  return config;
}

controller_interface::InterfaceConfiguration aubo_controllers::GPIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  // digital io  output index[0-11] input index[12-23]
  for (size_t i = 0; i < 12; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/digital_output_" + std::to_string(i));
  }
  for (size_t i = 0; i < 12; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/digital_input_" + std::to_string(i));
  }

  // analog io output index[24-25] input index[26-27]
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/standard_analog_output_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/standard_analog_input_" + std::to_string(i));
  }

  // tool voltage index[28] analog_input[29-30]
  // config.names.emplace_back(tf_prefix + "gpio/tool_output_voltage"); //TODO rtde未实现

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/tool_analog_input_" + std::to_string(i));
  }

  // // robot  TODO
  // config.names.emplace_back(tf_prefix + "gpio/robot_mode");

  // // safety  TODO
  // config.names.emplace_back(tf_prefix + "gpio/safety_mode");
  config.names.emplace_back(tf_prefix + "system_interface/initialized");

  // program running TODO
  // config.names.emplace_back(tf_prefix + "gpio/program_running");

  return config;
}

controller_interface::return_type aubo_controllers::GPIOController::update(const rclcpp::Time& /*time*/,
                                                                         const rclcpp::Duration& /*period*/)
{
  publishIO();
  publishToolData();
  // publishRobotMode();
  // publishSafetyMode();
  // publishProgramRunning();
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
aubo_controllers::GPIOController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GPIOController::publishIO()
{
  for (size_t i = 0; i < 12; ++i) {
    io_msg_.digital_out_states[i].pin = i;
    io_msg_.digital_out_states[i].state = static_cast<bool>(state_interfaces_[i].get_value());

    io_msg_.digital_in_states[i].pin = i;
    io_msg_.digital_in_states[i].state =
        static_cast<bool>(state_interfaces_[i + StateInterfaces::DIGITAL_INPUTS].get_value());
  }

  for (size_t i = 0; i < 2; ++i) {
    io_msg_.analog_in_states[i].pin = i;
    io_msg_.analog_in_states[i].state =
        static_cast<float>(state_interfaces_[i + StateInterfaces::ANALOG_INPUTS].get_value());
  }

  for (size_t i = 0; i < 2; ++i) {
    io_msg_.analog_out_states[i].pin = i;
    io_msg_.analog_out_states[i].state =
        static_cast<float>(state_interfaces_[i + StateInterfaces::ANALOG_OUTPUTS].get_value());
  }

  io_pub_->publish(io_msg_);
}

void GPIOController::publishToolData()
{

  tool_data_msg_.analog_input0 = static_cast<float>(state_interfaces_[StateInterfaces::TOOL_ANALOG_INPUTS].get_value());
  tool_data_msg_.analog_input1 =
      static_cast<float>(state_interfaces_[StateInterfaces::TOOL_ANALOG_INPUTS + 1].get_value());
  // tool_data_msg_.tool_output_voltage =
  //     static_cast<uint8_t>(state_interfaces_[StateInterfaces::TOOL_OUTPUT_VOLTAGE].get_value());
  tool_data_pub_->publish(tool_data_msg_);
}

// void GPIOController::publishRobotMode()
// {
//   auto robot_mode = static_cast<int8_t>(state_interfaces_[StateInterfaces::ROBOT_MODE].get_value());

//   if (robot_mode_msg_.mode != robot_mode) {
//     robot_mode_msg_.mode = robot_mode;
//     robot_mode_pub_->publish(robot_mode_msg_);
//   }
// }

// void GPIOController::publishSafetyMode()
// {
//   auto safety_mode = static_cast<uint8_t>(state_interfaces_[StateInterfaces::SAFETY_MODE].get_value());

//   if (safety_mode_msg_.mode != safety_mode) {
//     safety_mode_msg_.mode = safety_mode;
//     safety_mode_pub_->publish(safety_mode_msg_);
//   }
// }

// void GPIOController::publishProgramRunning()
// {
//   auto program_running_value = static_cast<uint8_t>(state_interfaces_[StateInterfaces::PROGRAM_RUNNING].get_value());
//   bool program_running = program_running_value == 1.0 ? true : false;
//   if (program_running_msg_.data != program_running) {
//     program_running_msg_.data = program_running;
//     program_state_pub_->publish(program_running_msg_);
//   }
// }

controller_interface::CallbackReturn
aubo_controllers::GPIOController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  while (state_interfaces_[StateInterfaces::INITIALIZED_FLAG].get_value() == 0.0) {
    RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  try {
    // register publisher
    io_pub_ = get_node()->create_publisher<aubo_msgs::msg::IOStates>("~/io_states", rclcpp::SystemDefaultsQoS());

    tool_data_pub_ =
        get_node()->create_publisher<aubo_msgs::msg::ToolDataMsg>("~/tool_data", rclcpp::SystemDefaultsQoS());
    // robot_mode_pub_ =
    //     get_node()->create_publisher<ur_dashboard_msgs::msg::RobotMode>("~/robot_mode", rclcpp::SystemDefaultsQoS());

    // safety_mode_pub_ =
    //     get_node()->create_publisher<ur_dashboard_msgs::msg::SafetyMode>("~/safety_mode", rclcpp::SystemDefaultsQoS());

    // auto program_state_pub_qos = rclcpp::SystemDefaultsQoS();
    // program_state_pub_qos.transient_local();
    // program_state_pub_ =
    //     get_node()->create_publisher<std_msgs::msg::Bool>("~/robot_program_running", program_state_pub_qos);

    set_io_srv_ = get_node()->create_service<aubo_msgs::srv::SetIO>(
        "~/set_io", std::bind(&GPIOController::setIO, this, std::placeholders::_1, std::placeholders::_2));

    // set_speed_slider_srv_ = get_node()->create_service<aubo_msgs::srv::SetSpeedSliderFraction>(
    //     "~/set_speed_slider",
    //     std::bind(&GPIOController::setSpeedSlider, this, std::placeholders::_1, std::placeholders::_2));

    // resend_robot_program_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    //     "~/resend_robot_program",
    //     std::bind(&GPIOController::resendRobotProgram, this, std::placeholders::_1, std::placeholders::_2));

    // hand_back_control_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
    //     "~/hand_back_control",
    //     std::bind(&GPIOController::handBackControl, this, std::placeholders::_1, std::placeholders::_2));

    set_payload_srv_ = get_node()->create_service<aubo_msgs::srv::SetPayload>(
        "~/set_payload", std::bind(&GPIOController::setPayload, this, std::placeholders::_1, std::placeholders::_2));
    
    modbus_add_signal_srv_ = get_node()->create_service<aubo_msgs::srv::AddSignal>(
        "~/modbus_add_signal", std::bind(&GPIOController::modbusAddSignal, this, std::placeholders::_1, std::placeholders::_2));
    modbus_delete_signal_srv_ = get_node()->create_service<aubo_msgs::srv::DeleteSignal>(
        "~/modbus_del_signal", std::bind(&GPIOController::modbusDeleteSignal, this, std::placeholders::_1, std::placeholders::_2));
    modbus_get_signal_status_srv_ = get_node()->create_service<aubo_msgs::srv::GetSignalStatus>(
        "~/modbus_get_signal_status", std::bind(&GPIOController::modbusGetSignalStatus, this, std::placeholders::_1, std::placeholders::_2));
    
    modbus_set_output_signal_srv_ = get_node()->create_service<aubo_msgs::srv::SetOutputSignal>(
            "~/modbus_set_output_signal", std::bind(&GPIOController::modbusSetOutputSignal, this, std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
aubo_controllers::GPIOController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try {
    // reset publisher
    io_pub_.reset();
    tool_data_pub_.reset();
    // robot_mode_pub_.reset();
    // safety_mode_pub_.reset();
    // program_state_pub_.reset();
    set_io_srv_.reset();
    set_payload_srv_.reset();
    modbus_add_signal_srv_.reset();
    modbus_delete_signal_srv_.reset();
    modbus_get_signal_status_srv_.reset();
    modbus_set_output_signal_srv_.reset();

  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool GPIOController::setIO(aubo_msgs::srv::SetIO::Request::SharedPtr req, aubo_msgs::srv::SetIO::Response::SharedPtr resp)
{
  if (req->fun == req->FUN_SET_DIGITAL_OUT && req->pin >= 0 && req->pin <= 11) {
    // io async success
    command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    command_interfaces_[req->pin].set_value(static_cast<double>(req->state));

    RCLCPP_INFO(get_node()->get_logger(), "Setting digital output '%d' to state: '%1.0f'.", req->pin, req->state);

    if (!waitForAsyncCommand([&]() { return command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value(); })) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not verify that io was set. (This might happen when using the "
                                            "mocked interface)");
    }

    resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    return resp->success;
  } else if (req->fun == req->FUN_SET_ANALOG_OUT && req->pin >= 0 && req->pin <= 2) {
    // io async success
    command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    command_interfaces_[CommandInterfaces::ANALOG_OUTPUTS_CMD + req->pin].set_value(static_cast<double>(req->state));

    RCLCPP_INFO(get_node()->get_logger(), "Setting analog output '%d' to state: '%1.0f'.", req->pin, req->state);

    if (!waitForAsyncCommand([&]() { return command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value(); })) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not verify that io was set. (This might happen when using the "
                                            "mocked interface)");
    }

    resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    return resp->success;
  }

   else {
    resp->success = false;
    return false;
  }
}


bool GPIOController::setPayload(const aubo_msgs::srv::SetPayload::Request::SharedPtr req,
                                aubo_msgs::srv::SetPayload::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::PAYLOAD_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  command_interfaces_[CommandInterfaces::PAYLOAD_MASS].set_value(req->mass);
  command_interfaces_[CommandInterfaces::PAYLOAD_COG_X].set_value(req->center_of_gravity.x);
  command_interfaces_[CommandInterfaces::PAYLOAD_COG_Y].set_value(req->center_of_gravity.y);
  command_interfaces_[CommandInterfaces::PAYLOAD_COG_Z].set_value(req->center_of_gravity.z);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::PAYLOAD_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that payload was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::PAYLOAD_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Payload has been set successfully");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set the payload");
    return false;
  }

  return true;
}

bool GPIOController::modbusAddSignal(const aubo_msgs::srv::AddSignal::Request::SharedPtr req,
                                aubo_msgs::srv::AddSignal::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  // command_interfaces_[CommandInterfaces::MODBUS_DEVICE_INFO].set_value(req->device_info.c_str());
  command_interfaces_[CommandInterfaces::MODBUS_SLAVE_NUMBER].set_value(req->slave_number);
  command_interfaces_[CommandInterfaces::MODBUD_SIGNAL_ADDRESS].set_value(req->signal_address);
  command_interfaces_[CommandInterfaces::MODBUS_SIGNAL_TYPE].set_value(req->signal_type);
  command_interfaces_[CommandInterfaces::MODBUS_SIGNAL_NAME_A_D].set_value(req->signal_name_index);
  // command_interfaces_[CommandInterfaces::MODBUS_SEQUENTIAL_MODE].set_value(req->sequential_mode);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that payload was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Modbus Signal has been add successfully");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not add Modbus Signal");
    return false;
  }

  return true;
}

bool GPIOController::modbusDeleteSignal(const aubo_msgs::srv::DeleteSignal::Request::SharedPtr req,
                                aubo_msgs::srv::DeleteSignal::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  command_interfaces_[CommandInterfaces::MODBUS_SIGNAL_NAME_A_D].set_value(req->signal_name_index);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that payload was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Modbus  Signal has been delete successfully");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not delete Modbus Signal");
    return false;
  }

  return true;
}

int32_t GPIOController::modbusGetSignalStatus(const aubo_msgs::srv::GetSignalStatus::Request::SharedPtr req,
                                aubo_msgs::srv::GetSignalStatus::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  command_interfaces_[CommandInterfaces::MODBUS_SIGNAL_NAME_GET].set_value(req->signal_name_index);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that payload was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->status = static_cast<int32_t>(command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].get_value());

  if (resp->status != -1) {
    RCLCPP_INFO(get_node()->get_logger(), "Modbus Signal get status successfully");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not get Modbus Signal");
  }

  return resp->status;
}

bool GPIOController::modbusSetOutputSignal(const aubo_msgs::srv::SetOutputSignal::Request::SharedPtr req,
                                aubo_msgs::srv::SetOutputSignal::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  command_interfaces_[CommandInterfaces::MODBUS_SIGNAL_NAME_SET].set_value(req->signal_name_index);
  command_interfaces_[CommandInterfaces::MODBUS_SET_SIGNAL_VALUE].set_value(req->value);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that payload was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::MODBUS_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Modbus Signal has been set successfully");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set Modbus Signal");
    return false;
  }

  return true;
}
void GPIOController::initMsgs()
{
  io_msg_.digital_in_states.resize(standard_digital_output_cmd_.size());
  io_msg_.digital_out_states.resize(standard_digital_output_cmd_.size());
  io_msg_.analog_in_states.resize(2);
  io_msg_.analog_out_states.resize(2);
}

bool GPIOController::waitForAsyncCommand(std::function<double(void)> get_value)
{
  const auto maximum_retries = params_.check_io_successfull_retries;
  int retries = 0;
  while (get_value() == ASYNC_WAITING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries)
      return false;
  }
  return true;
}

}  // namespace aubo_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aubo_controllers::GPIOController, controller_interface::ControllerInterface)
