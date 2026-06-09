#include "aubo_hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <ctime>
#include <limits>
#include <thread>
namespace aubo_driver {

AuboHardwareInterface::~AuboHardwareInterface()
{
    stopServoMode();
}

bool AuboHardwareInterface::connectRpcClient()
{
    if (rpc_client_ && !robot_name_.empty()) {
        return true;
    }

    try {
        rpc_client_ = std::make_shared<RpcClient>();
        rpc_client_->setRequestTimeout(1000);
        rpc_client_->connect(robot_ip_, 30004);
        rpc_client_->login("aubo", "123456");
        robot_name_ = rpc_client_->getRobotNames().front();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to connect RPC client to %s: %s",
                     robot_ip_.c_str(), e.what());
        rpc_client_.reset();
        robot_name_.clear();
        return false;
    }

    return true;
}

bool AuboHardwareInterface::connectRtdeClient()
{
    if (rtde_client_) {
        return true;
    }

    try {
        rtde_client_ = std::make_shared<RtdeClient>();
        rtde_client_->connect(robot_ip_, 30010);
        rtde_client_->login("aubo", "123456");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to connect RTDE client to %s: %s",
                     robot_ip_.c_str(), e.what());
        rtde_client_.reset();
        return false;
    }

    return true;
}

bool AuboHardwareInterface::probeIoControlLayout()
{
    if (!connectRpcClient()) {
        return false;
    }

    try {
        auto io_control =
            rpc_client_->getRobotInterface(robot_name_)->getIoControl();

        auto resize_state = [](std::vector<double> &values, int count) {
            values.assign(std::max(count, 0), 0.0);
        };

        resize_state(standard_digital_input_states_,
                     io_control->getStandardDigitalInputNum());
        resize_state(standard_digital_output_states_,
                     io_control->getStandardDigitalOutputNum());
        resize_state(configurable_digital_input_states_,
                     io_control->getConfigurableDigitalInputNum());
        resize_state(configurable_digital_output_states_,
                     io_control->getConfigurableDigitalOutputNum());
        resize_state(tool_digital_input_states_,
                     io_control->getToolDigitalInputNum());
        resize_state(tool_digital_output_states_,
                     io_control->getToolDigitalOutputNum());
        resize_state(standard_analog_input_states_,
                     io_control->getStandardAnalogInputNum());
        resize_state(standard_analog_output_states_,
                     io_control->getStandardAnalogOutputNum());
        resize_state(tool_analog_input_states_,
                     io_control->getToolAnalogInputNum());
        resize_state(tool_analog_output_states_,
                     io_control->getToolAnalogOutputNum());

        const std::size_t tool_io_count =
            std::max(tool_digital_input_states_.size(),
                     tool_digital_output_states_.size());
        tool_io_input_states_.assign(tool_io_count, 0.0);
        tool_digital_input_action_states_.assign(
            tool_digital_input_states_.size(), 0.0);
        tool_digital_output_runstate_states_.assign(
            tool_digital_output_states_.size(), 0.0);
        tool_analog_input_domain_states_.assign(
            tool_analog_input_states_.size(), 0.0);
        tool_analog_output_domain_states_.assign(
            tool_analog_output_states_.size(), 0.0);
        tool_analog_output_runstate_states_.assign(
            tool_analog_output_states_.size(), 0.0);

    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to probe IoControl layout: %s", e.what());
        return false;
    }

    return true;
}

void AuboHardwareInterface::initAsyncSdkCommands()
{
    const double no_new_command = std::numeric_limits<double>::quiet_NaN();
    std::fill(tcp_offset_commands_.begin(), tcp_offset_commands_.end(),
              no_new_command);
    std::fill(payload_cog_commands_.begin(), payload_cog_commands_.end(),
              no_new_command);
    std::fill(handguide_free_axis_commands_.begin(),
              handguide_free_axis_commands_.end(), no_new_command);
    std::fill(handguide_feature_commands_.begin(),
              handguide_feature_commands_.end(), no_new_command);
    digital_output_type_command_ = no_new_command;
    digital_output_pin_command_ = no_new_command;
    digital_output_value_command_ = no_new_command;
    digital_output_trigger_command_ = no_new_command;
    analog_output_type_command_ = no_new_command;
    analog_output_pin_command_ = no_new_command;
    analog_output_value_command_ = no_new_command;
    analog_output_trigger_command_ = no_new_command;
    tool_voltage_output_domain_command_ = no_new_command;
    tool_io_input_pin_command_ = no_new_command;
    tool_io_input_value_command_ = no_new_command;
    tool_io_input_trigger_command_ = no_new_command;
    tool_io_config_type_command_ = no_new_command;
    tool_io_config_pin_command_ = no_new_command;
    tool_io_config_value_command_ = no_new_command;
    tool_io_config_trigger_command_ = no_new_command;
    payload_mass_command_ = no_new_command;
    handguide_enable_command_ = no_new_command;
}

void AuboHardwareInterface::readIoControlConfigStates()
{
    if (!connectRpcClient()) {
        return;
    }

    auto io_control =
        rpc_client_->getRobotInterface(robot_name_)->getIoControl();

    for (std::size_t i = 0; i < tool_io_input_states_.size(); ++i) {
        try {
            tool_io_input_states_[i] =
                io_control->isToolIoInput(static_cast<int>(i)) ? 1.0 : 0.0;
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Failed to read get_tool_io_input_%zu: %s", i,
                        e.what());
        }
    }

    for (std::size_t i = 0; i < tool_digital_input_action_states_.size();
         ++i) {
        try {
            tool_digital_input_action_states_[i] =
                static_cast<double>(io_control->getToolDigitalInputAction(
                    static_cast<int>(i)));
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Failed to read get_tool_digital_input_action_%zu: %s",
                        i, e.what());
        }
    }

    for (std::size_t i = 0; i < tool_digital_output_runstate_states_.size();
         ++i) {
        try {
            tool_digital_output_runstate_states_[i] =
                static_cast<double>(io_control->getToolDigitalOutputRunstate(
                    static_cast<int>(i)));
        } catch (const std::exception &e) {
            RCLCPP_WARN(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Failed to read get_tool_digital_output_runstate_%zu: %s", i,
                e.what());
        }
    }

    for (std::size_t i = 0; i < tool_analog_input_domain_states_.size(); ++i) {
        try {
            tool_analog_input_domain_states_[i] = static_cast<double>(
                io_control->getToolAnalogInputDomain(static_cast<int>(i)));
        } catch (const std::exception &e) {
            RCLCPP_WARN(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Failed to read get_tool_analog_input_domain_%zu: %s", i,
                e.what());
        }
    }

    for (std::size_t i = 0; i < tool_analog_output_domain_states_.size();
         ++i) {
        try {
            tool_analog_output_domain_states_[i] = static_cast<double>(
                io_control->getToolAnalogOutputDomain(static_cast<int>(i)));
        } catch (const std::exception &e) {
            RCLCPP_WARN(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Failed to read get_tool_analog_output_domain_%zu: %s", i,
                e.what());
        }
    }

    for (std::size_t i = 0; i < tool_analog_output_runstate_states_.size();
         ++i) {
        try {
            tool_analog_output_runstate_states_[i] =
                static_cast<double>(io_control->getToolAnalogOutputRunstate(
                    static_cast<int>(i)));
        } catch (const std::exception &e) {
            RCLCPP_WARN(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Failed to read get_tool_analog_output_runstate_%zu: %s", i,
                e.what());
        }
    }

    try {
        tool_voltage_output_domain_state_ =
            static_cast<double>(io_control->getToolVoltageOutputDomain());
    } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                    "Failed to read get_tool_voltage_output_domain: %s",
                    e.what());
    }
}

void AuboHardwareInterface::readRobotManageStates()
{
    if (!connectRpcClient()) {
        return;
    }

    auto robot_manage =
        rpc_client_->getRobotInterface(robot_name_)->getRobotManage();

    try {
        handguide_enabled_state_ =
            robot_manage->isHandguideEnabled() ? 1.0 : 0.0;
    } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                    "Failed to read get_handguide_enabled: %s", e.what());
    }

    try {
        auto free_axes = robot_manage->getHandguideFreeAxes();
        const std::size_t count =
            std::min(free_axes.size(), handguide_free_axis_state_.size());
        for (std::size_t i = 0; i < count; ++i) {
            handguide_free_axis_state_[i] =
                static_cast<double>(free_axes[i]);
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                    "Failed to read get_handguide_free_axis: %s", e.what());
    }

    try {
        auto feature = robot_manage->getHandguideFeature();
        const std::size_t count =
            std::min(feature.size(), handguide_feature_state_.size());
        for (std::size_t i = 0; i < count; ++i) {
            handguide_feature_state_[i] = feature[i];
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                    "Failed to read get_handguide_feature: %s", e.what());
    }
}

void AuboHardwareInterface::readRobotConfigStates()
{
    if (!connectRpcClient()) {
        return;
    }

    try {
        auto tcp_offset = rpc_client_->getRobotInterface(robot_name_)
                              ->getRobotConfig()
                              ->getTcpOffset();
        const std::size_t count =
            std::min(tcp_offset.size(), tcp_offset_state_.size());
        for (std::size_t i = 0; i < count; ++i) {
            tcp_offset_state_[i] = tcp_offset[i];
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                    "Failed to read RobotConfig TCP offset: %s", e.what());
    }
}

bool AuboHardwareInterface::OnActive()
{
    if (!connectRpcClient()) {
        return false;
    }
    if (!connectRtdeClient()) {
        return false;
    }

    int topic = rtde_client_->setTopic(false, { "R1_message" }, 200, 0);
    if (topic < 0) {
        std::cout << "Set topic fail!" << std::endl;
    }
    rtde_client_->subscribe(topic, [](InputParser &parser) {
        parser.popRobotMsgVector();
    });
    rpc_client_->getRobotInterface(robot_name_)
    ->getRobotConfig()
    ->setHardwareCustomParameters("[joint_func] \n vff_enable = false\n");

    std::cout << "vff_enable = false" << std::endl;

    // 设置rtde输入
    setInput(rtde_client_);

    // 配置输出
    configSubscribe(rtde_client_);

    readIoControlConfigStates();
    readRobotManageStates();
    readRobotConfigStates();

    return true;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &system_info)
{
    if (hardware_interface::SystemInterface::on_init(system_info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = system_info;
    initialized_ = false;
    robot_ip_ = info_.hardware_parameters["robot_ip"];

    auto servo_mode_param = info_.hardware_parameters.find("servo_mode");
    if (servo_mode_param != info_.hardware_parameters.end() &&
        !servo_mode_param->second.empty()) {
        try {
            servo_mode_ = std::stoi(servo_mode_param->second);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Invalid servo_mode parameter '%s': %s",
                servo_mode_param->second.c_str(), e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    if (!probeIoControlLayout()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    initAsyncSdkCommands();
    std::fill(aubo_velocity_commands_.begin(), aubo_velocity_commands_.end(),
              0.0);
    readIoControlConfigStates();
    readRobotManageStates();
    readRobotConfigStates();

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Joint '%s' has %zu command interfaces found. 2 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[1].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Joint '%s' have %s command interfaces found. '%s' expected "
                "as unsupported placeholder.",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
                         "Joint '%s' has %zu state interface. 2 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
                         "Joint '%s' have %s state interface. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
                         "Joint '%s' have %s state interface. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(),
                         hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn AuboHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Starting ...please wait...");
    OnActive();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    readActualQ();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        std::fill(aubo_velocity_commands_.begin(),
                  aubo_velocity_commands_.end(), 0.0);
        initialized_ = true;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AuboHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    auto export_indexed_state = [&state_interfaces](
                                    const std::string &component_name,
                                    const std::string &interface_prefix,
                                    auto &values) {
        for (std::size_t i = 0; i < values.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                component_name, interface_prefix + "_" + std::to_string(i),
                &values[i]));
        }
    };

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &actual_q_copy_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &joint_velocity_copy_[i]));
    }

    export_indexed_state("io_control", "get_standard_digital_input",
                         standard_digital_input_states_);
    export_indexed_state("io_control", "get_standard_digital_output",
                         standard_digital_output_states_);
    export_indexed_state("io_control", "get_configurable_digital_input",
                         configurable_digital_input_states_);
    export_indexed_state("io_control", "get_configurable_digital_output",
                         configurable_digital_output_states_);
    export_indexed_state("io_control", "get_tool_digital_input",
                         tool_digital_input_states_);
    export_indexed_state("io_control", "get_tool_digital_output",
                         tool_digital_output_states_);
    export_indexed_state("io_control", "get_standard_analog_input",
                         standard_analog_input_states_);
    export_indexed_state("io_control", "get_standard_analog_output",
                         standard_analog_output_states_);
    export_indexed_state("io_control", "get_tool_analog_input",
                         tool_analog_input_states_);
    export_indexed_state("io_control", "get_tool_analog_output",
                         tool_analog_output_states_);
    export_indexed_state("io_control", "get_tool_io_input",
                         tool_io_input_states_);
    export_indexed_state("io_control", "get_tool_digital_input_action",
                         tool_digital_input_action_states_);
    export_indexed_state("io_control", "get_tool_digital_output_runstate",
                         tool_digital_output_runstate_states_);
    export_indexed_state("io_control", "get_tool_analog_input_domain",
                         tool_analog_input_domain_states_);
    export_indexed_state("io_control", "get_tool_analog_output_domain",
                         tool_analog_output_domain_states_);
    export_indexed_state("io_control", "get_tool_analog_output_runstate",
                         tool_analog_output_runstate_states_);
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "io_control", "get_tool_voltage_output_domain",
        &tool_voltage_output_domain_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "io_control", "get_tool_button_status", &tool_button_status_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "io_control", "get_handle_status", &handle_io_status_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "io_control", "get_handle_state", &handle_dev_state_state_));

    export_indexed_state("robot_state", "get_tcp_pose", tcp_pose_state_);
    export_indexed_state("robot_state", "get_tcp_speed", tcp_speed_state_);
    export_indexed_state("robot_state", "get_tcp_force", tcp_force_state_);
    export_indexed_state("robot_state", "get_tool_pose", tool_pose_state_);

    export_indexed_state("robot_config", "get_tcp_offset", tcp_offset_state_);
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "robot_config", "get_payload_mass", &payload_mass_state_));
    export_indexed_state("robot_config", "get_payload_cog",
                         payload_cog_state_);

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "robot_manage", "get_handguide_enabled",
        &handguide_enabled_state_));
    export_indexed_state("robot_manage", "get_handguide_free_axis",
                         handguide_free_axis_state_);
    export_indexed_state("robot_manage", "get_handguide_feature",
                         handguide_feature_state_);

    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
AuboHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    auto export_indexed_command = [&command_interfaces](
                                      const std::string &component_name,
                                      const std::string &interface_prefix,
                                      auto &values) {
        for (std::size_t i = 0; i < values.size(); ++i) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    component_name,
                    interface_prefix + "_" + std::to_string(i), &values[i]));
        }
    };
    auto export_command = [&command_interfaces](
                              const std::string &component_name,
                              const std::string &interface_name,
                              double &value) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            component_name, interface_name, &value));
    };

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &aubo_position_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &aubo_velocity_commands_[i]));
    }

    export_command("io_control", "set_digital_output_type",
                   digital_output_type_command_);
    export_command("io_control", "set_digital_output_pin",
                   digital_output_pin_command_);
    export_command("io_control", "set_digital_output_value",
                   digital_output_value_command_);
    export_command("io_control", "set_digital_output_trigger",
                   digital_output_trigger_command_);
    export_command("io_control", "set_analog_output_type",
                   analog_output_type_command_);
    export_command("io_control", "set_analog_output_pin",
                   analog_output_pin_command_);
    export_command("io_control", "set_analog_output_value",
                   analog_output_value_command_);
    export_command("io_control", "set_analog_output_trigger",
                   analog_output_trigger_command_);
    export_command("io_control", "set_tool_voltage_output_domain",
                   tool_voltage_output_domain_command_);
    export_command("io_control", "set_tool_io_input_pin",
                   tool_io_input_pin_command_);
    export_command("io_control", "set_tool_io_input_value",
                   tool_io_input_value_command_);
    export_command("io_control", "set_tool_io_input_trigger",
                   tool_io_input_trigger_command_);
    export_command("io_control", "set_tool_io_config_type",
                   tool_io_config_type_command_);
    export_command("io_control", "set_tool_io_config_pin",
                   tool_io_config_pin_command_);
    export_command("io_control", "set_tool_io_config_value",
                   tool_io_config_value_command_);
    export_command("io_control", "set_tool_io_config_trigger",
                   tool_io_config_trigger_command_);

    export_indexed_command("robot_config", "set_tcp_offset",
                           tcp_offset_commands_);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "robot_config", "set_payload_mass", &payload_mass_command_));
    export_indexed_command("robot_config", "set_payload_cog",
                           payload_cog_commands_);

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "robot_manage", "set_handguide_enable",
        &handguide_enable_command_));
    export_indexed_command("robot_manage", "set_handguide_free_axis",
                           handguide_free_axis_commands_);
    export_indexed_command("robot_manage", "set_handguide_feature",
                           handguide_feature_commands_);

    return command_interfaces;
}

hardware_interface::return_type
AuboHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
{
    auto has_joint_interface = [this](const std::vector<std::string> &interfaces,
                                      const std::string &interface_name) {
        for (const auto &joint : info_.joints) {
            const std::string full_name = joint.name + "/" + interface_name;
            if (std::find(interfaces.begin(), interfaces.end(), full_name) !=
                interfaces.end()) {
                return true;
            }
        }
        return false;
    };

    const bool start_position =
        has_joint_interface(start_interfaces, hardware_interface::HW_IF_POSITION);
    const bool start_velocity =
        has_joint_interface(start_interfaces, hardware_interface::HW_IF_VELOCITY);
    const bool stop_position =
        has_joint_interface(stop_interfaces, hardware_interface::HW_IF_POSITION);
    const bool stop_velocity =
        has_joint_interface(stop_interfaces, hardware_interface::HW_IF_VELOCITY);

    bool position_running = position_controller_running_.load();
    bool velocity_running = velocity_controller_running_.load();
    if (stop_position) {
        position_running = false;
    }
    if (stop_velocity) {
        velocity_running = false;
    }
    if (start_position) {
        position_running = true;
    }
    if (start_velocity) {
        velocity_running = true;
    }

    if (position_running && velocity_running) {
        RCLCPP_ERROR(
            rclcpp::get_logger("AuboHardwareInterface"),
            "Cannot run position and velocity command interfaces at the same "
            "time.");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
AuboHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces)
{
    auto has_joint_interface = [this](const std::vector<std::string> &interfaces,
                                      const std::string &interface_name) {
        for (const auto &joint : info_.joints) {
            const std::string full_name = joint.name + "/" + interface_name;
            if (std::find(interfaces.begin(), interfaces.end(), full_name) !=
                interfaces.end()) {
                return true;
            }
        }
        return false;
    };

    if (has_joint_interface(stop_interfaces, hardware_interface::HW_IF_POSITION)) {
        position_controller_running_.store(false);
    }
    if (has_joint_interface(stop_interfaces, hardware_interface::HW_IF_VELOCITY)) {
        velocity_controller_running_.store(false);
        std::fill(aubo_velocity_commands_.begin(),
                  aubo_velocity_commands_.end(), 0.0);
    }
    if (has_joint_interface(start_interfaces, hardware_interface::HW_IF_POSITION)) {
        {
            std::unique_lock<std::mutex> lck(rtde_mtx_);
            const std::size_t count =
                std::min(actual_q_.size(), aubo_position_commands_.size());
            for (std::size_t i = 0; i < count; ++i) {
                aubo_position_commands_[i] = actual_q_[i];
            }
        }
        position_controller_running_.store(true);
    }
    if (has_joint_interface(start_interfaces, hardware_interface::HW_IF_VELOCITY)) {
        velocity_controller_running_.store(true);
        std::fill(aubo_velocity_commands_.begin(),
                  aubo_velocity_commands_.end(), 0.0);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuboHardwareInterface::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    readActualQ();
    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        std::fill(aubo_velocity_commands_.begin(),
                  aubo_velocity_commands_.end(), 0.0);
        initialized_ = true;
    }
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type AuboHardwareInterface::write(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    checkAsyncSdkCommands();

    if (handguide_mode_active_.load()) {
        return hardware_interface::return_type::OK;
    }

    const bool velocity_controller_running =
        velocity_controller_running_.load();
    const bool position_controller_running =
        position_controller_running_.load();
    if (!velocity_controller_running && !position_controller_running) {
        if (stopServoMode() != 0) {
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    if (robot_mode_ == RobotModeType::Running && (safety_mode_ == 
        SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode)) {
        try {
            if (velocity_controller_running) {
                speedServo(aubo_velocity_commands_);
            } else if (position_controller_running) {
                Servoj(aubo_position_commands_);
            }
        } catch (const std::exception &e) {
        }
    }else{
        // 机器人状态异常
        RCLCPP_WARN_STREAM(
            rclcpp::get_logger("AuboHardwareInterface"),
            "Robot not in valid state for motion command. Plz check&fix robot status firstly then restart driver"
            << "robot_mode_: " << static_cast<int>(robot_mode_)
            << ", safety_mode_: " << static_cast<int>(safety_mode_));

        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

void AuboHardwareInterface::readActualQ()
{
    {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        auto copy_vector = [](const std::vector<double> &source,
                              auto &target) {
            const std::size_t count = std::min(source.size(), target.size());
            for (std::size_t i = 0; i < count; ++i) {
                target[i] = source[i];
            }
        };
        auto copy_bits = [](std::uint64_t mask, std::vector<double> &target) {
            for (std::size_t i = 0; i < target.size(); ++i) {
                target[i] =
                    i < 64 && (mask & (1ULL << i)) != 0ULL ? 1.0 : 0.0;
            }
        };

        copy_vector(actual_q_, actual_q_copy_);
        copy_vector(joint_velocity_, joint_velocity_copy_);

        copy_vector(actual_TCP_pose_, tcp_pose_state_);
        copy_vector(actual_TCP_speed_, tcp_speed_state_);
        copy_vector(actual_TCP_force_, tcp_force_state_);
        copy_vector(actual_tool_pose_, tool_pose_state_);

        copy_bits(standard_digital_input_bits_, standard_digital_input_states_);
        copy_bits(standard_digital_output_bits_,
                  standard_digital_output_states_);
        copy_bits(configurable_digital_input_bits_,
                  configurable_digital_input_states_);
        copy_bits(configurable_digital_output_bits_,
                  configurable_digital_output_states_);
        copy_bits(tool_digital_input_bits_, tool_digital_input_states_);
        copy_bits(tool_digital_output_bits_, tool_digital_output_states_);

        copy_vector(standard_analog_input_values_,
                    standard_analog_input_states_);
        copy_vector(standard_analog_output_values_,
                    standard_analog_output_states_);
        copy_vector(tool_analog_input_values_, tool_analog_input_states_);
        copy_vector(tool_analog_output_values_, tool_analog_output_states_);

        tool_button_status_state_ = tool_button_status_ ? 1.0 : 0.0;
        handle_io_status_state_ = static_cast<double>(handle_status_);
        handle_dev_state_state_ = static_cast<double>(handle_dev_state_);

        payload_mass_state_ = std::get<0>(actual_payload_);
        copy_vector(std::get<1>(actual_payload_), payload_cog_state_);
    }
}
// 设置rtde输入

bool AuboHardwareInterface::isServoModeStart()
{
    return servo_mode_start_;
}
int AuboHardwareInterface::startServoMode()
{
    if (!rpc_client_ || robot_name_.empty()) {
        return -1;
    }

    auto motion_control =
        rpc_client_->getRobotInterface(robot_name_)->getMotionControl();
    if (servo_mode_start_ &&
        motion_control->getServoModeSelect() == servo_mode_) {
        return 0;
    }

    //开启servo模式
    motion_control->setServoModeSelect(servo_mode_);
    int i = 0;
    while (motion_control->getServoModeSelect() != servo_mode_) {
        if (i++ > 5) {
            std::cout << "Servo Mode enable fail! Servo Mode is "
                      << motion_control->getServoModeSelect()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    servo_mode_start_ = true;
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Servo mode select %d enabled.", servo_mode_);
    return 0;
}

int AuboHardwareInterface::stopServoMode()
{
    if (!rpc_client_ || robot_name_.empty()) {
        servo_mode_start_ = false;
        return 0;
    }

    try {
        auto robot_interface = rpc_client_->getRobotInterface(robot_name_);
        auto motion_control = robot_interface->getMotionControl();
        int selected_servo_mode = 0;
        selected_servo_mode = motion_control->getServoModeSelect();

        if (selected_servo_mode == 0) {
            servo_mode_start_ = false;
            return 0;
        }

        RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                    "Disabling servo mode select %d.", selected_servo_mode);

        int steady_wait_count = 0;
        while (!robot_interface->getRobotState()->isSteady()) {
            if (steady_wait_count++ > 100) {
                RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                            "Timed out waiting robot steady before disabling "
                            "servo mode.");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        motion_control->setServoModeSelect(0);
        int i = 0;
        while (motion_control->getServoModeSelect() != 0) {
            if (i++ > 5) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("AuboHardwareInterface"),
                    "Servo mode disable failed. Current servo mode is %d.",
                    motion_control->getServoModeSelect());
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to disable servo mode: %s", e.what());
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Servo mode disabled.");
    servo_mode_start_ = false;
    return 0;
}

int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    if (startServoMode() != 0) {
        return -1;
    }

    std::vector<double> traj(6, 0);
    for (size_t i = 0; i < traj.size(); i++) {
        traj[i] = joint_position_command[i];
    }

    // 接口调用: servoJoint
    while (true) {
        int servoJoint_num = rpc_client_->getRobotInterface(robot_name_)
                                ->getMotionControl()
                                ->servoJoint(traj, 0.2, 0.2, 0.01, 0.1, 200);
        if(servoJoint_num != 2){
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

int AuboHardwareInterface::speedServo(
    const std::array<double, 6> joint_velocity_command)
{
    (void)joint_velocity_command;
    RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                 "speedServo failed: HW_IF_VELOCITY command chain is ready, "
                 "but the speedServo backend is not implemented yet.");

    return -1;
}

void AuboHardwareInterface::checkAsyncSdkCommands()
{
    if (!rpc_client_ || robot_name_.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(sdk_command_mtx_);

    const double no_new_command = std::numeric_limits<double>::quiet_NaN();
    auto has_new_command = [](double value) { return !std::isnan(value); };

    auto robot_interface = rpc_client_->getRobotInterface(robot_name_);
    auto io_control = robot_interface->getIoControl();
    auto robot_config = robot_interface->getRobotConfig();
    auto robot_manage = robot_interface->getRobotManage();

    auto log_sdk_result = [](const std::string &name, int result) {
        if (result != 0) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "%s returned error code %d", name.c_str(), result);
        }
    };

    auto update_indexed_state = [](std::vector<double> &states, int index,
                                   double value) {
        if (index >= 0 && static_cast<std::size_t>(index) < states.size()) {
            states[static_cast<std::size_t>(index)] = value;
        }
    };

    if (has_new_command(digital_output_trigger_command_)) {
        if (has_new_command(digital_output_type_command_) &&
            has_new_command(digital_output_pin_command_) &&
            has_new_command(digital_output_value_command_)) {
            const int output_type =
                static_cast<int>(std::lround(digital_output_type_command_));
            const int pin =
                static_cast<int>(std::lround(digital_output_pin_command_));
            const bool value = digital_output_value_command_ != 0.0;
            try {
                int result = 0;
                switch (output_type) {
                case 0:
                    result = io_control->setStandardDigitalOutput(pin, value);
                    log_sdk_result("setStandardDigitalOutput", result);
                    if (result == 0) {
                        update_indexed_state(standard_digital_output_states_,
                                             pin, value ? 1.0 : 0.0);
                    }
                    break;
                case 1:
                    result =
                        io_control->setConfigurableDigitalOutput(pin, value);
                    log_sdk_result("setConfigurableDigitalOutput", result);
                    if (result == 0) {
                        update_indexed_state(
                            configurable_digital_output_states_, pin,
                            value ? 1.0 : 0.0);
                    }
                    break;
                case 2:
                    result = io_control->setToolDigitalOutput(pin, value);
                    log_sdk_result("setToolDigitalOutput", result);
                    if (result == 0) {
                        update_indexed_state(tool_digital_output_states_, pin,
                                             value ? 1.0 : 0.0);
                    }
                    break;
                default:
                    RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                                "Unknown digital output type: %d",
                                output_type);
                }
            } catch (const std::exception &e) {
                RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                            "Failed to set digital output: %s", e.what());
            }
        }
        digital_output_type_command_ = no_new_command;
        digital_output_pin_command_ = no_new_command;
        digital_output_value_command_ = no_new_command;
        digital_output_trigger_command_ = no_new_command;
    }

    if (has_new_command(analog_output_trigger_command_)) {
        if (has_new_command(analog_output_type_command_) &&
            has_new_command(analog_output_pin_command_) &&
            has_new_command(analog_output_value_command_)) {
            const int output_type =
                static_cast<int>(std::lround(analog_output_type_command_));
            const int pin =
                static_cast<int>(std::lround(analog_output_pin_command_));
            const double value = analog_output_value_command_;
            try {
                int result = 0;
                switch (output_type) {
                case 0:
                    result = io_control->setStandardAnalogOutput(pin, value);
                    log_sdk_result("setStandardAnalogOutput", result);
                    if (result == 0) {
                        update_indexed_state(standard_analog_output_states_,
                                             pin, value);
                    }
                    break;
                case 1:
                    result = io_control->setToolAnalogOutput(pin, value);
                    log_sdk_result("setToolAnalogOutput", result);
                    if (result == 0) {
                        update_indexed_state(tool_analog_output_states_, pin,
                                             value);
                    }
                    break;
                default:
                    RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                                "Unknown analog output type: %d",
                                output_type);
                }
            } catch (const std::exception &e) {
                RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                            "Failed to set analog output: %s", e.what());
            }
        }
        analog_output_type_command_ = no_new_command;
        analog_output_pin_command_ = no_new_command;
        analog_output_value_command_ = no_new_command;
        analog_output_trigger_command_ = no_new_command;
    }

    if (has_new_command(tool_io_input_trigger_command_)) {
        if (has_new_command(tool_io_input_pin_command_) &&
            has_new_command(tool_io_input_value_command_)) {
            const int pin =
                static_cast<int>(std::lround(tool_io_input_pin_command_));
            const bool value = tool_io_input_value_command_ != 0.0;
            try {
                const int result = io_control->setToolIoInput(pin, value);
                log_sdk_result("setToolIoInput", result);
                if (result == 0) {
                    update_indexed_state(tool_io_input_states_, pin,
                                         value ? 1.0 : 0.0);
                }
            } catch (const std::exception &e) {
                RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                            "Failed to set tool IO input: %s", e.what());
            }
        }
        tool_io_input_pin_command_ = no_new_command;
        tool_io_input_value_command_ = no_new_command;
        tool_io_input_trigger_command_ = no_new_command;
    }

    if (has_new_command(tool_io_config_trigger_command_)) {
        if (has_new_command(tool_io_config_type_command_) &&
            has_new_command(tool_io_config_pin_command_) &&
            has_new_command(tool_io_config_value_command_)) {
            const int config_type =
                static_cast<int>(std::lround(tool_io_config_type_command_));
            const int pin =
                static_cast<int>(std::lround(tool_io_config_pin_command_));
            const int value =
                static_cast<int>(std::lround(tool_io_config_value_command_));
            try {
                int result = 0;
                switch (config_type) {
                case 0:
                    result = io_control->setToolDigitalInputAction(
                        pin, static_cast<StandardInputAction>(value));
                    log_sdk_result("setToolDigitalInputAction", result);
                    if (result == 0) {
                        update_indexed_state(tool_digital_input_action_states_,
                                             pin, static_cast<double>(value));
                    }
                    break;
                case 1:
                    result = io_control->setToolDigitalOutputRunstate(
                        pin, static_cast<StandardOutputRunState>(value));
                    log_sdk_result("setToolDigitalOutputRunstate", result);
                    if (result == 0) {
                        update_indexed_state(
                            tool_digital_output_runstate_states_, pin,
                            static_cast<double>(value));
                    }
                    break;
                case 2:
                    result = io_control->setToolAnalogInputDomain(pin, value);
                    log_sdk_result("setToolAnalogInputDomain", result);
                    if (result == 0) {
                        update_indexed_state(tool_analog_input_domain_states_,
                                             pin, static_cast<double>(value));
                    }
                    break;
                case 3:
                    result = io_control->setToolAnalogOutputDomain(pin, value);
                    log_sdk_result("setToolAnalogOutputDomain", result);
                    if (result == 0) {
                        update_indexed_state(tool_analog_output_domain_states_,
                                             pin, static_cast<double>(value));
                    }
                    break;
                case 4:
                    result = io_control->setToolAnalogOutputRunstate(
                        pin, static_cast<StandardOutputRunState>(value));
                    log_sdk_result("setToolAnalogOutputRunstate", result);
                    if (result == 0) {
                        update_indexed_state(
                            tool_analog_output_runstate_states_, pin,
                            static_cast<double>(value));
                    }
                    break;
                default:
                    RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                                "Unknown tool IO config type: %d",
                                config_type);
                }
            } catch (const std::exception &e) {
                RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                            "Failed to set tool IO config: %s", e.what());
            }
        }
        tool_io_config_type_command_ = no_new_command;
        tool_io_config_pin_command_ = no_new_command;
        tool_io_config_value_command_ = no_new_command;
        tool_io_config_trigger_command_ = no_new_command;
    }

    if (has_new_command(tool_voltage_output_domain_command_)) {
        try {
            const int value = static_cast<int>(
                std::lround(tool_voltage_output_domain_command_));
            const int result = io_control->setToolVoltageOutputDomain(value);
            log_sdk_result("setToolVoltageOutputDomain", result);
            if (result == 0) {
                tool_voltage_output_domain_state_ = static_cast<double>(value);
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Failed to set set_tool_voltage_output_domain: %s",
                        e.what());
        }
        tool_voltage_output_domain_command_ = no_new_command;
    }

    bool has_handguide_params_command = false;
    std::vector<int> handguide_free_axes(handguide_free_axis_state_.size(), 1);
    for (std::size_t i = 0; i < handguide_free_axis_state_.size(); ++i) {
        handguide_free_axes[i] =
            static_cast<int>(std::lround(handguide_free_axis_state_[i]));
        if (!has_new_command(handguide_free_axis_commands_[i])) {
            continue;
        }
        handguide_free_axes[i] =
            static_cast<int>(std::lround(handguide_free_axis_commands_[i]));
        handguide_free_axis_commands_[i] = no_new_command;
        has_handguide_params_command = true;
    }

    std::vector<double> handguide_feature(handguide_feature_state_.begin(),
                                          handguide_feature_state_.end());
    for (std::size_t i = 0; i < handguide_feature_commands_.size(); ++i) {
        if (!has_new_command(handguide_feature_commands_[i])) {
            continue;
        }
        handguide_feature[i] = handguide_feature_commands_[i];
        handguide_feature_commands_[i] = no_new_command;
        has_handguide_params_command = true;
    }

    if (has_handguide_params_command) {
        try {
            robot_manage->setHandguideParams(handguide_free_axes,
                                             handguide_feature);
            for (std::size_t i = 0; i < handguide_free_axis_state_.size();
                 ++i) {
                handguide_free_axis_state_[i] =
                    static_cast<double>(handguide_free_axes[i]);
            }
            const std::size_t count = std::min(handguide_feature.size(),
                                               handguide_feature_state_.size());
            for (std::size_t i = 0; i < count; ++i) {
                handguide_feature_state_[i] = handguide_feature[i];
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Failed to set handguide parameters: %s", e.what());
        }
    }

    if (has_new_command(handguide_enable_command_)) {
        try {
            if (handguide_enable_command_ != 0.0) {
                handguide_mode_active_.store(true);
                if (stopServoMode() != 0) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("AuboHardwareInterface"),
                        "Failed to disable servo mode before entering "
                        "handguide mode.");
                    handguide_mode_active_.store(false);
                    handguide_enable_command_ = no_new_command;
                    return;
                }

                std::vector<int> free_axes(handguide_free_axis_state_.size(),
                                           1);
                for (std::size_t i = 0; i < handguide_free_axis_state_.size();
                     ++i) {
                    free_axes[i] = static_cast<int>(
                        std::lround(handguide_free_axis_state_[i]));
                }
                std::vector<double> feature(handguide_feature_state_.begin(),
                                            handguide_feature_state_.end());
                robot_manage->handguideMode(free_axes, feature);
                handguide_enabled_state_ = 1.0;
            } else {
                robot_manage->exitHandguideMode();
                handguide_enabled_state_ = 0.0;
                handguide_mode_active_.store(false);
            }
        } catch (const std::exception &e) {
            handguide_mode_active_.store(false);
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Failed to set set_handguide_enable: %s", e.what());
        }
        handguide_enable_command_ = no_new_command;
    }

    bool has_tcp_offset_command = false;
    std::vector<double> tcp_offset(tcp_offset_state_.begin(),
                                   tcp_offset_state_.end());
    for (std::size_t i = 0; i < tcp_offset_commands_.size(); ++i) {
        if (!has_new_command(tcp_offset_commands_[i])) {
            continue;
        }
        tcp_offset[i] = tcp_offset_commands_[i];
        tcp_offset_commands_[i] = no_new_command;
        has_tcp_offset_command = true;
    }
    if (has_tcp_offset_command) {
        try {
            robot_config->setTcpOffset(tcp_offset);
            const std::size_t count =
                std::min(tcp_offset.size(), tcp_offset_state_.size());
            for (std::size_t i = 0; i < count; ++i) {
                tcp_offset_state_[i] = tcp_offset[i];
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Failed to set set_tcp_offset: %s", e.what());
        }
    }

    bool has_payload_command = has_new_command(payload_mass_command_);
    std::vector<double> payload_cog(payload_cog_state_.begin(),
                                    payload_cog_state_.end());
    for (std::size_t i = 0; i < payload_cog_commands_.size(); ++i) {
        if (!has_new_command(payload_cog_commands_[i])) {
            continue;
        }
        payload_cog[i] = payload_cog_commands_[i];
        payload_cog_commands_[i] = no_new_command;
        has_payload_command = true;
    }

    if (has_payload_command) {
        double payload_mass = payload_mass_state_;
        if (has_new_command(payload_mass_command_)) {
            payload_mass = payload_mass_command_;
        }

        std::vector<double> payload_aom;
        std::vector<double> payload_inertia;
        {
            std::unique_lock<std::mutex> lck(rtde_mtx_);
            payload_aom = std::get<2>(actual_payload_);
            payload_inertia = std::get<3>(actual_payload_);
        }

        try {
            robot_config->setPayload(payload_mass, payload_cog, payload_aom,
                                     payload_inertia);
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Failed to set payload command: %s", e.what());
        }
        payload_mass_command_ = no_new_command;
    }
}

// 设置rtde输入
void AuboHardwareInterface::setInput(RtdeClientPtr cli)
{
    // 接口调用: 发布
    // 组合设置输入
    cli->setTopic(
        true,
        { "input_bit_registers0_to_31", "input_bit_registers32_to_63",
          "input_bit_registers64_to_127", "input_int_registers_0" },
        1, 5);

    std::vector<int> value = { 0x00ff, 0x00, 0x00, 44 };
    cli->publish(
        5, [value](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value); });

    cli->setTopic(
        true, { "input_float_registers_0", "input_double_registers_1" }, 1, 6);

    std::vector<double> value2 = { 3.1, 4.1 };
    cli->publish(
        6, [value2](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value2); });
}
void AuboHardwareInterface::configSubscribe(RtdeClientPtr cli)
{
    // 接口调用: 设置 topic1
    int topic1 = cli->setTopic(
        false,
        { "R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode",
          "runtime_state", "line_number", "R1_actual_TCP_pose",
          "R1_actual_TCP_speed", "R1_actual_TCP_force", "R1_actual_tool_pose",
          "R1_standard_digital_input_bits", "R1_tool_digital_input_bits",
          "R1_configurable_digital_input_bits",
          "R1_standard_digital_output_bits", "R1_tool_digital_output_bits",
          "R1_configurable_digital_output_bits",
          "R1_standard_analog_input_values", "R1_tool_analog_input_values",
          "R1_standard_analog_output_values", "R1_tool_analog_output_values",
          "R1_actual_payload", "R1_tool_button_status", "R1_handle_status",
          "R1_handle_dev_state" },
        200, 0);
    // 接口调用: 订阅
    cli->subscribe(topic1, [this](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_ = parser.popVectorDouble();
        joint_velocity_ = parser.popVectorDouble();
        robot_mode_ = parser.popRobotModeType();
        safety_mode_ = parser.popSafetyModeType();
        runtime_state_ = parser.popRuntimeState();
        line_ = parser.popInt32();
        actual_TCP_pose_ = parser.popVectorDouble();
        actual_TCP_speed_ = parser.popVectorDouble();
        actual_TCP_force_ = parser.popVectorDouble();
        actual_tool_pose_ = parser.popVectorDouble();
        standard_digital_input_bits_ =
            static_cast<std::uint64_t>(parser.popInt64());
        tool_digital_input_bits_ =
            static_cast<std::uint64_t>(parser.popInt64());
        configurable_digital_input_bits_ =
            static_cast<std::uint64_t>(parser.popInt64());
        standard_digital_output_bits_ =
            static_cast<std::uint64_t>(parser.popInt64());
        tool_digital_output_bits_ =
            static_cast<std::uint64_t>(parser.popInt64());
        configurable_digital_output_bits_ =
            static_cast<std::uint64_t>(parser.popInt64());
        standard_analog_input_values_ = parser.popVectorDouble();
        tool_analog_input_values_ = parser.popVectorDouble();
        standard_analog_output_values_ = parser.popVectorDouble();
        tool_analog_output_values_ = parser.popVectorDouble();
        actual_payload_ = parser.popPayload();
        tool_button_status_ = parser.popBool();
        handle_status_ = static_cast<std::uint64_t>(parser.popInt64());
        handle_dev_state_ = static_cast<int>(parser.popHandleStateType());
    });
}
} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
