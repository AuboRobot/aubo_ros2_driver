#include "aubo_hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <exception>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <ctime>
#include <limits>
#include <thread>
#include <initializer_list>
#include <iostream>
namespace aubo_driver {

namespace {
const char * robotModeName(RobotModeType mode)
{
    switch (mode) {
    case RobotModeType::NoController: return "NoController";
    case RobotModeType::Disconnected: return "Disconnected";
    case RobotModeType::ConfirmSafety: return "ConfirmSafety";
    case RobotModeType::Booting: return "Booting";
    case RobotModeType::PowerOff: return "PowerOff";
    case RobotModeType::PowerOn: return "PowerOn";
    case RobotModeType::Idle: return "Idle";
    case RobotModeType::BrakeReleasing: return "BrakeReleasing";
    case RobotModeType::BackDrive: return "BackDrive";
    case RobotModeType::Running: return "Running";
    case RobotModeType::Maintaince: return "Maintaince";
    case RobotModeType::Error: return "Error";
    case RobotModeType::PowerOffing: return "PowerOffing";
    }
    return "Unknown";
}

const char * safetyModeName(SafetyModeType mode)
{
    switch (mode) {
    case SafetyModeType::Undefined: return "Undefined";
    case SafetyModeType::Normal: return "Normal";
    case SafetyModeType::ReducedMode: return "ReducedMode";
    case SafetyModeType::Recovery: return "Recovery";
    case SafetyModeType::Violation: return "Violation";
    case SafetyModeType::ProtectiveStop: return "ProtectiveStop";
    case SafetyModeType::SafeguardStop: return "SafeguardStop";
    case SafetyModeType::SystemEmergencyStop: return "SystemEmergencyStop";
    case SafetyModeType::RobotEmergencyStop: return "RobotEmergencyStop";
    case SafetyModeType::Fault: return "Fault";
    }
    return "Unknown";
}

bool isAbortRobotMode(RobotModeType mode)
{
    return mode == RobotModeType::NoController ||
           mode == RobotModeType::Disconnected ||
           mode == RobotModeType::ConfirmSafety ||
           mode == RobotModeType::Maintaince ||
           mode == RobotModeType::Error;
}

bool isFatalSafetyMode(SafetyModeType mode)
{
    return mode == SafetyModeType::Fault ||
           mode == SafetyModeType::RobotEmergencyStop ||
           mode == SafetyModeType::SystemEmergencyStop;
}

bool isPowerProgressMode(RobotModeType mode)
{
    return mode == RobotModeType::Booting ||
           mode == RobotModeType::PowerOn ||
           mode == RobotModeType::Idle ||
           mode == RobotModeType::BrakeReleasing ||
           mode == RobotModeType::BackDrive ||
           mode == RobotModeType::Running;
}

bool commandAccepted(int ret)
{
    return ret == 0 || ret == AUBO_BUSY;
}
}  // namespace


AuboHardwareInterface::~AuboHardwareInterface()
{
    disconnectClients();
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


bool AuboHardwareInterface::clearProtectiveStopIfNeeded()
{
    if (!rpc_client_ || robot_name_.empty()) {
        return false;
    }
    try {
        auto iface = rpc_client_->getRobotInterface(robot_name_);
        const auto safety_mode = iface->getRobotState()->getSafetyModeType();
        if (safety_mode != SafetyModeType::ProtectiveStop) {
            return true;
        }
        RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
            "[protective_stop] %s 处于 ProtectiveStop，调用 setUnlockProtectiveStop()",
            robot_name_.c_str());
        const int ret = iface->getRobotManage()->setUnlockProtectiveStop();
        if (ret != 0 && ret != AUBO_BUSY) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                "[protective_stop] setUnlockProtectiveStop failed ret=%d", ret);
            return false;
        }
        for (int i = 0; i < 40; ++i) {
            const auto current = iface->getRobotState()->getSafetyModeType();
            if (current == SafetyModeType::Normal ||
                current == SafetyModeType::ReducedMode) {
                RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                    "[protective_stop] 已解除，当前 %s", safetyModeName(current));
                return true;
            }
            if (i == 0 || i % 10 == 0) {
                RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                    "[protective_stop] 等待退出 ProtectiveStop，当前 %s",
                    safetyModeName(current));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "[protective_stop] 解除超时，当前 %s",
            safetyModeName(iface->getRobotState()->getSafetyModeType()));
        return false;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "[protective_stop] 异常: %s", e.what());
        return false;
    }
}

bool AuboHardwareInterface::ensureRobotRunning()
{
    if (!rpc_client_ || robot_name_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] 机器人接口为空");
        return false;
    }
    auto iface = rpc_client_->getRobotInterface(robot_name_);
    if (!iface || !iface->getRobotState() || !iface->getRobotManage()) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] 机器人接口为空");
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
    constexpr auto kPoll = std::chrono::milliseconds(200);
    const auto & name = robot_name_;

    auto read_state = [&](RobotModeType & mode, SafetyModeType & safety) -> bool {
        try {
            mode = iface->getRobotState()->getRobotModeType();
            safety = iface->getRobotState()->getSafetyModeType();
            return true;
        } catch (const std::exception & e) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                "[startup] %s 读取模式失败: %s", name.c_str(), e.what());
            return false;
        }
    };

    auto wait_mode = [&](std::initializer_list<RobotModeType> targets,
                         bool abort_on_power_off_regression,
                         RobotModeType & mode) -> bool {
        bool seen_progress = isPowerProgressMode(mode);
        RobotModeType last_logged = mode;
        while (std::chrono::steady_clock::now() < deadline) {
            SafetyModeType safety = SafetyModeType::Undefined;
            if (!read_state(mode, safety)) {
                std::this_thread::sleep_for(kPoll);
                continue;
            }
            if (mode != last_logged) {
                RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                    "[startup] %s 模式 %s，安全状态 %s",
                    name.c_str(), robotModeName(mode), safetyModeName(safety));
                last_logged = mode;
            }
            for (const auto target : targets) {
                if (mode == target) {
                    return true;
                }
            }
            if (isAbortRobotMode(mode) || isFatalSafetyMode(safety)) {
                RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                    "[startup] %s 上电中止，模式 %s，安全状态 %s",
                    name.c_str(), robotModeName(mode), safetyModeName(safety));
                return false;
            }
            if (abort_on_power_off_regression && seen_progress &&
                mode == RobotModeType::PowerOff) {
                RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                    "[startup] %s 上电回退到 PowerOff，安全状态 %s",
                    name.c_str(), safetyModeName(safety));
                return false;
            }
            if (isPowerProgressMode(mode)) {
                seen_progress = true;
            }
            std::this_thread::sleep_for(kPoll);
        }
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] %s 等待模式超时，当前 %s",
            name.c_str(), robotModeName(mode));
        return false;
    };

    RobotModeType mode = RobotModeType::NoController;
    SafetyModeType safety = SafetyModeType::Undefined;
    if (!read_state(mode, safety)) {
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
        "[startup] %s 当前模式 %s，安全状态 %s",
        name.c_str(), robotModeName(mode), safetyModeName(safety));

    if (mode == RobotModeType::Running) {
        RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] %s 已处于 Running，跳过上电与启动", name.c_str());
        return true;
    }
    if (isAbortRobotMode(mode) || isFatalSafetyMode(safety)) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] %s 无法上电，模式 %s，安全状态 %s",
            name.c_str(), robotModeName(mode), safetyModeName(safety));
        return false;
    }

    if (mode == RobotModeType::PowerOffing) {
        if (!wait_mode(
                {RobotModeType::PowerOff, RobotModeType::Idle, RobotModeType::Running},
                false, mode)) {
            return false;
        }
        if (mode == RobotModeType::Running) {
            return true;
        }
    }

    if (mode == RobotModeType::PowerOff) {
        int ret = -1;
        try {
            ret = iface->getRobotManage()->poweron();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                "[startup] %s poweron 异常: %s", name.c_str(), e.what());
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] %s 请求上电 poweron ret=%d", name.c_str(), ret);
        if (!commandAccepted(ret)) {
            return false;
        }
        if (!wait_mode({RobotModeType::Idle, RobotModeType::Running}, true, mode)) {
            return false;
        }
    } else if (mode == RobotModeType::BrakeReleasing) {
        RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] %s 刹车松开中，跳过 poweron", name.c_str());
        if (!wait_mode({RobotModeType::Running}, true, mode)) {
            return false;
        }
    } else {
        RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] %s 已上电（模式 %s），跳过 poweron",
            name.c_str(), robotModeName(mode));
        if (mode != RobotModeType::Idle && mode != RobotModeType::BackDrive &&
            mode != RobotModeType::Running) {
            if (!wait_mode(
                    {RobotModeType::Idle, RobotModeType::Running,
                     RobotModeType::BackDrive, RobotModeType::BrakeReleasing},
                    true, mode)) {
                return false;
            }
        }
    }

    if (mode == RobotModeType::Running) {
        return true;
    }
    if (mode == RobotModeType::BrakeReleasing) {
        if (!wait_mode({RobotModeType::Running}, true, mode)) {
            return false;
        }
        return true;
    }

    int ret = -1;
    try {
        ret = iface->getRobotManage()->startup();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "[startup] %s startup 异常: %s", name.c_str(), e.what());
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
        "[startup] %s 请求启动 startup ret=%d，当前模式 %s",
        name.c_str(), ret, robotModeName(mode));
    if (!commandAccepted(ret)) {
        return false;
    }
    if (!wait_mode({RobotModeType::Running}, true, mode)) {
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
        "[startup] %s 已进入 Running", name.c_str());
    return true;
}


bool AuboHardwareInterface::OnActive()
{
    if (!connectRpcClient()) {
        return false;
    }
    if (!connectRtdeClient()) {
        return false;
    }

    // 对齐 dual：激活前先解除防护停止，再自动上电到 Running
    if (!clearProtectiveStopIfNeeded()) {
        RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
            "protective stop unlock failed before power-on");
        return false;
    }
    if (!ensureRobotRunning()) {
        RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
            "自动上电/启动失败，机械臂未进入 Running。");
        return false;
    }
    if (!clearProtectiveStopIfNeeded()) {
        RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
            "protective stop unlock failed before servo-ready");
        return false;
    }

    rpc_client_->getRobotInterface(robot_name_)
    ->getRobotConfig()
    ->setHardwareCustomParameters("[joint_func] \n vff_enable = false\n");

    std::cout << "vff_enable = false" << std::endl;

    // Do not write the SDK example register values on a real robot.

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
            std::size_t used = 0;
            servo_mode_ = std::stoi(servo_mode_param->second, &used);
            if (used != servo_mode_param->second.size() || servo_mode_ < 1 || servo_mode_ > 7)
                throw std::invalid_argument("active servo_mode must be 1..7; 0 exits Servo");
        } catch (const std::exception &e) {
            RCLCPP_FATAL(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Invalid servo_mode parameter '%s': %s",
                servo_mode_param->second.c_str(), e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    try {
        auto read_period = [this](const char *name, double fallback) {
            auto it = info_.hardware_parameters.find(name);
            if (it == info_.hardware_parameters.end()) return fallback;
            std::size_t used = 0;
            double value = std::stod(it->second, &used);
            if (used != it->second.size() || !std::isfinite(value) || value <= 0)
                throw std::invalid_argument(name);
            return value;
        };
        control_period_s_ = read_period("control_period", 0.005);
        servo_period_s_ = read_period("servo_period", 0.01);
        if (servo_period_s_ < 0.002 || servo_period_s_ > 0.02 ||
            control_period_s_ >= 5.0 * servo_period_s_)
            throw std::invalid_argument("control/servo period outside supported watchdog budget");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "Invalid cyclic timing: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
        "Cyclic timing: nominal_period_ms=%.3f servo_t_ms=%.3f watchdog_ms=%.3f",
        control_period_s_*1000, servo_period_s_*1000, servo_period_s_*5000);

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
    try {
        if (!OnActive()) return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "Hardware activation failed: %s", e.what());
        disconnectClients();
        return hardware_interface::CallbackReturn::ERROR;
    }
    // Activation is outside the cyclic write path. Require real feedback before
    // seeding commands; zero-filled startup buffers are not measured positions.
    bool ready = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(rtde_mtx_);
            ready = last_motion_feedback_.time_since_epoch().count() != 0 &&
                std::chrono::steady_clock::now() - last_motion_feedback_ <
                    std::chrono::milliseconds(100) &&
                std::all_of(joint_velocity_.begin(), joint_velocity_.end(),
                    [](double v) { return std::isfinite(v) && std::abs(v) < 0.02; });
        }
        if (ready) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!ready) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Activation requires fresh RTDE feedback and a stationary robot.");
        disconnectClients();
        return hardware_interface::CallbackReturn::ERROR;
    }
    // Clear residual cabinet mode only during explicit lifecycle activation.
    if (stopServoMode() != 0) {
        disconnectClients();
        return hardware_interface::CallbackReturn::ERROR;
    }
    {
        std::lock_guard<std::mutex> lock(rtde_mtx_);
        ready = last_motion_feedback_.time_since_epoch().count() != 0 &&
            std::chrono::steady_clock::now() - last_motion_feedback_ <
                std::chrono::milliseconds(100) &&
            std::all_of(joint_velocity_.begin(), joint_velocity_.end(),
                [](double v) { return std::isfinite(v) && std::abs(v) < 0.02; });
        if (ready) std::copy(actual_q_.begin(), actual_q_.end(), aubo_position_commands_.begin());
    }
    if (!ready) {
        disconnectClients();
        return hardware_interface::CallbackReturn::ERROR;
    }
    servo_fault_latched_ = false;
    servo_mode_pending_ = false;
    initialized_ = true;
    servo_return_policy_.reset();
    last_servo_attempt_ = {};
    last_servo_report_ = {};
    readActualQ();

    std::fill(aubo_velocity_commands_.begin(), aubo_velocity_commands_.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    disconnectClients();
    initialized_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    disconnectClients();
    initialized_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    disconnectClients();
    initialized_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_error(
    const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    disconnectClients();
    initialized_ = false;
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
    if (servo_fault_latched_) return hardware_interface::return_type::ERROR;
    checkAsyncSdkCommands();

    if (handguide_mode_active_.load()) {
        return hardware_interface::return_type::OK;
    }

    const bool velocity_controller_running =
        velocity_controller_running_.load();
    const bool position_controller_running =
        position_controller_running_.load();
    const bool motion_controller_running =
        velocity_controller_running || position_controller_running;

    RobotModeType robot_mode;
    SafetyModeType safety_mode;
    bool feedback_fresh;
    {
        std::lock_guard<std::mutex> lock(rtde_mtx_);
        robot_mode = robot_mode_;
        safety_mode = safety_mode_;
        feedback_fresh = last_motion_feedback_.time_since_epoch().count() != 0 &&
            std::chrono::steady_clock::now() - last_motion_feedback_ <
                std::chrono::milliseconds(100);
    }
    if (motion_controller_running && !feedback_fresh)
        return latchServoFault("RTDE motion feedback stale (>100 ms)");
    if (robot_mode != RobotModeType::Running ||
        (safety_mode != SafetyModeType::Normal &&
         safety_mode != SafetyModeType::ReducedMode)) {
        if (servo_mode_start_ || servo_mode_pending_)
            return latchServoFault("robot left motion-ready state during servo session");
        servo_mode_start_ = false;

        if (motion_controller_running && !motion_command_paused_) {
            RCLCPP_WARN_STREAM(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Motion command paused until robot is running and safety "
                "state is normal or reduced. robot_mode_: "
                    << static_cast<int>(robot_mode)
                    << ", safety_mode_: " << static_cast<int>(safety_mode));
            motion_command_paused_ = true;
        }

        return hardware_interface::return_type::OK;
    }

    if (motion_command_paused_ && motion_controller_running) {
        if (position_controller_running) {
            std::unique_lock<std::mutex> lck(rtde_mtx_);
            const std::size_t count =
                std::min(actual_q_.size(), aubo_position_commands_.size());
            for (std::size_t i = 0; i < count; ++i) {
                aubo_position_commands_[i] = actual_q_[i];
            }
        }
        if (velocity_controller_running) {
            std::fill(aubo_velocity_commands_.begin(),
                      aubo_velocity_commands_.end(), 0.0);
        }
        servo_mode_start_ = false;
        RCLCPP_INFO(
            rclcpp::get_logger("AuboHardwareInterface"),
            "Robot returned to motion-ready state. Servo mode will be "
            "entered again on the next motion command.");
    }
    motion_command_paused_ = false;

    if (!velocity_controller_running && !position_controller_running) {
        if ((servo_mode_start_ || servo_mode_pending_) && stopServoMode() != 0) {
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    try {
        if (velocity_controller_running) {
            if (speedServo(aubo_velocity_commands_) != 0)
                return latchServoFault("velocity backend failed");
        } else if (position_controller_running) {
            if (Servoj(aubo_position_commands_) != 0)
                return latchServoFault("servoJoint or mode entry failed");
        }
    } catch (const std::exception &e) {
        servo_mode_start_ = false;
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Motion command failed: %s", e.what());
        return latchServoFault("motion RPC exception");
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuboHardwareInterface::latchServoFault(const char *reason)
{
    if (!servo_fault_latched_) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "Servo fault latched: %s. Motion is rejected until explicit hardware "
            "deactivation/activation with fresh stationary feedback.", reason);
    }
    servo_fault_latched_ = true;
    // Keep mode ownership until lifecycle cleanup can explicitly disable it.
    return hardware_interface::return_type::ERROR;
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
    if (servo_fault_latched_ || !rpc_client_ || robot_name_.empty()) return -1;
    if (servo_mode_start_) return 0;

    try {
        auto motion_control =
            rpc_client_->getRobotInterface(robot_name_)->getMotionControl();
        // Mode entry progresses over cycles; never sleep in write().
        const auto now = std::chrono::steady_clock::now();
        if (!servo_mode_pending_) {
            const int result = motion_control->setServoModeSelect(servo_mode_);
            if (result != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                    "setServoModeSelect returned %d", result);
                return -1;
            }
            servo_mode_pending_ = true;
            servo_mode_deadline_ = now + std::chrono::milliseconds(150);
            return 1;
        }
        if (now >= servo_mode_deadline_) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                "Servo mode entry timed out before query; explicit recovery required.");
            return -1;
        }
        const int selected = motion_control->getServoModeSelect();
        if (std::chrono::steady_clock::now() >= servo_mode_deadline_) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                "Servo mode entry reply arrived after deadline; explicit recovery required.");
            return -1;
        }
        if (selected != servo_mode_) return 1;
        servo_mode_pending_ = false;
    } catch (const std::exception &e) {
        servo_mode_start_ = false;
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to enable servo mode select %d: %s", servo_mode_,
                     e.what());
        return -1;
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
            servo_mode_pending_ = false;
            last_servo_attempt_ = {};
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
    servo_mode_pending_ = false;
    last_servo_attempt_ = {};
    return 0;
}

int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    const int mode_result = startServoMode();
    if (mode_result < 0) return -1;
    if (mode_result > 0) return 0; // Entry pending; no trajectory point queued.

    if (!std::all_of(joint_position_command.begin(), joint_position_command.end(),
                     [](double q) { return std::isfinite(q); })) return -1;
    std::copy(joint_position_command.begin(), joint_position_command.end(), servo_target_.begin());
    const auto begin = std::chrono::steady_clock::now();
    const double gap_ms = last_servo_attempt_.time_since_epoch().count() == 0 ? 0.0 :
        std::chrono::duration<double, std::milli>(begin - last_servo_attempt_).count();
    if (gap_ms >= 5000.0 * servo_period_s_) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "Servo attempt gap %.3f ms reached 5*t; refusing stale-session command.", gap_ms);
        return -1;
    }
    last_servo_attempt_ = begin;
    // One attempt per cycle. QUEUE_FULL/BUSY never sleep or replay an old target.
    const int result = rpc_client_->getRobotInterface(robot_name_)
        ->getMotionControl()->servoJoint(servo_target_, 0.2, 0.2, servo_period_s_, 0.1, 200);
    const auto end = std::chrono::steady_clock::now();
    const double rpc_ms = std::chrono::duration<double, std::milli>(end - begin).count();
    const bool ok = servo_return_policy_.observe(result,
        std::chrono::duration_cast<std::chrono::nanoseconds>(end.time_since_epoch()).count());
    if (!ok || ((result != 0 || rpc_ms > 1000.0 * control_period_s_ || gap_ms > 25.0) &&
        (last_servo_report_.time_since_epoch().count() == 0 ||
         end - last_servo_report_ >= std::chrono::seconds(5)))) {
        RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
            "servoJoint result=%d rpc_ms=%.3f attempt_gap_ms=%.3f fault=%d "
            "(2=queue full, 3=busy; neither acknowledges this target)",
            result, rpc_ms, gap_ms, !ok);
        last_servo_report_ = end;
    }
    // A late RPC reply is not proof that the cabinet still has a live session.
    if (rpc_ms >= 5000.0 * servo_period_s_) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
            "servoJoint RPC %.3f ms reached 5*t; explicit recovery required.", rpc_ms);
        return -1;
    }
    return ok ? 0 : -1;
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
    auto pending = [](double v) { return !std::isnan(v); };
    const bool has_command =
        pending(digital_output_trigger_command_) || pending(analog_output_trigger_command_) ||
        pending(tool_voltage_output_domain_command_) || pending(tool_io_input_trigger_command_) ||
        pending(tool_io_config_trigger_command_) || pending(payload_mass_command_) ||
        pending(handguide_enable_command_) ||
        std::any_of(tcp_offset_commands_.begin(), tcp_offset_commands_.end(), pending) ||
        std::any_of(payload_cog_commands_.begin(), payload_cog_commands_.end(), pending) ||
        std::any_of(handguide_free_axis_commands_.begin(), handguide_free_axis_commands_.end(), pending) ||
        std::any_of(handguide_feature_commands_.begin(), handguide_feature_commands_.end(), pending);
    if (!has_command) return;
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

void AuboHardwareInterface::configSubscribe(RtdeClientPtr cli)
{
    const int motion_topic = cli->setTopic(false,
        {"R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode",
         "runtime_state", "line_number"}, 200, 0);
    if (motion_topic < 0) throw std::runtime_error("RTDE motion recipe rejected");
    if (cli->subscribe(motion_topic, [this](InputParser &parser) {
        // Parse first, commit together. Failed parsing never refreshes freshness.
        auto q = parser.popVectorDouble();
        auto qd = parser.popVectorDouble();
        auto mode = parser.popRobotModeType();
        auto safety = parser.popSafetyModeType();
        auto runtime = parser.popRuntimeState();
        auto line = parser.popInt32();
        if (q.size() != 6 || qd.size() != 6 ||
            !std::all_of(q.begin(), q.end(), [](double v) { return std::isfinite(v); }) ||
            !std::all_of(qd.begin(), qd.end(), [](double v) { return std::isfinite(v); })) return;
        std::lock_guard<std::mutex> lock(rtde_mtx_);
        actual_q_ = std::move(q);
        joint_velocity_ = std::move(qd);
        robot_mode_ = mode;
        safety_mode_ = safety;
        runtime_state_ = runtime;
        line_ = line;
        last_motion_feedback_ = std::chrono::steady_clock::now();
    }) != 0) throw std::runtime_error("RTDE motion subscription failed");
    // Large TCP/IO/payload fields do not need the joint control rate.
    const int status_topic = cli->setTopic(false,
        {"R1_actual_TCP_pose", "R1_actual_TCP_speed", "R1_actual_TCP_force",
         "R1_actual_tool_pose", "R1_standard_digital_input_bits",
         "R1_tool_digital_input_bits", "R1_configurable_digital_input_bits",
         "R1_standard_digital_output_bits", "R1_tool_digital_output_bits",
         "R1_configurable_digital_output_bits", "R1_standard_analog_input_values",
         "R1_tool_analog_input_values", "R1_standard_analog_output_values",
         "R1_tool_analog_output_values", "R1_actual_payload", "R1_tool_button_status",
         "R1_handle_status", "R1_handle_dev_state"}, 50, 1);
    if (status_topic < 0) throw std::runtime_error("RTDE status recipe rejected");
    if (cli->subscribe(status_topic, [this](InputParser &parser) {
        std::lock_guard<std::mutex> lock(rtde_mtx_);
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
    }) != 0) throw std::runtime_error("RTDE status subscription failed");
}

void AuboHardwareInterface::disconnectClients()
{
    try {
        stopServoMode();
    } catch (...) {
    }

    try {
        if (rpc_client_) {
            rpc_client_->logout();
        }
    } catch (...) {
    }

    try {
        if (rtde_client_) {
            rtde_client_->logout();
        }
    } catch (...) {
    }

    rpc_client_.reset();
    rtde_client_.reset();
    robot_name_.clear();
    servo_mode_start_ = false;
    servo_mode_pending_ = false;
    position_controller_running_.store(false);
    velocity_controller_running_.store(false);
    motion_command_paused_ = false;
    std::lock_guard<std::mutex> lock(rtde_mtx_);
    last_motion_feedback_ = {};
}

} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
