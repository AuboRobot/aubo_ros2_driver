#include "aubo_hardware_interface.h"

#include <chrono>
#include <exception>
#include <thread>

#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace aubo_driver {

AuboHardwareInterface::~AuboHardwareInterface()
{
    disconnectClients();
}

bool AuboHardwareInterface::OnActive()
{
    const auto logger = rclcpp::get_logger("AuboHardwareInterface");

    if (info_.hardware_parameters.find("robot_ip") ==
        info_.hardware_parameters.end()) {
        RCLCPP_ERROR(logger, "Missing required hardware parameter 'robot_ip'");
        return false;
    }

    robot_ip_ = info_.hardware_parameters["robot_ip"];

    try {
        rpc_client_ = std::make_shared<RpcClient>();

        rpc_client_->setRequestTimeout(1000);
        rpc_client_->connect(robot_ip_, 30004);
        rpc_client_->login("aubo", "123456");

        rtde_client_ = std::make_shared<RtdeClient>();
        rtde_client_->connect(robot_ip_, 30010);
        rtde_client_->login("aubo", "123456");
        const int topic = rtde_client_->setTopic(false, { "R1_message" }, 200, 0);
        if (topic < 0) {
            RCLCPP_ERROR(logger, "Failed to create RTDE message topic");
            disconnectClients();
            return false;
        }
        rtde_client_->subscribe(topic, [](InputParser &parser) {
            arcs::common_interface::RobotMsgVector msgs;
            msgs = parser.popRobotMsgVector();
            for (size_t i = 0; i < msgs.size(); ++i) {
                auto &msg = msgs[i];
                (void)msg;
            }
        });

        const auto robot_names = rpc_client_->getRobotNames();
        if (robot_names.empty()) {
            RCLCPP_ERROR(logger, "No robot name returned from controller");
            disconnectClients();
            return false;
        }
        robot_name_ = robot_names.front();

        rpc_client_->getRobotInterface(robot_name_)
            ->getRobotConfig()
            ->setHardwareCustomParameters("[joint_func] \n vff_enable = false\n");

        RCLCPP_INFO(logger, "Configured hardware parameter vff_enable=false");

        setInput(rtde_client_);
        configSubscribe(rtde_client_);

        if (startServoMode() != 0) {
            RCLCPP_ERROR(logger, "Failed to enter servo mode");
            disconnectClients();
            return false;
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger, "Hardware activation failed: %s", e.what());
        disconnectClients();
        return false;
    }

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
    aubo_position_commands_.fill(0.0);
    aubo_velocity_commands_.fill(0.0);
    actual_q_copy_.fill(0.0);
    joint_velocity_copy_.fill(0.0);

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
                "Joint '%s' has second command interface '%s'. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
                         "Joint '%s' has %zu state interfaces. 2 expected.",
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
                         "Joint '%s' has second state interface '%s'. '%s' expected.",
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
    (void)previous_state;
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Starting ...please wait...");
    if (!OnActive()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    readActualQ();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        initialized_ = true;
    }
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
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &actual_q_copy_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &joint_velocity_copy_[i]));
    }

    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
AuboHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &aubo_position_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &aubo_velocity_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type AuboHardwareInterface::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    readActualQ();
    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        initialized_ = true;
    }

    return hardware_interface::return_type::OK;
}
hardware_interface::return_type AuboHardwareInterface::write(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    if (robot_mode_ == RobotModeType::Running &&
        (safety_mode_ == SafetyModeType::Normal ||
         safety_mode_ == SafetyModeType::ReducedMode)) {
        try {
            Servoj(aubo_position_commands_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                         "Servoj failed: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    } else {
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
    // 使用 actual_q_copy_
    // 固定该时间戳下read到的位姿，否则读取到的关节状态不稳定
    // actual_q_copy_必须用 array 否则会 bad_alloc
    {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_copy_[0] = actual_q_[0];
        actual_q_copy_[1] = actual_q_[1];
        actual_q_copy_[2] = actual_q_[2];
        actual_q_copy_[3] = actual_q_[3];
        actual_q_copy_[4] = actual_q_[4];
        actual_q_copy_[5] = actual_q_[5];

    //获取机械臂关节速度

        joint_velocity_copy_[0] = joint_velocity_[0];
        joint_velocity_copy_[1] = joint_velocity_[1];
        joint_velocity_copy_[2] = joint_velocity_[2];
        joint_velocity_copy_[3] = joint_velocity_[3];
        joint_velocity_copy_[4] = joint_velocity_[4];
        joint_velocity_copy_[5] = joint_velocity_[5];
    }
}
// 设置rtde输入

bool AuboHardwareInterface::isServoModeStart()
{
    return servo_mode_start_;
}

int AuboHardwareInterface::startServoMode()
{
    if (!rpc_client_) {
        return -1;
    }
    if (servo_mode_start_) {
        return 0;
    }
    const auto robot_names = rpc_client_->getRobotNames();
    if (robot_names.empty()) {
        return -1;
    }
    auto robot_name = robot_names.front();

    //开启servo模式
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(true);
    int i = 0;
    while (!rpc_client_->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo Mode enable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    servo_mode_start_ = true;
    return 0;
}

int AuboHardwareInterface::stopServoMode()
{
    if (!rpc_client_) {
        servo_mode_start_ = false;
        return 0;
    }
    if (!servo_mode_start_) {
        return 0;
    }
    const auto robot_names = rpc_client_->getRobotNames();
    if (robot_names.empty()) {
        servo_mode_start_ = false;
        return 0;
    }
    auto robot_name = robot_names.front();

    while (!rpc_client_->getRobotInterface(robot_name)
                ->getRobotState()
                ->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // 关闭servo模式
    int i = 0;
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(false);
    while (rpc_client_->getRobotInterface(robot_name)
               ->getMotionControl()
               ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo Mode disable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Servoj end" << std::endl;
    servo_mode_start_ = false;
    return 0;
}

int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    if (!rpc_client_) {
        return -1;
    }
    const auto robot_names = rpc_client_->getRobotNames();
    if (robot_names.empty()) {
        return -1;
    }
    auto robot_name = robot_names.front();

    std::vector<double> traj(6, 0);
    for (size_t i = 0; i < traj.size(); i++) {
        traj[i] = joint_position_command[i];
    }
    
    if(!rpc_client_->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()){
                
        rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(true);           
    }
    // 接口调用: servoJoint
    while (true) {
        int servoJoint_num = rpc_client_->getRobotInterface(robot_name)
                                ->getMotionControl()
                                ->servoJoint(traj, 0.2, 0.2, 0.01, 0.1, 200);
        if(servoJoint_num != 2){
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
// 设置rtde输入
void AuboHardwareInterface::setInput(RtdeClientPtr cli)
{
    // 接口调用: 发布
    // 组合设置输入
    int topic5 = cli->setTopic(
        true,
        { "input_bit_registers0_to_31", "input_bit_registers32_to_63",
          "input_bit_registers64_to_127", "input_int_registers_0" },
        1, 5);

    std::vector<int> value = { 0x00ff, 0x00, 0x00, 44 };
    cli->publish(
        5, [value](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value); });

    int topic6 = cli->setTopic(
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
          "runtime_state", "line_number", "R1_actual_TCP_pose" },
        500, 0);
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
    });
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

    rpc_client_.reset();
    rtde_client_.reset();
    robot_name_.clear();
    servo_mode_start_ = false;
}
} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
