#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "aubo_hardware_interface.hpp"

namespace aubo_driver {
AuboHardwareInterface::~AuboHardwareInterface()
{
    stopServoMode();
}
hardware_interface::return_type AuboHardwareInterface::configure(
    const HardwareInfo &system_info)
{
    info_ = system_info;
    initialized_ = false;
    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        // RRBotSystemPositionOnly has exactly one state and command interface
        // on each joint
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("auboRBotSystemPositionOnlyHardware"),
                "Joint '%s' has %zu command interfaces found. 2 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return return_type::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("auboRBotSystemPositionOnlyHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return return_type::ERROR;
        }
        if (joint.command_interfaces[1].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("auboRBotSystemPositionOnlyHardware"),
                "Joint '%s' have %s command interfaces found as "
                "second command interface. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("auboRBotSystemPositionOnlyHardware"),
                "Joint '%s' has %zu state interface. 2 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return return_type::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("auboRBotSystemPositionOnlyHardware"),
                "Joint '%s' have %s state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return return_type::ERROR;
        }
        if (joint.state_interfaces[1].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("auboRBotSystemPositionOnlyHardware"),
                "Joint '%s' have %s state interface as second state "
                "interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return return_type::ERROR;
        }
    }

    status_ = status::CONFIGURED;

    return return_type::OK;
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
return_type AuboHardwareInterface::start()
{
    on_Active();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    readActualQ();
    aubo_position_commands_ = actual_q_copy_;
    initialized_ = true;
    status_ = hardware_interface::status::STARTED;
    return return_type::OK;
}

void AuboHardwareInterface::on_Active()
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Starting ...please wait...");
    const std::string robot_ip_ = info_.hardware_parameters["robot_ip"];
    rpc_client_ = std::make_shared<RpcClient>();

    rpc_client_->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    rpc_client_->connect(robot_ip_, 30004);
    // 接口调用: 登录
    rpc_client_->login("aubo", "123456");

    rtde_client_ = std::make_shared<RtdeClient>();
    // 接口调用: 连接到 RTDE 服务
    rtde_client_->connect(robot_ip_, 30010);
    // 接口调用: 登录
    rtde_client_->login("aubo", "123456");
    int topic = rtde_client_->setTopic(false, { "R1_message" }, 200, 0);
    if (topic < 0) {
        std::cout << "Set topic fail!" << std::endl;
    }
    rtde_client_->subscribe(topic, [](InputParser &parser) {
        arcs::common_interface::RobotMsgVector msgs;
        msgs = parser.popRobotMsgVector();
        for (size_t i = 0; i < msgs.size(); i++) {
            auto &msg = msgs[i];
        }
    });
    robot_name_ = rpc_client_->getRobotNames().front();
    // 设置rtde输入
    setInput(rtde_client_);

    // 配置输出
    configSubscribe(rtde_client_);

    startServoMode();

    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "System successfully started!");
}
return_type AuboHardwareInterface::stop()
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Stopping ...please wait...");
    stopServoMode();
    status_ = hardware_interface::status::STOPPED;
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "System successfully stopped!");
    return return_type::OK;
}
return_type AuboHardwareInterface::read()
{
    readActualQ();
    std::cout << "read--------------------" << std::endl;
    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        initialized_ = true;
    }
    return return_type::OK;
}
return_type AuboHardwareInterface::write()
{
    Servoj(aubo_position_commands_);
    std::cout << "write--------------------" << std::endl;
    return return_type::OK;
}

void AuboHardwareInterface::readActualQ()
{
    // 使用 actual_q_copy_
    // 固定该时间戳下read到的位姿，否则读取到的关节状态不稳定
    // actual_q_copy_必须用 array 否则会 bad_alloc
    actual_q_copy_[0] = actual_q_[0];
    actual_q_copy_[1] = actual_q_[1];
    actual_q_copy_[2] = actual_q_[2];
    actual_q_copy_[3] = actual_q_[3];
    actual_q_copy_[4] = actual_q_[4];
    actual_q_copy_[5] = actual_q_[5];

    //获取机械臂关节速度
    joint_velocity_ = rpc_client_->getRobotInterface(robot_name_)
                          ->getRobotState()
                          ->getJointSpeeds();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    joint_velocity_copy_[0] = joint_velocity_[0];
    joint_velocity_copy_[1] = joint_velocity_[1];
    joint_velocity_copy_[2] = joint_velocity_[2];
    joint_velocity_copy_[3] = joint_velocity_[3];
    joint_velocity_copy_[4] = joint_velocity_[4];
    joint_velocity_copy_[5] = joint_velocity_[5];
}

bool AuboHardwareInterface::isServoModeStart()
{
    return servo_mode_start_;
}
int AuboHardwareInterface::startServoMode()
{
    if (servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

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
    if (!servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

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
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    std::vector<double> traj(6, 0);
    for (size_t i = 0; i < traj.size(); i++) {
        traj[i] = joint_position_command[i];
    }

    // 接口调用: 关节运动
    int servoJoint_num = rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->servoJoint(traj, 0.2, 0.2, 0.005, 0.1, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    //    std::cout << "servoJoint finish!" << std::endl;

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
        { "R1_actual_q", "R1_actual_current", "R1_robot_mode", "R1_safety_mode",
          "runtime_state", "line_number", "R1_actual_TCP_pose" },
        500, 0);
    // 接口调用: 订阅
    cli->subscribe(topic1, [this](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_ = parser.popVectorDouble();
        actual_current_ = parser.popVectorDouble();
        robot_mode_ = parser.popRobotModeType();
        safety_mode_ = parser.popSafetyModeType();
        runtime_state_ = parser.popRuntimeState();
        line_ = parser.popInt32();
        actual_TCP_pose_ = parser.popVectorDouble();
    });
}

} // namespace aubo_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
