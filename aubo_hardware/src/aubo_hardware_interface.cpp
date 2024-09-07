#include "aubo_hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <ctime>
namespace aubo_hardware {
    static rclcpp::Logger LOGGER = rclcpp::get_logger("AUBO.RobotHW");
AuboHardwareInterface::~AuboHardwareInterface()
{
    stopServoMode();
}

template <typename T>
void printVec(std::vector<T> param, std::string name)
{
    std::cout << "@:" << name << std::endl;
    for (int i = 0; i < param.size(); i++) {
        std::cout << param.at(i) << ",";
    }
    std::cout << std::endl;
};
void printArray(const std::array<double, 6>& arr, std::string name) {
    std::cout << "@:" << name << std::endl;
    for (const auto& element : arr) {
        std::cout << element << " ";
    }
    std::cout << std::endl;
}

template <size_t N, size_t M>
std::bitset<N + M> concatenateBitsets(const std::bitset<N> &b1,
                                      const std::bitset<M> &b2)
{
    std::bitset<N + M> result;
    for (size_t i = 0; i < N; i++) {
        result[i] = b1[i];
    }
    for (size_t i = 0; i < M; i++) {
        result[i + N] = b2[i];
    }
    return result;
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
    async_thread_shutdown_ = false;
    system_interface_initialized_ = 0.0;

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                LOGGER,
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                LOGGER,
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(LOGGER,
                         "Joint '%s' has %zu state interface. 1 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(LOGGER,
                         "Joint '%s' have %s state interface. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> AuboHardwareInterface::export_state_interfaces()
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
    std::string tf_prefix = "";
    try {
        tf_prefix = info_.hardware_parameters["tf_prefix"];
    } catch (const std::out_of_range& oor) {
        tf_prefix = "";
    }
    if (tf_prefix != "") {
        LOGGER = rclcpp::get_logger("AUBO." + tf_prefix + "RobotHW");
        RCLCPP_INFO(LOGGER, "tf_prefix is [%s]", tf_prefix.c_str());
    }else{
        RCLCPP_INFO(LOGGER, "export_state_interfaces ---- tf_prefix is null");
    }
    for (size_t i = 0; i < 12; ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "digital_output_" + std::to_string(i), &actual_dig_out_bits_copy_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits_copy_[i]));
    }
    for (size_t i = 0; i < 2; ++i) {
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "tool_analog_input_" + std::to_string(i), &tool_analog_input_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            tf_prefix + "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output_[i]));
    }
    state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized",
                                                                    &system_interface_initialized_));

    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface> AuboHardwareInterface::export_command_interfaces()
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
    std::string tf_prefix = "";
    try {
        tf_prefix = info_.hardware_parameters["tf_prefix"];
        rtu_device_name = info_.hardware_parameters["rtu_device_name"];
    } catch (const std::out_of_range& oor) {
        tf_prefix = "";
        rtu_device_name = "";
    }
    if (tf_prefix != "") {
        LOGGER = rclcpp::get_logger("AUBO." + tf_prefix + "RobotHW");
        RCLCPP_INFO(LOGGER, "tf_prefix is [%s]", tf_prefix.c_str());
    }else{
        RCLCPP_INFO(LOGGER, "export_command_interfaces ---------- tf_prefix is null");
    }
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "gpio", "io_async_success", &io_async_success_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + "payload", "mass", &payload_mass_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "payload", "cog.x", &payload_center_of_gravity_[0]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "payload", "cog.y", &payload_center_of_gravity_[1]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "payload", "cog.z", &payload_center_of_gravity_[2]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "payload", "payload_async_success", &payload_async_success_));

    for (size_t i = 0; i < 12; ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            tf_prefix + "gpio", "standard_digital_output_cmd_" + std::to_string(i), &standard_dig_out_bits_cmd_[i]));
    }

    for (size_t i = 0; i < 2; ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            tf_prefix + "gpio", "standard_analog_output_cmd_" + std::to_string(i), &standard_analog_output_cmd_[i]));
    }


    command_interfaces.emplace_back(
    hardware_interface::CommandInterface(tf_prefix + "modbus", "slave_number", &modbus_slave_number_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "modbus", "signal_address", &modbus_signal_address_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "modbus", "signal_type", &modbus_signal_type_));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "modbus", "signal_name_a_d", &modbus_signal_name_a_d_)); 
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "modbus", "signal_name_get", &modbus_signal_name_get_)); 
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "modbus", "signal_name_set", &modbus_signal_name_set_));   
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "modbus", "set_signal_value", &modbus_set_signal_value_));   
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(tf_prefix + "modbus", "modbus_async_success", &modbus_async_success_)); 
        
    return command_interfaces;
}


bool AuboHardwareInterface::OnActive()
{
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
    // int topic = rtde_client_->setTopic(false, { "R1_message" }, 200, 0);
    // if (topic < 0) {
    //     std::cout << "Set topic fail!" << std::endl;
    // }
    // rtde_client_->subscribe(topic, [](InputParser &parser) {
    //     arcs::common_interface::RobotMsgVector msgs;
    //     msgs = parser.popRobotMsgVector();
    //     for (size_t i = 0; i < msgs.size(); i++) {
    //         auto &msg = msgs[i];
    //     }
    // });
    robot_name_ = rpc_client_->getRobotNames().front();
    // 设置rtde输入
    // setInput(rtde_client_);

    //获取Tool IO类型
    getToolIOType(rpc_client_);
    // 配置输出
    configSubscribe(rtde_client_);

    startServoMode();

    return true;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(LOGGER,
                "Starting ...please wait...");
    OnActive();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    readActualQ();
    readIO();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!initialized_) {
        initAsyncIO();
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        printArray(aubo_position_commands_,"aubo_position_commands_");
        initialized_ = true;
    }
    async_thread_ = std::make_shared<std::thread>(&AuboHardwareInterface::asyncThread, this);
    RCLCPP_INFO(LOGGER, "System successfully started!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AuboHardwareInterface::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    readActualQ();
    readIO();
    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        printArray(aubo_position_commands_,"aubo_position_commands_");
        initialized_ = true;
    }
    updateNonDoubleValues();
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type AuboHardwareInterface::write(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    
    bool allNonZero  = false;
    for (const auto& position : aubo_position_commands_) {
        if (position == 0.0) {
            allNonZero = false;
            break;
        }
    }
    if (!allNonZero) {
    // if (1) {
        try {
            Servoj(aubo_position_commands_);
        } catch (const std::exception &e) {
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(LOGGER, "Stopping ...please wait...");

  async_thread_shutdown_ = true;
  async_thread_->join();
  async_thread_.reset();
  
  RCLCPP_INFO(LOGGER, "System successfully stopped!");
  stopServoMode();
  return hardware_interface::CallbackReturn::SUCCESS;
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

void AuboHardwareInterface::readIO()
{
    {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        std::bitset<8> s_dig_out_bits(standard_digital_output_);
        std::bitset<4> t_dig_out_bits(tool_digital_output_);
        actual_dig_out_bits_ = concatenateBitsets(s_dig_out_bits, t_dig_out_bits);
        std::bitset<8> s_dig_in_bits(standard_digital_input_);
        std::bitset<4> t_dig_in_bits(tool_digital_input_);
        actual_dig_in_bits_ = concatenateBitsets(s_dig_in_bits, t_dig_in_bits);

        std::copy(tool_analog_input_values_.begin(), tool_analog_input_values_.end(), tool_analog_input_.begin());
        std::copy(standard_analog_input_values_.begin(), standard_analog_input_values_.end(), standard_analog_input_.begin());
        std::copy(standard_analog_output_values_.begin(), standard_analog_output_values_.end(), standard_analog_output_.begin());
    }
}
// 设置rtde输入

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

void AuboHardwareInterface::asyncThread()
{
  while (!async_thread_shutdown_) {
    if (initialized_) {
        // RCLCPP_INFO(LOGGER, "Initialized in async thread");
        checkAsyncIO();
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(20000000));
  }
}

void AuboHardwareInterface::initAsyncIO()
{
  for (size_t i = 0; i < 12; ++i) {
    standard_dig_out_bits_cmd_[i] = NO_NEW_CMD_;
  }
  for (size_t i = 0; i < 2; ++i) {
    standard_analog_output_cmd_[i] = NO_NEW_CMD_;
  }

  payload_mass_ = NO_NEW_CMD_;
  payload_center_of_gravity_ = { NO_NEW_CMD_, NO_NEW_CMD_, NO_NEW_CMD_ };
//   modbus_device_info_ = "";
  modbus_signal_name_a_d_ = NO_NEW_CMD_;
  modbus_signal_name_get_ = NO_NEW_CMD_;
  modbus_signal_name_set_ = NO_NEW_CMD_;
  modbus_slave_number_ = NO_NEW_CMD_;
  modbus_signal_address_ = NO_NEW_CMD_;
  modbus_signal_type_ = NO_NEW_CMD_;
  modbus_set_signal_value_ = NO_NEW_CMD_;
  modbus_async_success_ = NO_NEW_CMD_;
}

void AuboHardwareInterface::checkAsyncIO()
{
    if(rpc_client_ != nullptr && rtde_client_ != nullptr){
        // 接口调用: 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();
    auto robot = rpc_client_->getRobotInterface(robot_name);
    auto robot_config = rpc_client_->getRobotInterface(robot_name)->getRobotConfig();
    auto robot_register = rpc_client_->getRegisterControl();
    for (size_t i = 0; i < 12; ++i) {
        if (!std::isnan(standard_dig_out_bits_cmd_[i])) {
        if (i <= 7) {
            io_async_success_ =
            !(robot->getIoControl()->setStandardDigitalOutput(i, static_cast<bool>(standard_dig_out_bits_cmd_[i])));
        } else {
            for (int j = 0; j < tool_digital_num_; j++) {
                if (!tool_is_input_[j] && (i - 8) == j) {
                    io_async_success_ = !(robot->getIoControl()->setToolDigitalOutput(
                static_cast<uint8_t>(i - 8), static_cast<bool>(standard_dig_out_bits_cmd_[i])));
                }
            }
        }
        standard_dig_out_bits_cmd_[i] = NO_NEW_CMD_;
        }
    }
    for (size_t i = 0; i < 2; ++i) {
        if (!std::isnan(standard_analog_output_cmd_[i])) {
        io_async_success_ = !(robot->getIoControl()->setStandardAnalogOutput(i, standard_analog_output_cmd_[i]));
        standard_analog_output_cmd_[i] = NO_NEW_CMD_;
        }
    }
    if (!std::isnan(payload_mass_) && !std::isnan(payload_center_of_gravity_[0]) &&
        !std::isnan(payload_center_of_gravity_[1]) && !std::isnan(payload_center_of_gravity_[2])) {
        std::vector<double> cog_vec(payload_center_of_gravity_.begin(), payload_center_of_gravity_.end());
        std::vector<double> aom_vec(3, 0.0);
        std::vector<double> inertia_vec(9, 0.0);
        payload_async_success_ = !(robot_config->setPayload(payload_mass_, cog_vec, aom_vec, inertia_vec));
        payload_mass_ = NO_NEW_CMD_;
        payload_center_of_gravity_ = { NO_NEW_CMD_, NO_NEW_CMD_, NO_NEW_CMD_ };
    }

    if (rtu_device_name !="" && !std::isnan(modbus_signal_name_a_d_)  && !std::isnan(modbus_slave_number_) && !std::isnan(modbus_signal_address_) &&
        !std::isnan(modbus_signal_type_)) {

        modbus_async_success_ = !(robot_register->modbusAddSignal(rtu_device_name, static_cast<int>(modbus_slave_number_), static_cast<int>(modbus_signal_address_),
         static_cast<int>(modbus_signal_type_), "Modbus_" + std::to_string(static_cast<int>(modbus_signal_name_a_d_)), true));
        // modbus_device_info_ = "";
        modbus_signal_name_a_d_ = NO_NEW_CMD_;
        modbus_slave_number_ = NO_NEW_CMD_;
        modbus_signal_address_ = NO_NEW_CMD_;
        modbus_signal_type_ = NO_NEW_CMD_;
        // modbus_sequential_mode_ = NO_NEW_CMD_;
    }
    if (!std::isnan(modbus_signal_name_a_d_)  && std::isnan(modbus_slave_number_) && std::isnan(modbus_signal_address_) &&
        std::isnan(modbus_signal_type_) && std::isnan(modbus_signal_name_get_) && std::isnan(modbus_signal_name_set_)) {

        modbus_async_success_ = !(robot_register->modbusDeleteSignal("Modbus_" + std::to_string(static_cast<int>(modbus_signal_name_a_d_))));
        // modbus_device_info_ = "";
        modbus_signal_name_a_d_ = NO_NEW_CMD_;
    }

    if (!std::isnan(modbus_signal_name_get_) && std::isnan(modbus_signal_name_a_d_)  && std::isnan(modbus_slave_number_) && std::isnan(modbus_signal_address_) &&
        std::isnan(modbus_signal_type_)  && std::isnan(modbus_signal_name_set_)) {

        modbus_async_success_ = static_cast<double>(robot_register->modbusGetSignalStatus("Modbus_" + std::to_string(static_cast<int>(modbus_signal_name_get_))));
        modbus_signal_name_get_ = NO_NEW_CMD_;
    }

    if (!std::isnan(modbus_signal_name_set_) && !std::isnan(modbus_set_signal_value_) && std::isnan(modbus_signal_name_a_d_)  && std::isnan(modbus_slave_number_) && std::isnan(modbus_signal_address_) &&
        std::isnan(modbus_signal_type_)  && std::isnan(modbus_signal_name_get_)) {

        modbus_async_success_ = !(robot_register->modbusSetOutputSignal("Modbus_" + std::to_string(static_cast<int>(modbus_signal_name_set_)),static_cast<u_int16_t>(modbus_set_signal_value_)));
        modbus_signal_name_set_ = NO_NEW_CMD_;
        modbus_set_signal_value_ = NO_NEW_CMD_;
    }

    }
}

void AuboHardwareInterface::updateNonDoubleValues()
{
  for (size_t i = 0; i < 12; ++i) {
    actual_dig_out_bits_copy_[i] = static_cast<double>(actual_dig_out_bits_[i]);
    actual_dig_in_bits_copy_[i] = static_cast<double>(actual_dig_in_bits_[i]);
  }

//   robot_mode_copy_ = static_cast<double>(robot_mode_);
//   safety_mode_copy_ = static_cast<double>(safety_mode_);
  system_interface_initialized_ = initialized_ ? 1.0 : 0.0;

}

bool AuboHardwareInterface::check_command_changed(std::array<double, 6> prev_command, std::array<double, 6> current_command, double threshold_min_, double threshold_max_) {
    for (std::size_t i = 0; i < prev_command.size(); i++) {
        if (threshold_min_ < std::abs(prev_command[i] - current_command[i]) < threshold_max_ ) {
            return true;
        }
    }
    return false;
}

int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();
    // std::fstream outfile;
    // outfile.open("servoj_data.txt",std::ios::app);

    // time_t t = time(nullptr);
    // struct tm * now = localtime(&t);
    // std::stringstream timeStr;
    // timeStr << now->tm_year + 1900 <<"y";
    // timeStr << now->tm_mon + 1 <<"mon";
    // timeStr << now->tm_mday <<"d";
    // timeStr << now->tm_hour <<":";
    // timeStr <<now->tm_min<<":";
    // timeStr << now->tm_sec;
    // outfile<<timeStr.str()<<",";
    
    std::vector<double> cmd_joint_position_(6, 0);
    for (size_t i = 0; i < joint_position_command.size(); i++) {
        cmd_joint_position_[i] = joint_position_command[i];
        // outfile<<cmd_joint_position_[i]<<",";
    }
    // for (size_t i = 0; i < actual_q_.size(); i++) {
    //     outfile<<actual_q_[i]<<",";
    // }

    // outfile<<std::endl;
    // outfile.close();

    curr_write_time_ = clock_.now();
    bool is_changed = check_command_changed(prev_position_command_,aubo_position_commands_,0.001,0.02);

    // if(curr_write_time_.seconds() - prev_write_time_.seconds() > 4 * servoj_interval_time || is_changed){
    // // if(curr_write_time_.seconds() - prev_write_time_.seconds() > 0.008 || is_changed ){
    //     int servoJoint_num = rpc_client_->getRobotInterface(robot_name)
    //                          ->getMotionControl()
    //                          ->servoJoint(cmd_joint_position_, 0.2, 0.2, servoj_interval_time, 0.05, 150);
    //     // if(servoJoint_num == 0 || servoJoint_num == 2){
    //         prev_write_time_ = curr_write_time_;
    //         for(int i = 0; i < cmd_joint_position_.size(); i++){
    //             prev_position_command_[i] = cmd_joint_position_[i];
    //         }
    // //  }
    // }
    /*servoJ return:
    -13, "本次调用请求被忽略"；
    -1, "机器臂的模式状态错误"；
    -5, "传入的参数无效"；
    2, "servoJoint的轨迹队列已满"；
    0, "调用接口成功"；
    */
   int servoJoint_num = rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->servoJoint(cmd_joint_position_, 0.1, 0.1, servoj_interval_time, 0.05, 150);
//     std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return 0;
}

void AuboHardwareInterface::getToolIOType(RpcClientPtr rpc_cli)
{
    // 接口调用: 获取机器人的名字
    auto robot_name = rpc_cli->getRobotNames().front();
    auto robot = rpc_cli->getRobotInterface(robot_name);
    // 接口调用: 获取工具数字IO的数量，包含输入和输出
    tool_digital_num_ = robot->getIoControl()->getToolDigitalInputNum();
    for (int i = 0; i < tool_digital_num_; i++) {
        // 接口调用: 获取TOOL_IO[i]的类型
        bool isInput = robot->getIoControl()->isToolIoInput(i);
        if (isInput) {
            tool_is_input_[i] = true;
        }
        // std::cout << "获取TOOL_IO[" << i
        //           << "]的类型为:" << (isInput ? "输入" : "输出") << std::endl;
    }
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
        100, 0);
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

    int io_topic = cli->setTopic(
        false,
        { "R1_standard_analog_input_values", "R1_standard_analog_output_values",
          "R1_standard_digital_input_bits", "R1_standard_digital_output_bits",
          "R1_tool_digital_input_bits", "R1_tool_digital_output_bits",
          "R1_tool_analog_input_values" },
        50, 1);
    cli->subscribe(io_topic, [this](InputParser &parse) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        standard_analog_input_values_ = parse.popVectorDouble();
        standard_analog_output_values_ = parse.popVectorDouble();
        standard_digital_input_ = parse.popInt64();
        standard_digital_output_ = parse.popInt64();
        tool_digital_input_ = parse.popInt64();
        tool_digital_output_ = parse.popInt64();
        tool_analog_input_values_ = parse.popVectorDouble();
    });
}
} // namespace aubo_hardware

PLUGINLIB_EXPORT_CLASS(aubo_hardware::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
