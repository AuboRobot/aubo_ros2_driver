#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <aubo_msgs/msg/io_control_state.hpp>
#include <aubo_msgs/msg/robot_config_state.hpp>
#include <aubo_msgs/msg/robot_manage_state.hpp>
#include <aubo_msgs/srv/set_analog_output.hpp>
#include <aubo_msgs/srv/set_digital_output.hpp>
#include <aubo_msgs/srv/set_payload.hpp>
#include <aubo_msgs/srv/set_tcp_offset.hpp>
#include <aubo_msgs/srv/set_tool_io_config.hpp>
#include <aubo_msgs/srv/set_tool_io_input.hpp>
#include <aubo_msgs/srv/set_tool_voltage.hpp>

namespace aubo_driver {

class AuboIoStatusController : public controller_interface::ControllerInterface
{
public:
    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override
    {
        return { controller_interface::interface_configuration_type::INDIVIDUAL,
                 command_interface_names_ };
    }

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override
    {
        return { controller_interface::interface_configuration_type::ALL, {} };
    }

    controller_interface::CallbackReturn on_init() override
    {
        command_interface_names_ = fixedCommandInterfaceNames();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        io_control_state_publisher_ =
            get_node()->create_publisher<aubo_msgs::msg::IoControlState>(
                "/io_control/state", rclcpp::QoS(10));
        robot_manage_state_publisher_ =
            get_node()->create_publisher<aubo_msgs::msg::RobotManageState>(
                "/robot_manage/state", rclcpp::QoS(10));
        robot_config_state_publisher_ =
            get_node()->create_publisher<aubo_msgs::msg::RobotConfigState>(
                "/robot_config/state", rclcpp::QoS(10));

        setupServices();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        mapLoanedInterfaces();
        io_control_state_publisher_->on_activate();
        robot_manage_state_publisher_->on_activate();
        robot_config_state_publisher_->on_activate();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        io_control_state_publisher_->on_deactivate();
        robot_manage_state_publisher_->on_deactivate();
        robot_config_state_publisher_->on_deactivate();
        state_interface_index_.clear();
        command_interface_index_.clear();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override
    {
        (void)period;
        publishStates(time);
        return controller_interface::return_type::OK;
    }

private:
    static std::vector<std::string> fixedCommandInterfaceNames()
    {
        return {
            "io_control/set_digital_output_type",
            "io_control/set_digital_output_pin",
            "io_control/set_digital_output_value",
            "io_control/set_digital_output_trigger",
            "io_control/set_analog_output_type",
            "io_control/set_analog_output_pin",
            "io_control/set_analog_output_value",
            "io_control/set_analog_output_trigger",
            "io_control/set_tool_voltage_output_domain",
            "io_control/set_tool_io_input_pin",
            "io_control/set_tool_io_input_value",
            "io_control/set_tool_io_input_trigger",
            "io_control/set_tool_io_config_type",
            "io_control/set_tool_io_config_pin",
            "io_control/set_tool_io_config_value",
            "io_control/set_tool_io_config_trigger",
            "robot_config/set_payload_mass",
            "robot_config/set_payload_cog_0",
            "robot_config/set_payload_cog_1",
            "robot_config/set_payload_cog_2",
            "robot_config/set_tcp_offset_0",
            "robot_config/set_tcp_offset_1",
            "robot_config/set_tcp_offset_2",
            "robot_config/set_tcp_offset_3",
            "robot_config/set_tcp_offset_4",
            "robot_config/set_tcp_offset_5",
        };
    }

    void setupServices()
    {
        auto node = get_node();
        set_digital_output_service_ =
            node->create_service<aubo_msgs::srv::SetDigitalOutput>(
                "/io_control/set_digital_output",
                [this](
                    const std::shared_ptr<
                        aubo_msgs::srv::SetDigitalOutput::Request> request,
                    std::shared_ptr<
                        aubo_msgs::srv::SetDigitalOutput::Response> response) {
                    switch (request->output_type) {
                    case aubo_msgs::srv::SetDigitalOutput::Request::STANDARD:
                    case aubo_msgs::srv::SetDigitalOutput::Request::
                        CONFIGURABLE:
                    case aubo_msgs::srv::SetDigitalOutput::Request::TOOL:
                        break;
                    default:
                        response->success = false;
                        response->message = "unknown digital output type";
                        return;
                    }

                    response->success = setCommands(
                        { { "io_control/set_digital_output_type",
                            static_cast<double>(request->output_type) },
                          { "io_control/set_digital_output_pin",
                            static_cast<double>(request->pin) },
                          { "io_control/set_digital_output_value",
                            request->state ? 1.0 : 0.0 },
                          { "io_control/set_digital_output_trigger",
                            nextCommandTrigger() } });
                    response->message =
                        response->success ? "queued"
                                          : "command interface is not active";
                });

        set_analog_output_service_ =
            node->create_service<aubo_msgs::srv::SetAnalogOutput>(
                "/io_control/set_analog_output",
                [this](
                    const std::shared_ptr<
                        aubo_msgs::srv::SetAnalogOutput::Request> request,
                    std::shared_ptr<
                        aubo_msgs::srv::SetAnalogOutput::Response> response) {
                    switch (request->output_type) {
                    case aubo_msgs::srv::SetAnalogOutput::Request::STANDARD:
                    case aubo_msgs::srv::SetAnalogOutput::Request::TOOL:
                        break;
                    default:
                        response->success = false;
                        response->message = "unknown analog output type";
                        return;
                    }

                    response->success = setCommands(
                        { { "io_control/set_analog_output_type",
                            static_cast<double>(request->output_type) },
                          { "io_control/set_analog_output_pin",
                            static_cast<double>(request->pin) },
                          { "io_control/set_analog_output_value",
                            request->state },
                          { "io_control/set_analog_output_trigger",
                            nextCommandTrigger() } });
                    response->message =
                        response->success ? "queued"
                                          : "command interface is not active";
                });

        set_tool_voltage_service_ =
            node->create_service<aubo_msgs::srv::SetToolVoltage>(
                "/io_control/set_tool_voltage",
                [this](
                    const std::shared_ptr<
                        aubo_msgs::srv::SetToolVoltage::Request> request,
                    std::shared_ptr<
                        aubo_msgs::srv::SetToolVoltage::Response> response) {
                    response->success = setCommand(
                        "io_control/set_tool_voltage_output_domain",
                        static_cast<double>(request->domain));
                    response->message =
                        response->success ? "ok"
                                          : "command interface is not active";
                });

        set_tool_io_input_service_ =
            node->create_service<aubo_msgs::srv::SetToolIoInput>(
                "/io_control/set_tool_io_input",
                [this](
                    const std::shared_ptr<
                        aubo_msgs::srv::SetToolIoInput::Request> request,
                    std::shared_ptr<
                        aubo_msgs::srv::SetToolIoInput::Response> response) {
                    response->success = setCommands(
                        { { "io_control/set_tool_io_input_pin",
                            static_cast<double>(request->pin) },
                          { "io_control/set_tool_io_input_value",
                            request->input ? 1.0 : 0.0 },
                          { "io_control/set_tool_io_input_trigger",
                            nextCommandTrigger() } });
                    response->message =
                        response->success ? "queued"
                                          : "command interface is not active";
                });

        set_tool_io_config_service_ =
            node->create_service<aubo_msgs::srv::SetToolIoConfig>(
                "/io_control/set_tool_io_config",
                [this](
                    const std::shared_ptr<
                        aubo_msgs::srv::SetToolIoConfig::Request> request,
                    std::shared_ptr<
                        aubo_msgs::srv::SetToolIoConfig::Response> response) {
                    switch (request->config_type) {
                    case aubo_msgs::srv::SetToolIoConfig::Request::
                        DIGITAL_INPUT_ACTION:
                    case aubo_msgs::srv::SetToolIoConfig::Request::
                        DIGITAL_OUTPUT_RUNSTATE:
                    case aubo_msgs::srv::SetToolIoConfig::Request::
                        ANALOG_INPUT_DOMAIN:
                    case aubo_msgs::srv::SetToolIoConfig::Request::
                        ANALOG_OUTPUT_DOMAIN:
                    case aubo_msgs::srv::SetToolIoConfig::Request::
                        ANALOG_OUTPUT_RUNSTATE:
                        break;
                    default:
                        response->success = false;
                        response->message = "unknown tool io config type";
                        return;
                    }

                    response->success = setCommands(
                        { { "io_control/set_tool_io_config_type",
                            static_cast<double>(request->config_type) },
                          { "io_control/set_tool_io_config_pin",
                            static_cast<double>(request->pin) },
                          { "io_control/set_tool_io_config_value",
                            static_cast<double>(request->value) },
                          { "io_control/set_tool_io_config_trigger",
                            nextCommandTrigger() } });
                    response->message =
                        response->success ? "queued"
                                          : "command interface is not active";
                });

        set_payload_service_ =
            node->create_service<aubo_msgs::srv::SetPayload>(
                "/robot_config/set_payload",
                [this](
                    const std::shared_ptr<
                        aubo_msgs::srv::SetPayload::Request> request,
                    std::shared_ptr<
                        aubo_msgs::srv::SetPayload::Response> response) {
                    response->success =
                        setCommand("robot_config/set_payload_mass",
                                   static_cast<double>(request->mass)) &&
                        setIndexedCommand("robot_config/set_payload_cog", 0,
                                          request->center_of_gravity.x) &&
                        setIndexedCommand("robot_config/set_payload_cog", 1,
                                          request->center_of_gravity.y) &&
                        setIndexedCommand("robot_config/set_payload_cog", 2,
                                          request->center_of_gravity.z);
                });

        set_tcp_offset_service_ =
            node->create_service<aubo_msgs::srv::SetTcpOffset>(
                "/robot_config/set_tcp_offset",
                [this](
                    const std::shared_ptr<
                        aubo_msgs::srv::SetTcpOffset::Request> request,
                    std::shared_ptr<
                        aubo_msgs::srv::SetTcpOffset::Response> response) {
                    if (request->tcp_offset.size() != 6) {
                        response->success = false;
                        response->message = "tcp_offset must contain 6 values";
                        return;
                    }

                    response->success = true;
                    for (std::size_t i = 0; i < request->tcp_offset.size();
                         ++i) {
                        response->success =
                            response->success &&
                            setIndexedCommand("robot_config/set_tcp_offset", i,
                                              request->tcp_offset[i]);
                    }
                    response->message =
                        response->success ? "ok"
                                          : "command interface is not active";
                });
    }

    void mapLoanedInterfaces()
    {
        state_interface_index_.clear();
        command_interface_index_.clear();
        for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
            state_interface_index_[state_interfaces_[i].get_name()] = i;
        }
        for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
            command_interface_index_[command_interfaces_[i].get_name()] = i;
        }
    }

    void publishStates(const rclcpp::Time &time)
    {
        aubo_msgs::msg::IoControlState io_msg;
        io_msg.stamp = time;
        appendBoolStates(io_msg.standard_digital_inputs,
                         "io_control/get_standard_digital_input");
        appendBoolStates(io_msg.standard_digital_outputs,
                         "io_control/get_standard_digital_output");
        appendBoolStates(io_msg.configurable_digital_inputs,
                         "io_control/get_configurable_digital_input");
        appendBoolStates(io_msg.configurable_digital_outputs,
                         "io_control/get_configurable_digital_output");
        appendBoolStates(io_msg.tool_digital_inputs,
                         "io_control/get_tool_digital_input");
        appendBoolStates(io_msg.tool_digital_outputs,
                         "io_control/get_tool_digital_output");
        appendDoubleStates(io_msg.standard_analog_inputs,
                           "io_control/get_standard_analog_input");
        appendDoubleStates(io_msg.standard_analog_outputs,
                           "io_control/get_standard_analog_output");
        appendDoubleStates(io_msg.tool_analog_inputs,
                           "io_control/get_tool_analog_input");
        appendDoubleStates(io_msg.tool_analog_outputs,
                           "io_control/get_tool_analog_output");
        appendBoolStates(io_msg.tool_io_inputs,
                         "io_control/get_tool_io_input");
        appendIntStates(io_msg.tool_digital_input_actions,
                        "io_control/get_tool_digital_input_action");
        appendIntStates(io_msg.tool_digital_output_runstates,
                        "io_control/get_tool_digital_output_runstate");
        appendIntStates(io_msg.tool_analog_input_domains,
                        "io_control/get_tool_analog_input_domain");
        appendIntStates(io_msg.tool_analog_output_domains,
                        "io_control/get_tool_analog_output_domain");
        appendIntStates(io_msg.tool_analog_output_runstates,
                        "io_control/get_tool_analog_output_runstate");
        io_msg.tool_voltage_output_domain = static_cast<std::int32_t>(
            std::lround(stateValue(
                "io_control/get_tool_voltage_output_domain")));
        io_msg.tool_button_status =
            stateValue("io_control/get_tool_button_status") != 0.0;
        io_msg.handle_status = static_cast<std::uint64_t>(
            std::llround(stateValue("io_control/get_handle_status")));
        io_msg.handle_device_state = static_cast<std::int32_t>(
            std::lround(stateValue("io_control/get_handle_state")));
        io_control_state_publisher_->publish(io_msg);

        aubo_msgs::msg::RobotManageState robot_manage_msg;
        robot_manage_msg.stamp = time;
        robot_manage_msg.handguide_enabled =
            stateValue("robot_manage/get_handguide_enabled") != 0.0;
        appendIntStates(robot_manage_msg.handguide_free_axes,
                        "robot_manage/get_handguide_free_axis");
        appendDoubleStates(robot_manage_msg.handguide_feature,
                           "robot_manage/get_handguide_feature");
        robot_manage_state_publisher_->publish(robot_manage_msg);

        aubo_msgs::msg::RobotConfigState robot_config_msg;
        robot_config_msg.stamp = time;
        appendDoubleStates(robot_config_msg.tcp_offset,
                           "robot_config/get_tcp_offset");
        robot_config_msg.payload_mass =
            stateValue("robot_config/get_payload_mass");
        robot_config_msg.payload_cog.x =
            stateValue("robot_config/get_payload_cog_0");
        robot_config_msg.payload_cog.y =
            stateValue("robot_config/get_payload_cog_1");
        robot_config_msg.payload_cog.z =
            stateValue("robot_config/get_payload_cog_2");
        robot_config_state_publisher_->publish(robot_config_msg);
    }

    template <typename MessageVector>
    void appendBoolStates(MessageVector &target, const std::string &prefix) const
    {
        target.clear();
        const std::size_t count = indexedStateCount(prefix);
        target.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            target.push_back(
                stateValue(prefix + "_" + std::to_string(i)) != 0.0);
        }
    }

    template <typename MessageVector>
    void appendIntStates(MessageVector &target, const std::string &prefix) const
    {
        target.clear();
        const std::size_t count = indexedStateCount(prefix);
        target.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            target.push_back(static_cast<std::int32_t>(
                std::lround(stateValue(prefix + "_" + std::to_string(i)))));
        }
    }

    template <typename MessageVector>
    void appendDoubleStates(MessageVector &target,
                            const std::string &prefix) const
    {
        target.clear();
        const std::size_t count = indexedStateCount(prefix);
        target.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            target.push_back(stateValue(prefix + "_" + std::to_string(i)));
        }
    }

    std::size_t indexedStateCount(const std::string &prefix) const
    {
        std::size_t count = 0;
        while (state_interface_index_.find(prefix + "_" +
                                           std::to_string(count)) !=
               state_interface_index_.end()) {
            ++count;
        }
        return count;
    }

    double stateValue(const std::string &name, double default_value = 0.0) const
    {
        const auto it = state_interface_index_.find(name);
        if (it == state_interface_index_.end()) {
            return default_value;
        }
        return state_interfaces_[it->second].get_value();
    }

    bool setCommand(const std::string &name, double value)
    {
        const auto it = command_interface_index_.find(name);
        if (it == command_interface_index_.end()) {
            return false;
        }
        command_interfaces_[it->second].set_value(value);
        return true;
    }

    bool setCommands(
        const std::vector<std::pair<std::string, double>> &commands)
    {
        for (const auto &command : commands) {
            if (command_interface_index_.find(command.first) ==
                command_interface_index_.end()) {
                return false;
            }
        }

        for (const auto &command : commands) {
            command_interfaces_[command_interface_index_[command.first]]
                .set_value(command.second);
        }
        return true;
    }

    bool setIndexedCommand(const std::string &prefix, std::size_t index,
                           double value)
    {
        return setCommand(prefix + "_" + std::to_string(index), value);
    }

    double nextCommandTrigger()
    {
        command_trigger_ += 1.0;
        return command_trigger_;
    }

    std::vector<std::string> command_interface_names_;
    std::unordered_map<std::string, std::size_t> state_interface_index_;
    std::unordered_map<std::string, std::size_t> command_interface_index_;
    double command_trigger_{ 0.0 };

    rclcpp_lifecycle::LifecyclePublisher<
        aubo_msgs::msg::IoControlState>::SharedPtr io_control_state_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<
        aubo_msgs::msg::RobotManageState>::SharedPtr
        robot_manage_state_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<
        aubo_msgs::msg::RobotConfigState>::SharedPtr
        robot_config_state_publisher_;

    rclcpp::Service<aubo_msgs::srv::SetDigitalOutput>::SharedPtr
        set_digital_output_service_;
    rclcpp::Service<aubo_msgs::srv::SetAnalogOutput>::SharedPtr
        set_analog_output_service_;
    rclcpp::Service<aubo_msgs::srv::SetToolVoltage>::SharedPtr
        set_tool_voltage_service_;
    rclcpp::Service<aubo_msgs::srv::SetToolIoInput>::SharedPtr
        set_tool_io_input_service_;
    rclcpp::Service<aubo_msgs::srv::SetToolIoConfig>::SharedPtr
        set_tool_io_config_service_;
    rclcpp::Service<aubo_msgs::srv::SetPayload>::SharedPtr
        set_payload_service_;
    rclcpp::Service<aubo_msgs::srv::SetTcpOffset>::SharedPtr
        set_tcp_offset_service_;
};

} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboIoStatusController,
                       controller_interface::ControllerInterface)
