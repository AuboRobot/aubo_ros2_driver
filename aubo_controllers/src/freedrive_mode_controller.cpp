#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "controller_interface/controller_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/bool.hpp"

#include <aubo_msgs/srv/set_handguide.hpp>

namespace aubo_driver {

class FreedriveModeController : public controller_interface::ControllerInterface
{
public:
    controller_interface::CallbackReturn on_init() override
    {
        command_interface_names_ = defaultCommandInterfaceNames();
        state_interface_names_ = defaultStateInterfaceNames();
        auto_declare<std::vector<int64_t>>("free_axes", { 1, 1, 1, 1, 1 });
        auto_declare<std::vector<double>>("feature",
                                          { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override
    {
        return { controller_interface::interface_configuration_type::INDIVIDUAL,
                 command_interface_names_ };
    }

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override
    {
        return { controller_interface::interface_configuration_type::INDIVIDUAL,
                 state_interface_names_ };
    }

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        if (!readDefaultHandguideParams()) {
            return controller_interface::CallbackReturn::ERROR;
        }
        requested_free_axes_ = default_free_axes_;
        requested_feature_ = default_feature_;

        enable_freedrive_mode_sub_ =
            get_node()->create_subscription<std_msgs::msg::Bool>(
                "~/enable_freedrive_mode", 10,
                [this](const std_msgs::msg::Bool::SharedPtr message) {
                    std::string result;
                    if (!requestFreedriveMode(message->data, result)) {
                        RCLCPP_ERROR(get_node()->get_logger(), "%s",
                                     result.c_str());
                    }
                });

        set_handguide_service_ =
            get_node()->create_service<aubo_msgs::srv::SetHandguide>(
                "/robot_manage/set_handguide",
                [this](
                    const std::shared_ptr<
                        aubo_msgs::srv::SetHandguide::Request> request,
                    std::shared_ptr<
                        aubo_msgs::srv::SetHandguide::Response> response) {
                    if (!applyServiceRequest(*request, response->message)) {
                        response->success = false;
                        return;
                    }
                    response->success =
                        requestFreedriveMode(request->enable,
                                             response->message);
                });

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        mapLoanedInterfaces();
        command_pending_ = false;
        requested_enable_ = isHandguideEnabled();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        command_interface_index_.clear();
        state_interface_index_.clear();
        command_pending_ = false;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override
    {
        (void)time;
        (void)period;
        if (!command_pending_) {
            return controller_interface::return_type::OK;
        }

        bool success = true;
        for (std::size_t i = 0; i < requested_free_axes_.size(); ++i) {
            success =
                setIndexedCommand("robot_manage/set_handguide_free_axis", i,
                                  requested_free_axes_[i]) &&
                success;
        }
        for (std::size_t i = 0; i < requested_feature_.size(); ++i) {
            success = setIndexedCommand("robot_manage/set_handguide_feature", i,
                                        requested_feature_[i]) &&
                      success;
        }
        success =
            setCommand("robot_manage/set_handguide_enable",
                       requested_enable_ ? 1.0 : 0.0) &&
            success;
        command_pending_ = false;

        if (!success) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Failed to write handguide command interfaces");
            return controller_interface::return_type::ERROR;
        }
        return controller_interface::return_type::OK;
    }

private:
    static void appendIndexedInterfaces(std::vector<std::string> &names,
                                        const std::string &prefix,
                                        std::size_t count)
    {
        for (std::size_t i = 0; i < count; ++i) {
            names.push_back(prefix + "_" + std::to_string(i));
        }
    }

    static std::vector<std::string> defaultCommandInterfaceNames()
    {
        std::vector<std::string> names;
        names.push_back("robot_manage/set_handguide_enable");
        appendIndexedInterfaces(names, "robot_manage/set_handguide_free_axis",
                                5);
        appendIndexedInterfaces(names, "robot_manage/set_handguide_feature", 6);
        return names;
    }

    static std::vector<std::string> defaultStateInterfaceNames()
    {
        std::vector<std::string> names;
        names.push_back("robot_manage/get_handguide_enabled");
        appendIndexedInterfaces(names, "robot_manage/get_handguide_free_axis",
                                5);
        appendIndexedInterfaces(names, "robot_manage/get_handguide_feature", 6);
        return names;
    }

    bool readDefaultHandguideParams()
    {
        const auto free_axes =
            get_node()->get_parameter("free_axes").as_integer_array();
        if (free_axes.size() != 5) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "free_axes parameter must contain 5 values");
            return false;
        }
        default_free_axes_.clear();
        default_free_axes_.reserve(free_axes.size());
        for (const auto axis : free_axes) {
            default_free_axes_.push_back(static_cast<double>(axis));
        }

        default_feature_ = get_node()->get_parameter("feature").as_double_array();
        if (default_feature_.size() != 6) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "feature parameter must contain 6 values");
            return false;
        }
        return true;
    }

    bool applyServiceRequest(
        const aubo_msgs::srv::SetHandguide::Request &request,
        std::string &message)
    {
        if (!request.free_axes.empty() && request.free_axes.size() != 5) {
            message = "free_axes must contain 5 values";
            return false;
        }
        if (!request.feature.empty() && request.feature.size() != 6) {
            message = "feature must contain 6 values";
            return false;
        }

        requested_free_axes_ =
            request.free_axes.empty() ? currentFreeAxes() : default_free_axes_;
        for (std::size_t i = 0; i < request.free_axes.size(); ++i) {
            requested_free_axes_[i] = static_cast<double>(request.free_axes[i]);
        }

        requested_feature_ =
            request.feature.empty() ? currentFeature() : request.feature;
        return true;
    }

    bool requestFreedriveMode(bool enable, std::string &message)
    {
        if (get_node()->get_current_state().id() !=
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            message = "freedrive_mode_controller is not active";
            return false;
        }

        requested_enable_ = enable;
        command_pending_ = true;
        message = "ok";
        return true;
    }

    void mapLoanedInterfaces()
    {
        command_interface_index_.clear();
        state_interface_index_.clear();
        for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
            command_interface_index_[command_interfaces_[i].get_name()] = i;
        }
        for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
            state_interface_index_[state_interfaces_[i].get_name()] = i;
        }
    }

    double stateValue(const std::string &name, double default_value = 0.0) const
    {
        const auto it = state_interface_index_.find(name);
        if (it == state_interface_index_.end()) {
            return default_value;
        }
        return state_interfaces_[it->second].get_value();
    }

    bool isHandguideEnabled() const
    {
        return stateValue("robot_manage/get_handguide_enabled") != 0.0;
    }

    std::vector<double> currentFreeAxes() const
    {
        std::vector<double> values;
        values.reserve(5);
        for (std::size_t i = 0; i < 5; ++i) {
            values.push_back(stateValue(
                "robot_manage/get_handguide_free_axis_" + std::to_string(i),
                default_free_axes_[i]));
        }
        return values;
    }

    std::vector<double> currentFeature() const
    {
        std::vector<double> values;
        values.reserve(6);
        for (std::size_t i = 0; i < 6; ++i) {
            values.push_back(stateValue(
                "robot_manage/get_handguide_feature_" + std::to_string(i),
                default_feature_[i]));
        }
        return values;
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

    bool setIndexedCommand(const std::string &prefix, std::size_t index,
                           double value)
    {
        return setCommand(prefix + "_" + std::to_string(index), value);
    }

    std::vector<std::string> command_interface_names_;
    std::vector<std::string> state_interface_names_;
    std::unordered_map<std::string, std::size_t> command_interface_index_;
    std::unordered_map<std::string, std::size_t> state_interface_index_;

    bool command_pending_{ false };
    bool requested_enable_{ false };
    std::vector<double> default_free_axes_{ 1.0, 1.0, 1.0, 1.0, 1.0 };
    std::vector<double> default_feature_{ 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0 };
    std::vector<double> requested_free_axes_{ 1.0, 1.0, 1.0, 1.0, 1.0 };
    std::vector<double> requested_feature_{ 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0 };

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        enable_freedrive_mode_sub_;
    rclcpp::Service<aubo_msgs::srv::SetHandguide>::SharedPtr
        set_handguide_service_;
};

} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::FreedriveModeController,
                       controller_interface::ControllerInterface)
