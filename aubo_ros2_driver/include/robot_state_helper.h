#ifndef ROBOT_STATE_HELPER_H
#define ROBOT_STATE_HELPER_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace aubo_driver {

class RobotStateHelper
{
public:
    explicit RobotStateHelper(const rclcpp::Node::SharedPtr &node);
    RobotStateHelper() = delete;
    ~RobotStateHelper() = default;

private:
    rclcpp::Node::SharedPtr node_;
    std::string robot_ip_;
    std::string controller_manager_name_;
};

} // namespace aubo_driver

#endif
