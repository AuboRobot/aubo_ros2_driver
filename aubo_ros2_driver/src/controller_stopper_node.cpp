#include "controller_stopper.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("controller_stopper_node");
    aubo_driver::ControllerStopper stopper(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
