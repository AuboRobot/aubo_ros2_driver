#include "robot_state_helper.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_state_helper");
    std::shared_ptr<aubo_driver::RobotStateHelper> robot_state_helper;

    try {
        robot_state_helper =
            std::make_shared<aubo_driver::RobotStateHelper>(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_state_helper"), "%s",
                     e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
