#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dashboard_client_ros.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aubo_driver::DashboardClientROS>());
    rclcpp::shutdown();
    return 0;
}
