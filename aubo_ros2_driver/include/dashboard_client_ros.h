#ifndef AUBO_ROS2_DRIVER_DASHBOARD_CLIENT_ROS_H
#define AUBO_ROS2_DRIVER_DASHBOARD_CLIENT_ROS_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "aubo_msgs/srv/json_rpc.hpp"

namespace aubo_driver {

class DashboardClientROS : public rclcpp::Node
{
public:
    DashboardClientROS();
    ~DashboardClientROS() override;

private:
    struct RpcResponse
    {
        bool ok{ false };
        nlohmann::json result;
        std::string error;
        std::string raw;
    };

    static bool isNoPrefixClass(const std::string &name);
    static std::vector<std::string> split(const std::string &text, char token);

    std::string buildMethodName(const std::string &cls,
                                const std::string &func) const;
    bool connectWebSocket();
    void closeWebSocket();
    void setOperationDeadline();
    RpcResponse callJsonRpc(const std::string &method,
                            const nlohmann::json &params);
    void handleJsonRpc(
        const aubo_msgs::srv::JsonRpc::Request::SharedPtr request,
        const aubo_msgs::srv::JsonRpc::Response::SharedPtr response);
    void addRobotManageService(const std::string &service_name,
                               const std::string &method_name);

    std::string robot_ip_;
    int port_{ 9012 };
    std::string robot_prefix_;
    int request_timeout_ms_{ 5000 };

    boost::asio::io_context ioc_;
    std::unique_ptr<boost::beast::websocket::stream<
        boost::beast::tcp_stream>>
        ws_;
    std::mutex rpc_mutex_;
    int request_id_{ 0 };

    rclcpp::Service<aubo_msgs::srv::JsonRpc>::SharedPtr jsonrpc_service_;
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>
        robot_manage_services_;
};

} // namespace aubo_driver

#endif // AUBO_ROS2_DRIVER_DASHBOARD_CLIENT_ROS_H
