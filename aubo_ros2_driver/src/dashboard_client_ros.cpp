#include "dashboard_client_ros.h"

#include <algorithm>

#include <boost/asio/connect.hpp>
#include <boost/beast/core.hpp>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using json = nlohmann::json;

namespace aubo_driver {

DashboardClientROS::DashboardClientROS() : Node("dashboard_client")
{
    robot_ip_ = declare_parameter<std::string>("robot_ip", "127.0.0.1");
    port_ = declare_parameter<int>("port", 9012);
    robot_prefix_ = declare_parameter<std::string>("robot", "rob1");
    request_timeout_ms_ = declare_parameter<int>("request_timeout_ms", 5000);

    jsonrpc_service_ = create_service<aubo_msgs::srv::JsonRpc>(
        "/jsonrpc_service",
        std::bind(&DashboardClientROS::handleJsonRpc, this,
                  std::placeholders::_1, std::placeholders::_2));

    addRobotManageService("/robot_manage/poweron", "poweron");
    addRobotManageService("/robot_manage/startup", "startup");
    addRobotManageService("/robot_manage/poweroff", "poweroff");
    addRobotManageService("/robot_manage/release_robot_brake",
                          "releaseRobotBrake");
    addRobotManageService("/robot_manage/lock_robot_brake", "lockRobotBrake");
    addRobotManageService("/robot_manage/unlock_protective_stop",
                          "setUnlockProtectiveStop");

    RCLCPP_INFO(get_logger(),
                "Dashboard client ready for ws://%s:%d robot=\"%s\"",
                robot_ip_.c_str(), port_, robot_prefix_.c_str());
}

DashboardClientROS::~DashboardClientROS() { closeWebSocket(); }

bool DashboardClientROS::isNoPrefixClass(const std::string &name)
{
    static const std::vector<std::string> no_prefix_classes = {
        "AuboApi",        "Math",        "RegisterControl",
        "RobotInterface", "RuntimeMachine", "Serial",
        "Socket",         "SyncMove",    "SystemInfo",
    };

    return std::find(no_prefix_classes.begin(), no_prefix_classes.end(),
                     name) != no_prefix_classes.end();
}

std::vector<std::string> DashboardClientROS::split(const std::string &text,
                                                   char token)
{
    std::vector<std::string> parts;
    std::string current;
    for (char value : text) {
        if (value == token) {
            parts.push_back(current);
            current.clear();
            continue;
        }
        current.push_back(value);
    }
    parts.push_back(current);
    return parts;
}

std::string DashboardClientROS::buildMethodName(const std::string &cls,
                                                const std::string &func) const
{
    if (func.find('.') != std::string::npos) {
        const auto parts = split(func, '.');
        if (!parts.empty() &&
            (parts.front() == robot_prefix_ || isNoPrefixClass(parts.front()))) {
            return func;
        }

        if (parts.size() == 2) {
            if (isNoPrefixClass(parts[0])) {
                return func;
            }
            return robot_prefix_ + "." + func;
        }

        return func;
    }

    if (!cls.empty()) {
        if (isNoPrefixClass(cls)) {
            return cls + "." + func;
        }
        return robot_prefix_ + "." + cls + "." + func;
    }

    return robot_prefix_ + "." + func;
}

bool DashboardClientROS::connectWebSocket()
{
    closeWebSocket();
    ioc_.restart();

    try {
        tcp::resolver resolver(ioc_);
        const auto port = std::to_string(port_);
        const auto results = resolver.resolve(robot_ip_, port);

        ws_ = std::make_unique<websocket::stream<beast::tcp_stream>>(ioc_);
        beast::get_lowest_layer(*ws_).expires_after(
            std::chrono::milliseconds(request_timeout_ms_));
        beast::get_lowest_layer(*ws_).connect(results);
        ws_->set_option(websocket::stream_base::timeout::suggested(
            beast::role_type::client));
        ws_->handshake(robot_ip_ + ":" + port, "/");

        RCLCPP_INFO(get_logger(), "Connected to WebSocket ws://%s:%d",
                    robot_ip_.c_str(), port_);
        return true;
    } catch (const std::exception &e) {
        ws_.reset();
        RCLCPP_ERROR(get_logger(), "Failed to connect WebSocket: %s",
                     e.what());
        return false;
    }
}

void DashboardClientROS::closeWebSocket()
{
    if (!ws_) {
        return;
    }

    beast::error_code error_code;
    if (ws_->is_open() && ws_->next_layer().socket().is_open()) {
        ws_->next_layer().socket().shutdown(tcp::socket::shutdown_both,
                                            error_code);
        error_code.clear();
        ws_->next_layer().socket().close(error_code);
    }
    ws_.reset();
}

void DashboardClientROS::setOperationDeadline()
{
    if (!ws_) {
        return;
    }

    beast::get_lowest_layer(*ws_).expires_after(
        std::chrono::milliseconds(request_timeout_ms_));
}

DashboardClientROS::RpcResponse
DashboardClientROS::callJsonRpc(const std::string &method, const json &params)
{
    std::lock_guard<std::mutex> lock(rpc_mutex_);

    if (!ws_ || !ws_->is_open()) {
        RCLCPP_WARN(get_logger(), "WebSocket is not connected, reconnecting...");
        if (!connectWebSocket()) {
            return { false, {}, "failed to connect WebSocket", {} };
        }
    }

    const json request = {
        { "jsonrpc", "2.0" },
        { "method", method },
        { "params", params },
        { "id", ++request_id_ },
    };

    try {
        const auto request_text = request.dump();
        setOperationDeadline();
        ws_->write(net::buffer(request_text));

        beast::flat_buffer buffer;
        setOperationDeadline();
        ws_->read(buffer);
        const auto raw = beast::buffers_to_string(buffer.data());
        const auto response = json::parse(raw);

        if (response.contains("error") && !response["error"].is_null()) {
            return { false, {}, response["error"].dump(), raw };
        }

        return { true, response.value("result", json()), {}, raw };
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "WebSocket communication failed: %s",
                     e.what());
        closeWebSocket();
        return { false, {}, e.what(), {} };
    }
}

void DashboardClientROS::handleJsonRpc(
    const aubo_msgs::srv::JsonRpc::Request::SharedPtr request,
    const aubo_msgs::srv::JsonRpc::Response::SharedPtr response)
{
    json params = json::array();
    if (!request->params.empty()) {
        try {
            params = json::parse(request->params);
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(),
                        "Failed to parse JSON-RPC params, using []: %s",
                        e.what());
        }
    }

    const auto method = buildMethodName(request->cls, request->func);
    const auto result = callJsonRpc(method, params);

    if (!result.ok) {
        response->result = "None";
        response->error = result.error;
        return;
    }

    response->result = result.result.dump();
    response->error = "None";
}

void DashboardClientROS::addRobotManageService(
    const std::string &service_name, const std::string &method_name)
{
    robot_manage_services_.push_back(create_service<std_srvs::srv::Trigger>(
        service_name,
        [this, method_name](
            const std_srvs::srv::Trigger::Request::SharedPtr,
            const std_srvs::srv::Trigger::Response::SharedPtr response) {
            const auto result =
                callJsonRpc(robot_prefix_ + ".RobotManage." + method_name,
                            json::array());

            if (!result.ok) {
                response->success = false;
                response->message = result.error;
                return;
            }

            const bool sdk_success =
                !result.result.is_number_integer() || result.result.get<int>() == 0;
            response->success = sdk_success;
            response->message = "RobotManage." + method_name + " returned " +
                                result.result.dump();
        }));
}

} // namespace aubo_driver
