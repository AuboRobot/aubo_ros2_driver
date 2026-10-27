#include <aubo_sdk/rpc.h>
#include <aubo_msgs/msg/aubo_command_status.hpp>
#include <aubo_msgs/msg/aubo_joint_move.hpp>
#include <aubo_msgs/msg/aubo_linear_move.hpp>
#include <aubo_msgs/msg/aubo_power_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using arcs::common_interface::RobotInterfacePtr;
using arcs::aubo_sdk::RpcClient;
using aubo_msgs::msg::AuboCommandStatus;
using aubo_msgs::msg::AuboJointMove;
using aubo_msgs::msg::AuboLinearMove;
using aubo_msgs::msg::AuboPowerCommand;

bool isFinitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

std::array<double, 6> poseToSdkVector(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm < 1e-12) {
    throw std::invalid_argument("target orientation must be a valid quaternion");
  }

  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  const double angle = 2.0 * std::acos(std::clamp(w, -1.0, 1.0));
  const double sin_half_angle = std::sqrt(std::max(0.0, 1.0 - w * w));

  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  if (sin_half_angle > 1e-12) {
    rx = angle * x / sin_half_angle;
    ry = angle * y / sin_half_angle;
    rz = angle * z / sin_half_angle;
  }

  const auto & p = pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    throw std::invalid_argument("target position must contain finite values");
  }
  return {p.x, p.y, p.z, rx, ry, rz};
}

class AuboCommandNode : public rclcpp::Node
{
public:
  AuboCommandNode()
  : Node("aubo_command_node")
  {
    const auto robot_ip = declare_parameter<std::string>("robot_ip", "");
    const auto robot_port = declare_parameter<int>("robot_port", 30004);
    const auto username = declare_parameter<std::string>("username", "aubo");
    const auto password = declare_parameter<std::string>("password", "123456");
    const auto requested_robot_name = declare_parameter<std::string>("robot_name", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    mode_timeout_ = std::chrono::seconds(
      declare_parameter<int>("mode_timeout_seconds", 60));
    motion_timeout_ = std::chrono::seconds(
      declare_parameter<int>("motion_timeout_seconds", 180));

    if (robot_ip.empty()) {
      throw std::invalid_argument("parameter 'robot_ip' must be set");
    }
    if (robot_port <= 0 || robot_port > 65535) {
      throw std::invalid_argument("parameter 'robot_port' must be between 1 and 65535");
    }
    if (base_frame_.empty()) {
      throw std::invalid_argument("parameter 'base_frame' must not be empty");
    }
    if (mode_timeout_.count() <= 0 || motion_timeout_.count() <= 0) {
      throw std::invalid_argument("command timeouts must be positive");
    }

    rpc_client_ = std::make_shared<RpcClient>();
    rpc_client_->setRequestTimeout(1000);
    if (rpc_client_->connect(robot_ip, robot_port) != 0) {
      throw std::runtime_error("failed to connect to the robot RPC server");
    }
    if (rpc_client_->login(username, password) != 0) {
      throw std::runtime_error("failed to log in to the robot RPC server");
    }

    const auto robot_names = rpc_client_->getRobotNames();
    if (robot_names.empty()) {
      throw std::runtime_error("the robot RPC server returned no robot names");
    }
    robot_name_ = requested_robot_name.empty() ? robot_names.front() : requested_robot_name;
    if (std::find(robot_names.begin(), robot_names.end(), robot_name_) == robot_names.end()) {
      throw std::runtime_error("configured robot_name was not returned by the robot");
    }
    robot_interface_ = rpc_client_->getRobotInterface(robot_name_);
    if (!robot_interface_) {
      throw std::runtime_error("failed to acquire the robot interface");
    }

    status_publisher_ = create_publisher<AuboCommandStatus>(
      "/aubo/command_status", rclcpp::QoS(10).reliable());
    power_subscription_ = create_subscription<AuboPowerCommand>(
      "/aubo/power_command", rclcpp::QoS(10).reliable(),
      std::bind(&AuboCommandNode::onPowerCommand, this, std::placeholders::_1));
    joint_subscription_ = create_subscription<AuboJointMove>(
      "/aubo/joint_move", rclcpp::QoS(10).reliable(),
      std::bind(&AuboCommandNode::onJointMove, this, std::placeholders::_1));
    linear_subscription_ = create_subscription<AuboLinearMove>(
      "/aubo/linear_move", rclcpp::QoS(10).reliable(),
      std::bind(&AuboCommandNode::onLinearMove, this, std::placeholders::_1));

    worker_ = std::thread(&AuboCommandNode::commandWorker, this);
    RCLCPP_INFO(
      get_logger(), "Connected to robot '%s'; command topics are ready",
      robot_name_.c_str());
  }

  ~AuboCommandNode() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stopping_ = true;
    }
    queue_condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (rpc_client_) {
      rpc_client_->logout();
      rpc_client_->disconnect();
    }
  }

private:
  struct Command
  {
    std::string id;
    std::string type;
    std::function<void()> execute;
  };

  void publishStatus(
    const std::string & id, const std::string & type, const std::string & status,
    const std::string & message)
  {
    AuboCommandStatus result;
    result.command_id = id;
    result.command_type = type;
    result.status = status;
    result.message = message;
    status_publisher_->publish(result);
  }

  bool submit(const std::string & id, const std::string & type, std::function<void()> execute)
  {
    if (id.empty()) {
      publishStatus(id, type, "rejected", "command_id must not be empty");
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (stopping_) {
        publishStatus(id, type, "rejected", "command node is shutting down");
        return false;
      }
      if (busy_) {
        publishStatus(id, type, "rejected", "another command is executing");
        return false;
      }
      busy_ = true;
      command_queue_.push(Command{id, type, std::move(execute)});
      publishStatus(id, type, "accepted", "command accepted");
    }

    queue_condition_.notify_one();
    return true;
  }

  void onPowerCommand(const AuboPowerCommand::SharedPtr message)
  {
    const auto type = message->power_on ? "power_on" : "power_off";
    submit(message->command_id, type, [this, message]() { executePowerCommand(*message); });
  }

  void onJointMove(const AuboJointMove::SharedPtr message)
  {
    if (!isFinitePositive(message->max_velocity) ||
      !isFinitePositive(message->max_acceleration))
    {
      publishStatus(
        message->command_id, "joint_move", "rejected",
        "max_velocity and max_acceleration must be finite and positive");
      return;
    }
    for (const auto position : message->positions) {
      if (!std::isfinite(position)) {
        publishStatus(
          message->command_id, "joint_move", "rejected",
          "all joint positions must be finite");
        return;
      }
    }

    submit(
      message->command_id, "joint_move",
      [this, message]() { executeJointMove(*message); });
  }

  void onLinearMove(const AuboLinearMove::SharedPtr message)
  {
    if (message->target.header.frame_id != base_frame_) {
      publishStatus(
        message->command_id, "linear_move", "rejected",
        "target frame must match configured base_frame '" + base_frame_ + "'");
      return;
    }
    if (!isFinitePositive(message->speed) ||
      !isFinitePositive(message->acceleration) ||
      !std::isfinite(message->blend_radius) || message->blend_radius < 0.0)
    {
      publishStatus(
        message->command_id, "linear_move", "rejected",
        "speed and acceleration must be positive; blend_radius must be non-negative");
      return;
    }
    try {
      poseToSdkVector(message->target.pose);
    } catch (const std::exception & error) {
      publishStatus(message->command_id, "linear_move", "rejected", error.what());
      return;
    }

    submit(
      message->command_id, "linear_move",
      [this, message]() { executeLinearMove(*message); });
  }

  void commandWorker()
  {
    while (true) {
      Command command;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this]() { return stopping_ || !command_queue_.empty(); });
        if (stopping_ && command_queue_.empty()) {
          return;
        }
        command = std::move(command_queue_.front());
        command_queue_.pop();
      }

      publishStatus(command.id, command.type, "running", "command execution started");
      try {
        command.execute();
        publishStatus(command.id, command.type, "succeeded", "command completed");
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Command '%s' failed: %s",
          command.id.c_str(), error.what());
        publishStatus(command.id, command.type, "failed", error.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Command '%s' failed with an unknown error", command.id.c_str());
        publishStatus(command.id, command.type, "failed", "unknown SDK error");
      }

      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        busy_ = false;
      }
    }
  }

  bool waitForMode(
    const arcs::common_interface::RobotModeType target_mode,
    const std::chrono::seconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (robot_interface_->getRobotState()->getRobotModeType() == target_mode) {
        return true;
      }
      std::this_thread::sleep_for(200ms);
    }
    return robot_interface_->getRobotState()->getRobotModeType() == target_mode;
  }

  void executePowerCommand(const AuboPowerCommand & command)
  {
    auto manage = robot_interface_->getRobotManage();
    auto state = robot_interface_->getRobotState();
    if (command.power_on) {
      auto mode = state->getRobotModeType();
      if (mode == arcs::common_interface::RobotModeType::Running) {
        return;
      }
      if (mode != arcs::common_interface::RobotModeType::Idle) {
        if (manage->poweron() != 0 ||
          !waitForMode(arcs::common_interface::RobotModeType::Idle, mode_timeout_))
        {
          throw std::runtime_error("robot did not reach Idle mode after power-on request");
        }
      }
      if (manage->startup() != 0 ||
        !waitForMode(arcs::common_interface::RobotModeType::Running, mode_timeout_))
      {
        throw std::runtime_error("robot did not reach Running mode after startup request");
      }
      return;
    }

    if (state->getRobotModeType() == arcs::common_interface::RobotModeType::PowerOff) {
      return;
    }
    if (manage->poweroff() != 0 ||
      !waitForMode(arcs::common_interface::RobotModeType::PowerOff, mode_timeout_))
    {
      throw std::runtime_error("robot did not reach PowerOff mode after power-off request");
    }
  }

  void ensureRobotRunning() const
  {
    if (robot_interface_->getRobotState()->getRobotModeType() !=
      arcs::common_interface::RobotModeType::Running)
    {
      throw std::runtime_error("robot must be in Running mode before motion");
    }
  }

  void waitForMotionCompletion()
  {
    const auto deadline = std::chrono::steady_clock::now() + motion_timeout_;
    bool motion_started = false;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto exec_id = robot_interface_->getMotionControl()->getExecId();
      if (exec_id != -1) {
        motion_started = true;
      } else if (motion_started) {
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
    throw std::runtime_error("timed out waiting for robot motion to complete");
  }

  void executeJointMove(const AuboJointMove & command)
  {
    ensureRobotRunning();
    const std::vector<double> positions(
      command.positions.begin(), command.positions.end());
    const int result = robot_interface_->getMotionControl()->moveJoint(
      positions, command.max_acceleration, command.max_velocity, 0.0, 0.0);
    if (result != 0) {
      throw std::runtime_error("SDK rejected the joint move command");
    }
    waitForMotionCompletion();
  }

  void executeLinearMove(const AuboLinearMove & command)
  {
    ensureRobotRunning();
    const auto pose = poseToSdkVector(command.target.pose);
    const std::vector<double> target(pose.begin(), pose.end());
    const int result = robot_interface_->getMotionControl()->moveLine(
      target, command.speed, command.acceleration, command.blend_radius, 0);
    if (result != 0) {
      throw std::runtime_error("SDK rejected the linear move command");
    }
    waitForMotionCompletion();
  }

  std::shared_ptr<RpcClient> rpc_client_;
  RobotInterfacePtr robot_interface_;
  std::string robot_name_;
  std::string base_frame_;
  std::chrono::seconds mode_timeout_{60};
  std::chrono::seconds motion_timeout_{180};

  rclcpp::Publisher<AuboCommandStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<AuboPowerCommand>::SharedPtr power_subscription_;
  rclcpp::Subscription<AuboJointMove>::SharedPtr joint_subscription_;
  rclcpp::Subscription<AuboLinearMove>::SharedPtr linear_subscription_;

  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::queue<Command> command_queue_;
  bool busy_{false};
  bool stopping_{false};
  std::thread worker_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AuboCommandNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("aubo_command_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
