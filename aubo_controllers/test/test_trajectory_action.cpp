#include "aubo_controllers/aubo_joint_trajectory_controller.h"
#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <limits>

namespace aubo_driver {
class TrajectoryActionTest : public ::testing::Test {
protected:
    using Action = control_msgs::action::FollowJointTrajectory;
    AuboJointTrajectoryController controller;
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Node::SharedPtr client_node;
    rclcpp_action::Client<Action>::SharedPtr client;
    double position = 0, velocity = 0, command = 0, execution_state = 0;
    std::vector<hardware_interface::StateInterface> states;
    std::vector<hardware_interface::CommandInterface> commands;
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override {
        auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true);
        options.parameter_overrides({
            rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
            rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position"}),
            rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position", "velocity"}),
            rclcpp::Parameter("constraints.stopped_velocity_tolerance", 0.0),
            rclcpp::Parameter("constraints.joint.goal", 0.01),
            rclcpp::Parameter("constraints.goal_time", 0.1)});
        ASSERT_EQ(controller.init("test_trajectory", "", options), controller_interface::return_type::OK);
        ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        states.reserve(3);
        states.emplace_back("joint", "position", &position);
        states.emplace_back("joint", "velocity", &velocity);
        states.emplace_back("motion_status", "execution_state", &execution_state);
        commands.emplace_back("joint", "position", &command);
        std::vector<hardware_interface::LoanedStateInterface> loaned_states;
        std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
        for (auto &state : states) loaned_states.emplace_back(state);
        for (auto &cmd : commands) loaned_commands.emplace_back(cmd);
        controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
        ASSERT_EQ(controller.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
        executor.add_node(controller.get_node()->get_node_base_interface());
        client_node = std::make_shared<rclcpp::Node>("trajectory_test_client");
        executor.add_node(client_node);
        client = rclcpp_action::create_client<Action>(client_node, "/test_trajectory/follow_joint_trajectory");
        ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
    }
    void TearDown() override {
        if (controller.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            controller.get_node()->deactivate();
        if (client_node) {
            executor.remove_node(controller.get_node()->get_node_base_interface());
            executor.remove_node(client_node);
        }
    }
    template<class Future> bool spinUntil(Future &future) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    }
    void runGoal(double status, rclcpp_action::ResultCode expected, bool deactivate = false, double target = 0.0) {
        execution_state = 0;
        Action::Goal goal;
        goal.trajectory.joint_names = {"joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {target};
        point.time_from_start = rclcpp::Duration::from_seconds(.15);
        goal.trajectory.points = {point};
        auto accepted = client->async_send_goal(goal);
        ASSERT_TRUE(spinUntil(accepted));
        auto handle = accepted.get();
        ASSERT_NE(handle, nullptr);
        auto result = client->async_get_result(handle);
        execution_state = status;
        if (deactivate) controller.get_node()->deactivate();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (result.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
               std::chrono::steady_clock::now() < deadline) {
            if (!deactivate) controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(.01));
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        ASSERT_EQ(result.wait_for(std::chrono::seconds(0)), std::future_status::ready);
        EXPECT_EQ(result.get().code, expected);
        if (expected == rclcpp_action::ResultCode::ABORTED) {
            EXPECT_NE(result.get().result->error_code, Action::Result::SUCCESSFUL);
        }
    }
};
TEST_F(TrajectoryActionTest, ReadyGoalSucceeds) { runGoal(0, rclcpp_action::ResultCode::SUCCEEDED); }
TEST_F(TrajectoryActionTest, SafetyPauseAbortsAndAllowsNewGoalAfterRecovery) {
    runGoal(1, rclcpp_action::ResultCode::ABORTED);
    runGoal(0, rclcpp_action::ResultCode::SUCCEEDED);
}
TEST_F(TrajectoryActionTest, HardwareFaultAbortsInsteadOfSucceeding) { runGoal(2, rclcpp_action::ResultCode::ABORTED); }
TEST_F(TrajectoryActionTest, InvalidHardwareStatusCannotSucceed) {
    runGoal(std::numeric_limits<double>::quiet_NaN(), rclcpp_action::ResultCode::ABORTED);
}
TEST_F(TrajectoryActionTest, FrozenFeedbackCannotReachMovingGoal) {
    runGoal(0, rclcpp_action::ResultCode::ABORTED, false, 1.0);
}
TEST_F(TrajectoryActionTest, MissingHardwareStatusPreventsActivation) {
    controller.get_node()->deactivate();
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    loaned_states.emplace_back(states[0]);
    loaned_states.emplace_back(states[1]);
    loaned_commands.emplace_back(commands[0]);
    controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::ERROR);
}
TEST_F(TrajectoryActionTest, DeactivationAbortsActiveGoal) { runGoal(2, rclcpp_action::ResultCode::ABORTED, true); }
}  // namespace aubo_driver
