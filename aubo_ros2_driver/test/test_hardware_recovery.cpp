#include "aubo_hardware_interface.h"
#include <gtest/gtest.h>

namespace aubo_driver {
class AuboHardwareInterfaceRecoveryTest : public ::testing::Test {
protected:
    AuboHardwareInterface hw;
    void ready() {
        hw.robot_mode_ = RobotModeType::Running;
        hw.safety_mode_ = SafetyModeType::Normal;
        hw.last_motion_feedback_ = std::chrono::steady_clock::now();
    }
    hardware_interface::return_type write() {
        return hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
    }
    void safetyPause() {
        ready();
        hw.position_controller_running_ = true;
        hw.servo_mode_start_ = true;
        hw.servo_mode_pending_ = true;
        hw.robot_mode_ = RobotModeType::PowerOff;
        EXPECT_EQ(write(), hardware_interface::return_type::OK);
        EXPECT_FALSE(hw.servo_fault_latched_);
        EXPECT_FALSE(hw.servo_mode_start_);
        EXPECT_FALSE(hw.servo_mode_pending_);
        EXPECT_TRUE(hw.motion_command_paused_);
        // Disable motion so recovery can be exercised without a robot RPC.
        hw.position_controller_running_ = false;
        ready();
        EXPECT_EQ(write(), hardware_interface::return_type::OK);
        EXPECT_FALSE(hw.servo_fault_latched_);
        EXPECT_FALSE(hw.motion_command_paused_);
    }
    void feedbackPauseAndRecover() {
        ready();
        hw.position_controller_running_ = true;
        hw.last_motion_feedback_ -= std::chrono::milliseconds(150);
        EXPECT_EQ(write(), hardware_interface::return_type::OK);
        EXPECT_TRUE(hw.motion_command_paused_);
        EXPECT_FALSE(hw.servo_fault_latched_);
        hw.position_controller_running_ = false;
        ready();
        EXPECT_EQ(write(), hardware_interface::return_type::OK);
        EXPECT_FALSE(hw.motion_command_paused_);
    }
    void sustainedFeedbackFailure() {
        ready();
        hw.position_controller_running_ = true;
        hw.last_motion_feedback_ -= std::chrono::seconds(2);
        EXPECT_EQ(write(), hardware_interface::return_type::ERROR);
        ready();
        EXPECT_EQ(write(), hardware_interface::return_type::ERROR);
        EXPECT_TRUE(hw.servo_fault_latched_);
    }
    void safetyResultWaitsForNewFeedback() {
        ready();
        hw.motion_execution_state_ = 0.0;
        hw.position_controller_running_ = true;
        hw.velocity_controller_running_ = true;  // Avoid any RPC in the recovery cycle.
        hw.actual_q_.assign(6, .25);
        for (int result : {207, 208, 209, 210, 211, 212}) {
            EXPECT_EQ(hw.processServoResult(result, 0), 0);
            EXPECT_FALSE(hw.servo_fault_latched_);
            EXPECT_EQ(hw.motion_execution_state_, 1.0);
            // Cached Normal/Running is insufficient after a rejection.
            EXPECT_EQ(write(), hardware_interface::return_type::OK);
            EXPECT_TRUE(hw.safety_return_pause_);
            EXPECT_TRUE(hw.motion_command_paused_);
            EXPECT_EQ(hw.motion_execution_state_, 1.0);
            // New feedback still unsafe must keep output paused.
            ready();
            hw.robot_mode_ = RobotModeType::PowerOff;
            EXPECT_EQ(write(), hardware_interface::return_type::OK);
            EXPECT_EQ(hw.motion_execution_state_, 1.0);
            // Fresh ready feedback releases the pause, with commands resynced.
            ready();
            EXPECT_EQ(write(), hardware_interface::return_type::OK);
            EXPECT_FALSE(hw.safety_return_pause_);
            EXPECT_FALSE(hw.motion_command_paused_);
            EXPECT_EQ(hw.motion_execution_state_, 0.0);
            for (double q : hw.aubo_position_commands_) EXPECT_DOUBLE_EQ(q, .25);
        }
    }
    void faultStatusSurvivesCleanup() {
        ready();
        hw.motion_execution_state_ = 0.0;
        EXPECT_EQ(hw.latchServoFault("test"), hardware_interface::return_type::ERROR);
        EXPECT_EQ(hw.motion_execution_state_, 2.0);
        hw.on_error(rclcpp_lifecycle::State());
        hw.refreshMotionStatus();
        EXPECT_EQ(hw.motion_execution_state_, 2.0);
        EXPECT_EQ(write(), hardware_interface::return_type::ERROR);
    }
    void stateIsPublishedBeforeUpdate() {
        ready();
        hw.motion_execution_state_ = 0.0;
        hw.safety_mode_ = SafetyModeType::ProtectiveStop;
        hw.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
        EXPECT_EQ(hw.motion_execution_state_, 1.0);
        EXPECT_FALSE(hw.servo_fault_latched_);
    }
    void boundedSessionRecovery() {
        EXPECT_EQ(hw.retryServoSession("test transient"), 0);
        EXPECT_FALSE(hw.servo_fault_latched_);
        hw.servo_recovery_since_ -= std::chrono::seconds(2);
        EXPECT_EQ(hw.retryServoSession("test persistent"), -1);
    }
    void expiredSession() {
        ready();
        hw.rpc_client_ = std::make_shared<RpcClient>();
        hw.robot_name_ = "test";
        hw.servo_mode_start_ = true;
        hw.last_servo_attempt_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
        EXPECT_EQ(hw.Servoj({}), 0);
        EXPECT_FALSE(hw.servo_fault_latched_);
        EXPECT_FALSE(hw.servo_mode_start_);
        EXPECT_TRUE(hw.motion_command_paused_);
    }
    void recoverySynchronizesCommands() {
        ready();
        hw.actual_q_.assign(6, .25);
        hw.aubo_position_commands_.fill(1.0);
        hw.aubo_velocity_commands_.fill(1.0);
        hw.motion_command_paused_ = true;
        hw.position_controller_running_ = true;
        hw.velocity_controller_running_ = true;
        EXPECT_EQ(write(), hardware_interface::return_type::OK);
        for (double q : hw.aubo_position_commands_) EXPECT_DOUBLE_EQ(q, .25);
        for (double v : hw.aubo_velocity_commands_) EXPECT_DOUBLE_EQ(v, 0.0);
        EXPECT_FALSE(hw.motion_command_paused_);
    }
    void unsupportedVelocity() {
        ready();
        hw.velocity_controller_running_ = true;
        hw.last_motion_feedback_ -= std::chrono::seconds(2);
        EXPECT_EQ(write(), hardware_interface::return_type::OK);
        EXPECT_EQ(write(), hardware_interface::return_type::OK);
        EXPECT_FALSE(hw.servo_fault_latched_);
    }
};
TEST_F(AuboHardwareInterfaceRecoveryTest, SafetyStateResumesWithoutLatch) { safetyPause(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, TransientFeedbackLossResumes) { feedbackPauseAndRecover(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, SustainedFeedbackLossLatches) { sustainedFeedbackFailure(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, SafetyResultsRequireFreshReadyFeedback) { safetyResultWaitsForNewFeedback(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, FaultStateSurvivesErrorCleanup) { faultStatusSurvivesCleanup(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, ReadExposesUnsafeStateBeforeControllerUpdate) { stateIsPublishedBeforeUpdate(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, SessionRecoveryHasPersistentFailureBudget) { boundedSessionRecovery(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, ExpiredSessionPausesWithoutLatch) { expiredSession(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, RecoverySynchronizesCommands) { recoverySynchronizesCommands(); }
TEST_F(AuboHardwareInterfaceRecoveryTest, UnsupportedVelocityIsNonFatal) { unsupportedVelocity(); }
}  // namespace aubo_driver
