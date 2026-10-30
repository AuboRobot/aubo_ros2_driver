#include "servo_return_policy.h"

#include <gtest/gtest.h>

namespace aubo_driver {
namespace {
constexpr std::int64_t kMs = 1000000;

TEST(ServoReturnPolicy, RejectedSessionLatchesUntilExplicitReset)
{
    ServoReturnPolicy policy;
    EXPECT_FALSE(policy.observe(-13, 0));
    EXPECT_TRUE(policy.faulted());
    EXPECT_FALSE(policy.observe(0, kMs));
    EXPECT_FALSE(policy.observe(2, 2 * kMs));
    policy.reset();
    EXPECT_FALSE(policy.faulted());
    EXPECT_TRUE(policy.observe(0, 3 * kMs));
}

TEST(ServoReturnPolicy, QueueFullAndBusyShareExactTimeBudget)
{
    ServoReturnPolicy policy;
    EXPECT_TRUE(policy.observe(2, 0));
    EXPECT_TRUE(policy.observe(3, 50 * kMs));
    EXPECT_TRUE(policy.observe(2, 149 * kMs));
    EXPECT_FALSE(policy.observe(3, 150 * kMs));
}

TEST(ServoReturnPolicy, InvalidArgumentUsesExactTimeBudget)
{
    ServoReturnPolicy policy;
    EXPECT_TRUE(policy.observe(-5, 10 * kMs));
    EXPECT_TRUE(policy.observe(-5, 34 * kMs));
    EXPECT_FALSE(policy.observe(-5, 35 * kMs));
}

TEST(ServoReturnPolicy, ChangingCategoryDoesNotClearInvalidWindow)
{
    ServoReturnPolicy policy;
    EXPECT_TRUE(policy.observe(-5, 0));
    EXPECT_TRUE(policy.observe(2, 10 * kMs));
    EXPECT_TRUE(policy.observe(3, 24 * kMs));
    EXPECT_FALSE(policy.observe(2, 25 * kMs));
}

TEST(ServoReturnPolicy, ChangingCategoryDoesNotClearBusyWindow)
{
    ServoReturnPolicy policy;
    EXPECT_TRUE(policy.observe(2, 0));
    EXPECT_TRUE(policy.observe(-5, 149 * kMs));
    EXPECT_FALSE(policy.observe(-5, 150 * kMs));
}

TEST(ServoReturnPolicy, SuccessClearsBothPendingWindows)
{
    ServoReturnPolicy policy;
    EXPECT_TRUE(policy.observe(2, 0));
    EXPECT_TRUE(policy.observe(-5, 10 * kMs));
    EXPECT_TRUE(policy.observe(0, 20 * kMs));
    EXPECT_TRUE(policy.observe(3, 200 * kMs));
    EXPECT_TRUE(policy.observe(-5, 210 * kMs));
    EXPECT_TRUE(policy.observe(2, 234 * kMs));
    EXPECT_FALSE(policy.observe(3, 235 * kMs));
}

TEST(ServoReturnPolicy, ResetClearsPendingWindows)
{
    ServoReturnPolicy policy;
    EXPECT_TRUE(policy.observe(-5, 0));
    policy.reset();
    EXPECT_TRUE(policy.observe(-5, 100 * kMs));
    EXPECT_TRUE(policy.observe(-5, 124 * kMs));
    EXPECT_FALSE(policy.observe(-5, 125 * kMs));
}

TEST(ServoReturnPolicy, UnexpectedResultsImmediatelyLatch)
{
    for (int result : {-1, -4, -18, -22, 1, 4, 211}) {
        ServoReturnPolicy policy;
        EXPECT_FALSE(policy.observe(result, 0)) << result;
        EXPECT_TRUE(policy.faulted()) << result;
    }
}

TEST(ServoReturnPolicy, BudgetDependsOnElapsedTimeRatherThanAttemptCount)
{
    ServoReturnPolicy policy;
    for (std::int64_t ns = 0; ns < 25 * kMs; ns += 1000) {
        ASSERT_TRUE(policy.observe(-5, ns));
    }
    EXPECT_FALSE(policy.observe(-5, 25 * kMs));
}
}  // namespace
}  // namespace aubo_driver
