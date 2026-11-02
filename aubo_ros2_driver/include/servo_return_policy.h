#pragma once
#include <cstdint>

namespace aubo_driver {
// Monotonic elapsed-time budgets, independent of controller update frequency.
// 2/3 are tolerated temporarily, but do not acknowledge a new target.
class ServoReturnPolicy {
public:
    bool observe(int result, std::int64_t now_ns) {
        if (fault_) return false;
        if (result == 0) { invalid_since_ = -1; busy_since_ = -1; return true; }
        if (result != -5 && result != 2 && result != 3) {
            fault_ = true;
            return false;
        }
        // Only actual success clears a pending failure window. Alternating
        // categories or busy codes must never keep a failed stream alive.
        auto &since = result == -5 ? invalid_since_ : busy_since_;
        if (since < 0) since = now_ns;
        if ((invalid_since_ >= 0 && now_ns - invalid_since_ >= 25000000) ||
            (busy_since_ >= 0 && now_ns - busy_since_ >= 150000000))
            fault_ = true;
        return !fault_;
    }
    bool faulted() const { return fault_; }
    void reset() { fault_ = false; invalid_since_ = -1; busy_since_ = -1; }
private:
    bool fault_{false};
    std::int64_t invalid_since_{-1};
    std::int64_t busy_since_{-1};
};
} // namespace aubo_driver
