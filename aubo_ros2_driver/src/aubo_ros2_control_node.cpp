#include <thread>
#include <memory>
#include <fstream>
#include <string>
#include <sys/resource.h>

// ROS includes
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace {
std::string rtprio_failure_hint()
{
    std::ifstream in("/proc/self/status");
    std::string line;
    std::string capeff = "unknown";
    while (std::getline(in, line)) {
        if (line.compare(0, 7, "CapEff:") == 0) {
            capeff = line.substr(7);
            while (!capeff.empty() && (capeff.front() == ' ' || capeff.front() == '\t')) {
                capeff.erase(capeff.begin());
            }
            break;
        }
    }
    bool caps_zero = true;
    for (char c : capeff) {
        if (c != '0' && c != ' ' && c != '\t') {
            caps_zero = false;
            break;
        }
    }
    if (caps_zero) {
        return "无法提升 RLIMIT_RTPRIO（CapEff=" + capeff +
               "）。请运行 aubo_ros2_driver/scripts/configure_aubo_realtime.sh；"
               "rebuild 会清掉 setcap。若 getcap 已有 cap 仍为 0，检查挂载是否含 nosuid。";
    }
    return "无法提升 RLIMIT_RTPRIO（进程已有 file caps）。检查 PAM limits / ulimit -r。";
}

bool raise_rt_limits()
{
    struct rlimit lim {};
    if (getrlimit(RLIMIT_RTPRIO, &lim) != 0) {
        return false;
    }
    lim.rlim_cur = 95;
    if (lim.rlim_max < lim.rlim_cur) {
        lim.rlim_max = lim.rlim_cur;
    }
    if (setrlimit(RLIMIT_RTPRIO, &lim) != 0) {
        if (getrlimit(RLIMIT_RTPRIO, &lim) == 0 && lim.rlim_max > 0) {
            lim.rlim_cur = lim.rlim_max;
            return setrlimit(RLIMIT_RTPRIO, &lim) == 0;
        }
        return false;
    }
    return true;
}
}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Executor> e =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto controller_manager =
        std::make_shared<controller_manager::ControllerManager>(
            e, "controller_manager");

    std::thread control_loop([controller_manager]() {
        if (!raise_rt_limits()) {
            RCLCPP_WARN(controller_manager->get_logger(), "%s",
                        rtprio_failure_hint().c_str());
        }
        if (!realtime_tools::configure_sched_fifo(50)) {
            RCLCPP_WARN(controller_manager->get_logger(),
                        "Could not enable FIFO RT scheduling policy");
        } else {
            RCLCPP_INFO(controller_manager->get_logger(),
                        "FIFO RT scheduling enabled (priority 50)");
        }

        auto const period = std::chrono::nanoseconds(
            1'000'000'000 / controller_manager->get_update_rate());
        auto const cm_now =
            std::chrono::nanoseconds(controller_manager->now().nanoseconds());
        std::chrono::time_point<std::chrono::system_clock,
                                std::chrono::nanoseconds>
            next_iteration_time{ cm_now };

        rclcpp::Time previous_time = controller_manager->now();

        while (rclcpp::ok()) {
            auto const current_time = controller_manager->now();
            auto const measured_period = current_time - previous_time;
            previous_time = current_time;

            controller_manager->read(controller_manager->now(),
                                     measured_period);
            controller_manager->update(controller_manager->now(),
                                       measured_period);
            controller_manager->write(controller_manager->now(),
                                      measured_period);

            next_iteration_time += period;
            std::this_thread::sleep_until(next_iteration_time);
        }
    });

    e->add_node(controller_manager);
    e->spin();

    control_loop.join();
    rclcpp::shutdown();
    return 0;
}
