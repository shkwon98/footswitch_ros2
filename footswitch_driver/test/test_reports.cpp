#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <iostream>
#include <thread>
#include <vector>

#include "footswitch_driver/footswitch_driver.hpp"

// Replace only USB I/O; exercise the actual node, timer, decoding and ROS output.
static std::deque<std::vector<unsigned char>> reports;
extern "C"
{
    int hid_init()
    {
        return 0;
    }
    int hid_exit()
    {
        return 0;
    }
    hid_device_info *hid_enumerate(unsigned short vendor, unsigned short product)
    {
        static hid_device_info info{};
        info.interface_number = 0;
        return vendor == 0x3553 && product == 0xb001 ? &info : nullptr;
    }
    void hid_free_enumeration(hid_device_info *) {}
    hid_device *hid_open_path(const char *)
    {
        return reinterpret_cast<hid_device *>(&reports);
    }
    void hid_close(hid_device *) {}
    int hid_set_nonblocking(hid_device *, int)
    {
        return 0;
    }
    const wchar_t *hid_error(hid_device *)
    {
        return L"test disconnect";
    }
    int hid_read(hid_device *, unsigned char *buffer, size_t capacity)
    {
        if (reports.empty())
        {
            return 0;
        }
        const auto report = reports.front();
        reports.pop_front();
        if (report.size() > capacity || report.front() == 255)
        {
            return -1;
        }
        std::copy(report.begin(), report.end(), buffer);
        return static_cast<int>(report.size());
    }
}

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;
    using footswitch_driver::FootswitchState;
    rclcpp::init(argc, argv);
    auto driver = std::make_shared<footswitch_driver::FootSwitch>();
    auto observer = std::make_shared<rclcpp::Node>("report_test");
    std::vector<std::array<bool, 3>> states;
    bool is_input_alive = false;
    rclcpp::SubscriptionOptions options;
    options.event_callbacks.liveliness_callback = [&](rclcpp::QOSLivelinessChangedInfo &event)
    { is_input_alive = event.alive_count > 0; };
    auto subscription = observer->create_subscription<FootswitchState>(
        "state",
        rclcpp::QoS(10)
            .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
            .liveliness_lease_duration(500ms),
        [&](const FootswitchState &message) { states.push_back(message.state); }, options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(driver);
    executor.add_node(observer);
    const auto spin = [&](auto duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline)
        {
            executor.spin_some();
            std::this_thread::sleep_for(2ms);
        }
    };
    spin(500ms);
    reports = {{0, 0, 4, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}};
    spin(400ms);
    reports = {{1, 0, 0, 6, 0, 0, 0, 0, 0}, // Optional report ID.
               {1, 0, 0, 6, 0, 0, 0, 0, 0}, // Duplicate held state.
               {0, 0},
               {0, 0, 1, 1, 1, 1, 1, 1}, // Truncation and keyboard rollover.
               {0, 0, 6, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0},
               {0, 0, 5, 0, 0, 0, 0, 0}};
    spin(500ms);
    reports = {{0, 0, 0x1e, 0, 0, 0, 0, 0}, // Number-row 1/2/3 match A/B/C.
               {1, 0, 0, 0x1f, 0, 0, 0, 0, 0},
               {0, 0, 0x20, 0, 0, 0, 0, 0},
               {0, 0, 0x59, 0, 0, 0, 0, 0}, // Keypad 1/2/3.
               {0, 0, 0x5a, 0, 0, 0, 0, 0},
               {0, 0, 0x5b, 0, 0, 0, 0, 0},
               {0, 0, 0, 0, 0, 0, 0, 0}};
    spin(400ms);
    const std::vector<std::array<bool, 3>> expected{{true, false, false},
                                                    {false, false, false},
                                                    {false, false, true},
                                                    {false, false, false},
                                                    {false, true, false},
                                                    {true, false, false},
                                                    {false, true, false},
                                                    {false, false, true},
                                                    {true, false, false},
                                                    {false, true, false},
                                                    {false, false, true},
                                                    {false, false, false}};
    bool is_correct = states == expected && is_input_alive;
    if (!is_correct)
    {
        std::cerr
            << "Every valid transition must reach the subscriber with healthy input; received "
            << states.size() << " states\n";
    }
    reports = {{255}}; // A USB read error must stop asserting input health.
    spin(700ms);
    if (is_input_alive || states != expected)
    {
        std::cerr << "A read failure must lose liveliness without synthesizing pedal states\n";
        is_correct = false;
    }
    rclcpp::shutdown();
    return is_correct ? 0 : 1;
}
