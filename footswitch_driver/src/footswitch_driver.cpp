// Copyright 2024 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "footswitch_driver/footswitch_driver.hpp"

#include <algorithm>
#include <filesystem>
#include <glob.h>
#include <fcntl.h>
#include <linux/input.h>
#include <memory>
#include <sys/ioctl.h>
#include <system_error>
#include <vector>
#include <stdexcept>

namespace footswitch_driver
{

namespace
{

constexpr int kKeyboardInterfaceNumber = 0;

hid_device *open_footswitch_keyboard_interface(std::string &device_path)
{
    hid_device_info *devices = hid_enumerate(VEND_ID, PROD_ID);
    hid_device_info *current = devices;
    hid_device *handle = nullptr;

    while (current != nullptr)
    {
        if (current->interface_number == kKeyboardInterfaceNumber)
        {
            handle = hid_open_path(current->path);
            if (handle)
            {
                device_path = current->path;
            }
            break;
        }
        current = current->next;
    }

    hid_free_enumeration(devices);
    return handle;
}

std::vector<int> grab_keyboard_inputs(const std::string &hidraw_path)
{
    const auto pattern = "/sys/class/hidraw/" +
        std::filesystem::path(hidraw_path).filename().string() + "/device/input/input*/event*";
    glob_t matches{};
    const int result = glob(pattern.c_str(), 0, nullptr, &matches);
    if (result != 0)
    {
        globfree(&matches);
        throw std::runtime_error("No input event devices found for footswitch " + hidraw_path);
    }

    std::vector<int> inputs;
    try
    {
        inputs.reserve(matches.gl_pathc);
        for (std::size_t index = 0; index < matches.gl_pathc; ++index)
        {
            const auto device = std::filesystem::path("/dev/input") /
                std::filesystem::path(matches.gl_pathv[index]).filename();
            const int fd = open(device.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd < 0)
            {
                throw std::system_error(errno, std::generic_category(),
                    "Cannot open " + device.string() + "; install the footswitch udev rule");
            }
            inputs.push_back(fd);
            if (ioctl(fd, EVIOCGRAB, 1) < 0)
            {
                throw std::system_error(errno, std::generic_category(),
                    "Cannot exclusively grab " + device.string());
            }
        }
    }
    catch (...)
    {
        for (const int fd : inputs)
        {
            close(fd);
        }
        globfree(&matches);
        throw;
    }
    globfree(&matches);
    return inputs;
}

} // namespace

FootSwitch::FootSwitch(const rclcpp::NodeOptions &options)
    : rclcpp::Node("footswitch_node", options)
{
    // Initialize the hidapi library
    if (hid_init() < 0)
    {
        throw std::runtime_error("Unable to initialize HIDAPI");
    }

    // Open the keyboard HID interface explicitly. This device exposes multiple
    // interfaces under the same VID/PID, and opening an arbitrary one leads to
    // incorrect pedal decoding.
    std::string device_path;
    handle_ = open_footswitch_keyboard_interface(device_path);
    if (!handle_)
    {
        RCLCPP_FATAL(
            get_logger(),
            "Unable to open keyboard interface for device with VID '%x' and PID '%x'. Exiting...",
            VEND_ID, PROD_ID);
        hid_exit();
        ::exit(1);
    }

    // Set the hid_read() function to be non-blocking.
    if (hid_set_nonblocking(handle_, 1) < 0)
    {
        hid_close(handle_);
        hid_exit();
        throw std::runtime_error("Unable to enable nonblocking footswitch input");
    }

    int update_rate;
    declare_parameter("update_rate", 10);
    get_parameter("update_rate", update_rate);
    if (update_rate <= 0 || update_rate > 1000)
    {
        hid_close(handle_);
        hid_exit();
        throw std::invalid_argument("update_rate must be in [1, 1000] Hz");
    }

    update_period_ms_ = 1000 / update_rate;

    // Assert health after successful HID polling, even when no pedal changes.
    const auto qos =
        rclcpp::QoS(10)
            .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
            .liveliness_lease_duration(std::chrono::milliseconds(3 * update_period_ms_));
    pub_footswitch_state_ = create_publisher<FootswitchState>("state", qos);

    RCLCPP_INFO(get_logger(), "Publishing footswitch state on topic '%s'.",
                pub_footswitch_state_->get_topic_name());

    tmr_update_state_ = create_wall_timer(std::chrono::milliseconds(update_period_ms_),
                                          std::bind(&FootSwitch::update_state, this));

    // Grabbing evdev blocks desktop key delivery while HIDAPI continues reading reports.
    try
    {
        input_fds_ = grab_keyboard_inputs(device_path);
    }
    catch (...)
    {
        hid_close(handle_);
        hid_exit();
        throw;
    }

    RCLCPP_INFO(get_logger(), "Initialized Footswitch driver; desktop key input suppressed.");
}

FootSwitch::~FootSwitch()
{
    RCLCPP_INFO(get_logger(), "Exiting Footswitch driver.");
    // Closing evdev descriptors releases exclusive input ownership, including on process exit.
    for (const int fd : input_fds_)
    {
        close(fd);
    }

    // Close the device
    if (handle_)
    {
        hid_close(handle_);
    }

    // Finalize the hidapi library
    hid_exit();
}

void FootSwitch::update_state()
{
    unsigned char buf[65] = {0};

    // Publish each transition: the final report alone can hide a short press.
    // Bound work per callback and leave waiting to the wall timer.
    for (int i = 0; i < 32 && rclcpp::ok(); ++i)
    {
        const int res = hid_read(handle_, buf, sizeof(buf));
        if (res < 0)
        {
            RCLCPP_ERROR(get_logger(), "Unable to read from footswitch: %ls", hid_error(handle_));
            // Let subscribers detect loss of HID health without inventing pedal states.
            tmr_update_state_->cancel();
            return;
        }
        if (res == 0)
        {
            break;
        }
        if (res != 8 && res != 9)
        {
            continue;
        }

        std::array<bool, 3> parsed_state = {false, false, false};

        // Some devices prepend a report ID. Ignore keyboard rollover/error reports.
        const int keycode_start = res - 6;
        if (std::any_of(buf + keycode_start, buf + res,
                        [](unsigned char key) { return key >= 1 && key <= 3; }))
        {
            continue;
        }
        for (int byte_idx = keycode_start; byte_idx < res; ++byte_idx)
        {
            switch (buf[byte_idx])
            {
            case 0x04: // A
            case 0x1e: // 1
            case 0x59: // Keypad 1
                parsed_state[0] = true;
                break;
            case 0x05: // B
            case 0x1f: // 2
            case 0x5a: // Keypad 2
                parsed_state[1] = true;
                break;
            case 0x06: // C
            case 0x20: // 3
            case 0x5b: // Keypad 3
                parsed_state[2] = true;
                break;
            default:
                break;
            }
        }

        if (parsed_state != previous_state_)
        {
            FootswitchState state;
            state.state = parsed_state;
            state.header.stamp = now();
            pub_footswitch_state_->publish(state);
            previous_state_ = parsed_state;
        }
    }
    if (!pub_footswitch_state_->assert_liveliness())
    {
        RCLCPP_ERROR(get_logger(), "Unable to assert footswitch input health; restart the driver");
        tmr_update_state_->cancel();
    }
}

} // namespace footswitch_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(footswitch_driver::FootSwitch)
