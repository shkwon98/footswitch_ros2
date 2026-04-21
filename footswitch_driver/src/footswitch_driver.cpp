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

#include <algorithm>
#include <memory>

#include "footswitch_driver/footswitch_driver.hpp"

namespace footswitch_driver
{

namespace
{

constexpr int kKeyboardInterfaceNumber = 0;

hid_device *open_footswitch_keyboard_interface()
{
    hid_device_info *devices = hid_enumerate(VEND_ID, PROD_ID);
    hid_device_info *current = devices;
    hid_device *handle = nullptr;

    while (current != nullptr)
    {
        if (current->interface_number == kKeyboardInterfaceNumber)
        {
            handle = hid_open_path(current->path);
            break;
        }
        current = current->next;
    }

    hid_free_enumeration(devices);
    return handle;
}

} // namespace

FootSwitch::FootSwitch(const rclcpp::NodeOptions &options)
    : rclcpp::Node("footswitch_node")
{
    // Initialize the hidapi library
    hid_init();

    // Open the keyboard HID interface explicitly. This device exposes multiple
    // interfaces under the same VID/PID, and opening an arbitrary one leads to
    // incorrect pedal decoding.
    handle_ = open_footswitch_keyboard_interface();
    if (!handle_)
    {
        RCLCPP_FATAL(get_logger(), "Unable to open keyboard interface for device with VID '%x' and PID '%x'. Exiting...",
                     VEND_ID, PROD_ID);
        hid_exit();
        ::exit(1);
    }

    // Set the hid_read() function to be non-blocking.
    hid_set_nonblocking(handle_, 1);

    int update_rate;
    declare_parameter("update_rate", 10);
    get_parameter("update_rate", update_rate);

    update_period_ms_ = static_cast<int64_t>((1.0 / static_cast<float>(update_rate)) * 1'000);

    pub_footswitch_state_ = create_publisher<FootswitchState>("state", 10);

    RCLCPP_INFO(get_logger(), "Publishing footswitch state on topic '%s'.", pub_footswitch_state_->get_topic_name());

    tmr_update_state_ =
        create_wall_timer(std::chrono::milliseconds(update_period_ms_), std::bind(&FootSwitch::update_state, this));

    RCLCPP_INFO(get_logger(), "Initialized Footswitch driver.");
}

FootSwitch::~FootSwitch()
{
    RCLCPP_INFO(get_logger(), "Exiting Footswitch driver.");
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
    std::array<bool, 3> current_state = previous_state_;
    unsigned char buf[65] = { 0 };
    int i = 0;

    // Drain all queued HID reports during this update window. If no new report
    // arrives, keep the previous state instead of forcing an immediate release.
    while (rclcpp::ok())
    {
        const int res = hid_read(handle_, buf, sizeof(buf));
        if (res < 0)
        {
            RCLCPP_ERROR(get_logger(), "Unable to read from footswitch: %ls", hid_error(handle_));
            break;
        }

        if (res > 0)
        {
            std::array<bool, 3> parsed_state = { false, false, false };

            // Parse keycode slots and map key values to the left/middle/right pedals.
            // Some devices prepend a report-id byte, shifting keycodes by +1.
            const int keycode_start = (res >= 9) ? 3 : 2;
            const int keycode_end = std::min(res, keycode_start + 6);
            for (int byte_idx = keycode_start; byte_idx < keycode_end; byte_idx++)
            {
                switch (buf[byte_idx])
                {
                case 4:
                    parsed_state[0] = true;
                    break;
                case 5:
                    parsed_state[1] = true;
                    break;
                case 6:
                    parsed_state[2] = true;
                    break;
                default:
                    break;
                }
            }

            current_state = parsed_state;
        }

        i++;
        if (i >= update_period_ms_)
        {
            break;
        }

        get_clock()->sleep_for(std::chrono::milliseconds(1));
    }

    if (current_state != previous_state_)
    {
        FootswitchState state;
        state.state = current_state;
        state.header.stamp = now();
        pub_footswitch_state_->publish(state);
        previous_state_ = current_state;
    }
}

} // namespace footswitch_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(footswitch_driver::FootSwitch)
