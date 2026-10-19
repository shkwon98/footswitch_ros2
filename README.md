# ROS 2 Footswitch

ROS 2 driver and message interfaces for the 3-pedal PCsensor USB footswitch.

## Hardware

- Vendor ID: `0x3553`
- Product ID: `0xb001`

## Packages

- `footswitch_msgs`: Message definitions (`FootswitchState`)
- `footswitch_driver`: HID-based ROS 2 node publishing pedal states

## Getting Started

```bash
cd <YOUR_ROS2_WORKSPACE>
git clone https://github.com/shkwon98/footswitch_ros2.git src/footswitch_ros2
rosdep install --from-paths src/footswitch_ros2 --ignore-src -r -y
```

## Pedal Key Configuration

The driver decodes USB HID key reports, not physical switch IDs. Before use,
configure the device's left/middle/right pedals as A/B/C or 1/2/3:

| Pedal | Accepted device key | Published field |
| --- | --- | --- |
| Left | A or 1 | `state[0]` |
| Middle | B or 2 | `state[1]` |
| Right | C or 3 | `state[2]` |

The driver accepts number-row and keypad 1/2/3. Configure keyboard keys from
the table; other keys and mouse/media actions are ignored.

If the device has different assignments, use the
[PCsensor configuration software](https://pcsensor.com/download/) to set each
pedal to a single keyboard key and save the assignments to the device. Do not
use text macros or automatic press/release sequences. With robot control stopped,
check each pedal's key in a text editor before starting the ROS driver.

Save the pedal key assignments with the PCsensor software. Then install the
udev rule below to grant USB access.

## udev Rule

`hidraw` access is usually restricted to `root`, so install the bundled udev rule before building or running the driver as a normal user.

```bash
cd <YOUR_ROS2_WORKSPACE>/src/footswitch_ros2
./scripts/install_udev_rules.sh
```

Unplug and reconnect the footswitch after applying the rule.
The rule grants access to the active local session. Reinstall it if upgrading
from an older version. While the driver runs, pedal keys are not sent to desktop applications.

## Build

```bash
cd <YOUR_ROS2_WORKSPACE>
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to footswitch_driver
source install/setup.bash
```

## Run

```bash
ros2 run footswitch_driver footswitch_driver_node
```

## Interface

- Topic: `state` (relative to the node namespace; `/state` by default)
- Type: `footswitch_msgs/msg/FootswitchState`
- Field: `bool[3] state` (`state[0]`, `state[1]`, `state[2]` for each pedal)

Each changed HID report is published, including a press and release received
within one polling interval. Holding a pedal does not repeat its state message.
`update_rate` controls HID polling in Hz (default `10`, range `1`–`1000`).

State QoS is reliable, volatile, keep-last-10, with manual-by-topic liveliness.
The driver asserts liveliness after successful polling, even without a state
change. Its lease is three polling intervals (`0.3 s` at the default rate).
A USB read failure stops polling and liveliness assertions until restart.
Use the ROS liveliness-lost notification to request a safe stop on input loss.
For portable loss detection, request manual-by-topic liveliness with a finite
lease at least as long as the publisher's lease (e.g. `0.5 s` for default polling).
