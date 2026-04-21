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

## udev Rule

`hidraw` access is usually restricted to `root`, so install the bundled udev rule before building or running the driver as a normal user.

```bash
cd <YOUR_ROS2_WORKSPACE>/src/footswitch_ros2
./scripts/install_udev_rules.sh
```

Unplug and reconnect the footswitch after applying the rule.

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

- Topic: `/footswitch_node/state`
- Type: `footswitch_msgs/msg/FootswitchState`
- Field: `bool[3] state` (`state[0]`, `state[1]`, `state[2]` for each pedal)
