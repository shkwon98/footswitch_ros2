# ROS 2 Footswitch

ROS 2 driver and message interfaces for the 3-pedal PCsensor USB footswitch.

## Hardware

- Vendor ID: `0x3553`
- Product ID: `0xb001`

## Packages

- `footswitch_msgs`: Message definitions (`FootswitchState`)
- `footswitch_driver`: HID-based ROS 2 node publishing pedal states

## Build

```bash
cd ~/ros2_ws
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
