# footswitch_ros2

ROS 2 driver and pedal-to-topic mapper for the PCsensor three-pedal USB
footswitch with vendor ID `0x3553` and product ID `0xb001`.

## Install

Place this repository at `src/footswitch_ros2` in a ROS 2 Jazzy workspace.
Run these commands from the workspace root.

```bash
src/footswitch_ros2/scripts/install_udev_rules.sh
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/footswitch_ros2 --ignore-src -r -y
colcon build --packages-up-to footswitch_topic_mapper
source install/setup.bash
```

Reconnect the USB device after installing the udev rule. The rule grants access
to the active local session. While the driver runs, pedal keys are withheld from
desktop applications.

### Configure the pedal keys

| Pedal | Accepted device key |
| --- | --- |
| Left | A or 1 |
| Middle | B or 2 |
| Right | C or 3 |

Number-row and keypad digits both work. Use a single keyboard key per pedal.
The driver ignores mouse actions, media keys, and other key assignments.
Save the assignments to the device. Disable any equipment that uses the
pedals before verifying the keys in a text editor.
[PCsensor configuration software](https://pcsensor.com/download/)

## Run

The mapper launch starts the HID driver and publishes `std_msgs/msg/Bool` on
the configured topics. This example sets `/fixture/enabled` to true on left
press and to false on middle release.

```bash
ros2 launch footswitch_topic_mapper mapper.launch.py \
  left_binding:=press:true:/fixture/enabled \
  middle_binding:=release:false:/fixture/enabled
```

Set `left_binding`, `middle_binding`, and `right_binding` as
`<press|release>:<true|false>:</absolute/topic>`. Each binding defaults to empty,
which disables that pedal's output. Use `launch_driver:=false` to supply recorded
or synthetic pedal states to the mapper.

For raw pedal states, run the driver directly.

```bash
ros2 run footswitch_driver footswitch_driver_node
```

## ROS interfaces

The mapper publishes one `Bool` per matching transition. Output QoS is
reliable, volatile, keep-last-one.
When bindings share a topic, false wins on simultaneous transitions. A held
pedal with a false binding blocks true transitions on that topic. Loss of
driver liveliness after a valid baseline sends one `false` to each mapped topic.
After startup or input loss, a valid report with all pedals up establishes the
baseline. If the device reports only on changes, the first release can establish
that baseline.

The driver publishes `footswitch_msgs/msg/FootswitchState` on the relative
`state` topic, or `/state` with no namespace. The mapper launch remaps this
internal link to `/footswitch/_state`. The state array lists left, middle, and
right pedals in order. The driver publishes the first valid HID report and
subsequent changes, including transitions within one poll interval.
`update_rate` is the HID polling rate
in Hz. It defaults to 10 and accepts 1 through 1000.

Raw state QoS is reliable, volatile, keep-last-10 with manual-by-topic
liveliness. The driver asserts liveliness after each successful poll.
If the USB device is absent, the driver waits and retries the connection
every second. On a read failure it closes the device and removes the raw state
publisher. It creates a new publisher when the device reconnects. The publisher's
lease is three poll intervals. The mapper uses a 0.5 s lease and checks the
report timestamp against its ROS clock.

## License

[BSD-3-Clause](LICENSE).
