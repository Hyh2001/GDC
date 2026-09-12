# Recorder

`recorder` provides a service-controlled ROS 2 bag recorder and a separate
joystick adapter. The recorder node has no dependency on joystick behavior;
any input device can control it through `std_srvs/srv/SetBool`.

## Build

```bash
colcon build --packages-select recorder
source install/setup.bash
```

## Joystick operation

Edit `config/joy_recorder.yaml`, or copy it and pass the new path:

```bash
ros2 launch recorder joy_recorder.launch.py config:=/absolute/path/config.yaml
```

A rising edge on `record_button` starts recording. Release and press the same
button again to stop and finalize the bag. The adapter follows the recorder's
published state rather than maintaining an independent recording state.

If `topics` is empty, the recorder captures every discovered non-hidden topic.
Otherwise, it records only the fully qualified topics in the list. Each session
is stored below `output_directory` as
`<bag_name>_<YYYYMMDDTHHMMSSffffffZ>` using UTC system time.

## Input-independent control

The recorder executable can be started directly with ROS parameters:

```bash
ros2 run recorder recorder_node --ros-args \
  -p bag_name:=experiment \
  -p output_directory:=~/rosbags \
  -p topics:="[/joy, /robot/state]"
```

Start and stop it from any ROS node or from the command line:

```bash
ros2 service call /recorder/set_recording std_srvs/srv/SetBool "{data: true}"
ros2 service call /recorder/set_recording std_srvs/srv/SetBool "{data: false}"
```

The transient-local `/recorder/recording` `std_msgs/msg/Bool` topic reports the
current state. Repeating the current requested state is safe and succeeds
without creating or stopping another session.

## Notes

- Bags use MCAP storage without additional compression or splitting.
- `/joy` is recorded only when it is explicitly listed or all-topic recording
  is enabled.
- Select a joystick button that is not mapped to another action unless both
  actions are intended.
