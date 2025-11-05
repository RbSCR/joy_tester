# joy_tester

This is a very simple GUI program for testing joysticks in ROS.
More specifically, it displays the messages of a `sensor_msgs/Joy` topic in a more user-friendly format.

Additionally there is a ROS node to send 'rumble feedback' messages to the joystick.

## Usage

### test_joy

Start the controller node:

```bash
ros2 run joy joy_node
```

Start the GUI:

```bash
ros2 run joy_tester test_joy
```

By default it subscribes to the `/joy` topic, but this can be remapped as with any other ROS node (e.g. with `--ros-args -r joy:=other_joy`).

Particularly helpful (compared to a `topic echo`) is that it displays the axis/button numbers which saves time and errors in counting them manually.

### send_rumble_feedback

Start the rumble feedback node:

```bash
ros2 run joy_tester send_rumble_feedback
```

The node sends a rumble feedback every 5 seconds.
Intensity of the rumble can be changed by changing the parameter 'intensity', default value is 0.4; range is 0.0 - 1.0.

```bash
ros2 param set rumble_feedback intensity <(double)value>
```

## To Do

- ~~Clean up the code, particularly GUI element placement (was a very rough first go)~~
- Add functionality for sending `sensor_msgs/JoyFeedback` (e.g. LED, rumble) back to controllers
- Some way to assist in calibration?
