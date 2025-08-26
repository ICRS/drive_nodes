# Drive Node

The `drive` node receives `Twist` messages on the `drive/speed` topic:
- The field `linear.x` is the forward speed between -1 and 1, where 1 represents the maximum speed of the robot, and a negative value corresponds to moving backwards.
- The field `angular.z` is the turning speed between -1 and 1, where 1 represents the maximum turning speed of the robot, a positive value corresponds to a right turn, and a negative value corresponds to a left turn.

The data is then scaled between -255 and 255 and sent over serial in messages like `<LINEAR:-100>`, `<ANGULAR:43>`.

## Joystick

### Speed

The `drive_joy` node receives `Joy` messages and converts them to `Twist` messages for the drive node (described above):

* First axis = turn (left = positive, this is inverted)
* Second axis = forward (forwards = positive)
* Third axis = multiplier (-1 to 1, remapped onto 1x to 3x)

### Camera

It also posts a `Bool` to the `camera/capture` topic photos when the main (index finger) trigger is pressed:

* Button 1 (1 = pressed, 0 = released)

### Current limiting

The per-side current limit can be changed with the left and right thumb buttons:

* Button 3 (left) (1 = pressed, 0 = released): 10A current limit
* Button 4 (right) (1 = pressed, 0 = released): ~19A current limit (max)

Messages are sent over ROS2 in amps, and then converted to mA and sent over serial in the format `<I_LIMIT:10000>`.

### Start

To start the joy node, run the following command:

```bash
ros2 run joy joy_node --ros-args --remap __node:=drive_joy_node -r /joy:=/drive/joy
```