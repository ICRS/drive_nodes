# Drive Node

This node receives `Twist` messages on the `drive/speed` topic:
- The field `linear.x` is the forward speed between -1 and 1, where 1 represents the maximum speed of the robot, and a negative value corresponds to moving backwards.
- The field `angular.z` is the turning speed between -1 and 1, where 1 represents the maximum turning speed of the robot, a positive value corresponds to a right turn, and a negative value corresponds to a left turn.

The data is then scaled between -255 and 255 and sent over serial in messages like `<LINEAR:-100>`, `<ANGULAR:43>`.

## Joystick

First axis = turn (left = positive)
Second axis = forward (forwards = positive)
