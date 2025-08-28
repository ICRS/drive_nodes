import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import serial
import time
import re
import glob
import threading
from math import e


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


class JoyNode(Node):

    def __init__(self):
        super().__init__('drive_joy')

        # Setup ROS nodes
        self.drive_pub = self.create_publisher(Twist, "drive/speed", 10)
        self.joy_sub = self.create_subscription(Joy, "drive/joy", self.joy_receive, 10)

        self.camera_pub = self.create_publisher(Bool, "camera/capture", 10)
        self.previous_camera_trigger = False

        self.current_limit_pub = self.create_publisher(Float32, "drive/current_limit", 10)
        self.current_limit_changed = False

    def joy_receive(self, rx_msg: Joy):

        ############ Send drive control signals ############

        tx_msg = Twist()
        multiplier = ((1 - rx_msg.axes[2]) * (2/5) + 0.2)  # Remap -1 to 1 into 1 to 0.2

        # Set the angular velocity
        sign = -1.0 if rx_msg.axes[0] > 0.0 else 1.0
        if abs(rx_msg.axes[0]) < 0.07:  # Deadzone
            tx_msg.angular.z = 0.0
        else:
            tx_msg.angular.z = constrain(multiplier * e**(3.0 * abs(rx_msg.axes[0])) / 19, -1.0, 1.0)
            tx_msg.angular.z = sign * tx_msg.angular.z

        # Set the linear velocity
        sign = 1.0 if rx_msg.axes[1] > 0.0 else -1.0

        if abs(rx_msg.axes[1]) < 0.07: # Deadzone
            tx_msg.linear.x = 0.0
        else:
            tx_msg.linear.x = constrain(multiplier * e**(3.0 * abs(rx_msg.axes[1])) / 19, -1.0, 1.0)
            tx_msg.linear.x = sign * tx_msg.linear.x

        self.drive_pub.publish(tx_msg)

        ############ Send the camera capture message ############

        current_camera_trigger = bool(rx_msg.buttons[0])

        if current_camera_trigger and not self.previous_camera_trigger:
            tx_msg = Bool()
            tx_msg.data = True
            self.camera_pub.publish(tx_msg)

        self.previous_camera_trigger = current_camera_trigger

        ############ Send the updated current limit ############

        new_current_limit_change = bool(rx_msg.buttons[2]) or bool(rx_msg.buttons[3])

        if new_current_limit_change and not self.current_limit_changed:
            tx_msg = Float32()
            if rx_msg.buttons[2]:
                tx_msg.data = 10.0
            else:
                tx_msg.data = 18.0
            self.current_limit_pub.publish(tx_msg)

        self.current_limit_changed = new_current_limit_change


def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
