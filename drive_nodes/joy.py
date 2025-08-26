import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import serial
import time
import re
import glob
import threading


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


class JoyNode(Node):

    def __init__(self):
        super().__init__('drive_joy')

        # Setup ROS nodes
        self.drive_pub = self.create_publisher(Twist, "drive/speed", 10)
        self.joy_sub = self.create_subscription(Joy,"drive/joy", self.joy_receive, 10)

    def joy_receive(self, rx_msg: Joy):

        tx_msg = Twist()

        tx_msg.angular.z = constrain(-rx_msg.axes[0], -1.0, 1.0)
        tx_msg.linear.x = constrain(rx_msg.axes[1], -1.0, 1.0)

        self.drive_pub.publish(tx_msg)


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
