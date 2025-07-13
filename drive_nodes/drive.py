import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time
import re


class JoyToSerial(Node):
    def __init__(self):
        super().__init__('joy_to_serial_node')

        # Setup serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # Subscribe to /joy
        self.timer = self.create_timer(0.5,self.read_serial_loop)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def scale(self, value, src_min, src_max, dst_min, dst_max):
        src_range = src_max - src_min
        dst_range = dst_max - dst_min
        value_scaled = (value - src_min) / src_range
        return dst_min + (value_scaled * dst_range)

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def apply_deadzone(self, value, threshold=0.1):
        return 0.0 if abs(value) < threshold else value

    def cubic_scaling(self, value, gain=1.0):
        return gain * (value * abs(value))

    def joy_callback(self, msg: Joy):
        if len(msg.axes) < 2:
            self.get_logger().warn("Not enough joystick axes received.")
            return

        axis_0 = msg.axes[0]  # Turn
        axis_1 = msg.axes[1]  # Throttle

        throttle = -axis_1
        turn = axis_0

        throttle = self.apply_deadzone(throttle)
        turn = self.apply_deadzone(turn)

        throttle = self.cubic_scaling(throttle, gain=1.0)
        turn = self.cubic_scaling(turn, gain=0.8)

        left_motor = throttle + turn
        right_motor = throttle - turn

        max_mag = max(abs(left_motor), abs(right_motor), 1.0)
        left_motor /= max_mag
        right_motor /= max_mag

        left_pwm = int(self.scale(left_motor, -1, 1, -255, 255))
        right_pwm = int(self.scale(right_motor, -1, 1, -255, 255))

        left_pwm = self.clamp(left_pwm, -255, 255)
        right_pwm = self.clamp(right_pwm, -255, 255)

        msg_l = f"<MOTOR_L: {left_pwm}>\n"
        msg_r = f"<MOTOR_R: {right_pwm}>\n"

        try:
            self.serial_port.write(msg_l.encode('utf-8'))
            self.serial_port.write(msg_r.encode('utf-8'))
            # self.get_logger().info(f"Sent: {msg_l.strip()} {msg_r.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    
    def read_serial_loop(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            self.serial_port.flushInput()
            if not line:
                return

            self.get_logger().info(f"{line}")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def destroy_node(self):
        super().destroy_node()
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    node = JoyToSerial()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
