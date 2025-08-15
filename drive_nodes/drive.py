import rclpy
from rclpy.node import Node
from std_msgs.msg import Twist, Float32
import serial
import time
import re
import glob
import threading


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


class DriveNode(Node):

    PATTERN_L_CURRENT = re.compile(r"<L_CURRENT:(\d+)>")
    PATTERN_R_CURRENT = re.compile(r"<R_CURRENT:(\d+)>")

    def __init__(self):
        super().__init__('drive')

        # Setup ROS nodes
        self.drive_sub = self.create_subscription(Twist, "drive/speed", self.update_speed, 10)
        self.l_current_pub = self.create_publisher(Float32,"drive/l_current", 10)
        self.r_current_pub = self.create_publisher(Float32,"drive/r_current", 10)

        # Setup serial
        ports = sorted(glob.glob('/dev/ttyACM*'))
        if not ports:
            self.get_logger().error("No /dev/ttyACM* devices found.")
            raise RuntimeError("No ACM serial device found.")
        self.serial = None
        for port in ports:
            try:
                self.get_logger().info(f"Trying serial port: {port}")
                self.serial = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)  # give device time to reset if needed
                self.get_logger().info(f"Connected to {port}")
                break
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to open {port}: {e}")

        # Start the thread to read serial messages from the ESP32
        self.serial_read_thread = threading.Thread(target=self.serial_read, daemon=True)
        self.serial_read_thread.start()

    def serial_read(self):
        buffer = ""
        while rclpy.ok():
            if self.serial.in_waiting:
                buffer += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    m = DriveNode.PATTERN_L_CURRENT.match(line)
                    if m is not None:
                        msg = Float32()
                        msg.data = int(m.group(1)) / 1000  # mA to A
                        self.l_current_pub.publish(msg)
                        continue

                    m = DriveNode.PATTERN_R_CURRENT.match(line)
                    if m is not None:
                        msg = Float32()
                        msg.data = int(m.group(1)) / 1000  # mA to A
                        self.r_current_pub.publish(msg)
                        continue

    def update_speed(self, msg: Twist):

        # Constrain the inputs
        linear_velocity = constrain(float(msg.linear.x.data), -1, 1)
        angular_velocity = constrain(float(msg.angular.z.data), 1, 1)
        self.get_logger().info(f"Linear {linear_velocity}, turning {angular_velocity}")

        # Convert into the data that will be sent over the wire
        linear = int(constrain(linear_velocity * 255, -255, 255))
        angular = int(constrain(linear_velocity * 255, -255, 255))

        # Create and send the message
        out = f"<LINEAR:{linear}>\n"
        out += f"<ANGULAR:{angular}>\n"
        out_bytes = bytes(out.encode("utf-8"))
        self.serial.write(out_bytes)


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
