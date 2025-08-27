import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
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
    PATTERN_PONG      = re.compile(r"<PONG:(\d+)>")

    def __init__(self):
        super().__init__('drive')

        # Setup ROS nodes
        self.drive_sub = self.create_subscription(Twist, "drive/speed", self.update_speed, 10)
        self.l_current_pub = self.create_publisher(Float32,"drive/l_current", 10)
        self.r_current_pub = self.create_publisher(Float32,"drive/r_current", 10)

        # Setup serial
        ports = sorted(glob.glob('/dev/ttyUSB*'))
        if not ports:
            self.get_logger().error("No /dev/ttyACM* devices found.")
            raise RuntimeError("No ACM serial device found.")
        self.serial = None

        # Get the port
        connected = False
        while not connected:
            for port in ports:
                try:
                    self.get_logger().info(f"Trying serial port: {port}")
                    self.serial = serial.Serial(port, 115200, timeout=1)
                    time.sleep(2)  # give device time to reset if needed
                    self.get_logger().info(f"Connected to {port}")
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to open {port}: {e}")

                # Check the serial port is okay
                if self.serial is None:
                    self.get_logger().info("Couldn't get serial port")
                    time.sleep(1)
                    continue

                # Ping the device to check it responds and has the correct ID
                connected = self.serial_ping()

                # Invalid device: close the connection and try the next device
                if not connected:
                    self.serial.close()
                    continue

                # Valid device: move on
                else:
                    break

        # Start the thread to read serial messages from the ESP32
        self.serial_read_thread = threading.Thread(target=self.serial_read, daemon=True)
        self.serial_read_thread.start()

        self.current_limit_sub = self.create_subscription(Float32, "drive/current_limit", self.update_current_limit, 10)

    def serial_ping(self) -> bool:

        buffer = ""
        start_time = time.time()
        pong_received = False
        while not pong_received:

            # Timeout
            if (time.time() > (start_time + 3.0)) and not pong_received:
                break

            # Send PING
            out = "<PING:1>\n"
            out_bytes = bytes(out.encode("utf-8"))
            try:
                self.serial.write(out_bytes)
                self.get_logger().info("Sent PING")
            except serial.serialutil.SerialException:
                continue

            time.sleep(0.2)

            # Wait for PONG
            if self.serial.in_waiting:
                buffer += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    # Check if the message is a PONG
                    m = DriveNode.PATTERN_PONG.match(line)
                    if m is not None:

                        device_id = int(m.group(1))

                        # PONG contained invalid ID
                        if device_id != 69:
                            self.get_logger().info(f"Received PONG, incorrect ID: '{device_id}'")
                            return pong_received

                        # PONG containted correct ID
                        else:
                            pong_received = True
                            self.get_logger().info(f"Received PONG, correct ID: '{device_id}'")
                            return pong_received

        return pong_received

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

                    self.get_logger().info(f"Drive board: '{line}'")

    def update_speed(self, msg: Twist):

        # Constrain the inputs
        linear_velocity = constrain(float(msg.linear.x), -1, 1)
        angular_velocity = constrain(float(msg.angular.z), -1, 1)
        self.get_logger().info(f"Linear {linear_velocity}, turning {angular_velocity}")

        # Convert into the data that will be sent over the wire
        linear = int(constrain(linear_velocity * 255, -255, 255))
        angular = int(constrain(angular_velocity * 255, -255, 255))

        # Create and send the message
        out = f"<LINEAR:{linear}>\n"
        out += f"<ANGULAR:{angular}>\n"
        self.get_logger().info(f"\nSending:\n{out}\n")
        out_bytes = bytes(out.encode("utf-8"))
        self.serial.write(out_bytes)

    def update_current_limit(self, msg: Float32):

        # Convert the current limit to mA:
        current_limit = int(abs(msg.data * 1000.0))
        current_limit = constrain(current_limit, 0, 18_000)  # Maximum current limit is 18A (sensors measure up to 20A, but if that was the limit then we can't detect if its been crossed). Also measurements have uncertainty

        # Create and send the message
        out = f"<I_LIMIT:{current_limit}>\n"
        self.get_logger().info(f"\nSending:\n{out}\n")
        out_bytes = bytes(out.encode("utf-8"))
        self.serial.write(out_bytes)

    def destroy_node(self):
        self.serial.close()
        super().destroy_node()


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
