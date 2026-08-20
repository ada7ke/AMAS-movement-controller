import socket, math, time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


TCP_PORT = 5005

MAX_LINEAR_SPEED = 0.2657205
MAX_ROTATION_SPEED = 0.531441

KP = 2.0
ANGLE_TOLERANCE = 2.0

COMMAND_TIMEOUT = 0.25
UPDATE_PERIOD = 0.02


class PedalReceiver(Node):
    def __init__(self):
        super().__init__("pedal_receiver")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", TCP_PORT))
        self.server.listen(1)
        self.server.setblocking(False)

        self.client = None
        self.buffer = ""

        self.direction = "none"
        self.speed = 0.0
        self.encoder_angle = 0.0

        self.last_command_time = time.time()
        self.watchdog_active = True

        self.timer = self.create_timer(UPDATE_PERIOD, self.update)

        self.get_logger().info(f"Listening for pedal on TCP port {TCP_PORT}")

    def update(self):
        if self.client is not None:
            if time.time() - self.last_command_time > COMMAND_TIMEOUT:
                self.publish_stop()

                if not self.watchdog_active:
                    self.watchdog_active = True
                    self.get_logger().warning("Pedal command timeout, robot stopped")

        if self.client is None:
            try:
                self.client, address = self.server.accept()
                self.client.setblocking(False)

                self.buffer = ""
                self.last_command_time = time.time()
                self.watchdog_active = True

                self.get_logger().info(f"Pedal connected: {address}")

            except BlockingIOError:
                return

        try:
            data = self.client.recv(1024)

            if not data:
                self.disconnect()
                return

            self.buffer += data.decode()

            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)

                if line.strip():
                    self.process_command(line)

        except BlockingIOError:
            pass

        except (ConnectionResetError, BrokenPipeError):
            self.disconnect()

        except Exception as error:
            self.get_logger().error(f"Socket error: {error}")
            self.disconnect()

    def process_command(self, line):
        try:
            direction, speed, angle = line.strip().split(",")

            speed = float(speed)
            angle = float(angle)

            speed = max(0.0, min(speed, 1.0))

            self.direction = direction
            self.speed = speed
            self.encoder_angle = angle

            self.last_command_time = time.time()
            self.watchdog_active = False

            msg = Twist()

            linear_speed = speed * MAX_LINEAR_SPEED

            if direction == "forward":
                msg.linear.x = linear_speed

            elif direction == "backward":
                msg.linear.x = -linear_speed

            elif direction == "strafe_left":
                msg.linear.y = linear_speed

            elif direction == "strafe_right":
                msg.linear.y = -linear_speed

            elif direction == "none":
                msg.linear.x = 0.0
                msg.linear.y = 0.0

            else:
                self.get_logger().warning(f"Unknown direction: {direction}")
                self.publish_stop()
                return

            angle_error = ((angle + 180.0) % 360.0) - 180.0

            if abs(angle_error) > ANGLE_TOLERANCE:
                rotation = -math.radians(angle_error) * KP
                rotation = max(-MAX_ROTATION_SPEED, min(MAX_ROTATION_SPEED, rotation))
                msg.angular.z = rotation
            else:
                msg.angular.z = 0.0

            self.publisher.publish(msg)

        except ValueError:
            self.get_logger().warning(f"Invalid command: {line}")

    def publish_stop(self):
        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher.publish(msg)

        self.direction = "none"
        self.speed = 0.0

    def disconnect(self):
        self.publish_stop()

        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass

        self.client = None
        self.buffer = ""
        self.watchdog_active = True

        self.get_logger().warning("Pedal disconnected, robot stopped")


def main():
    rclpy.init()

    node = PedalReceiver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_stop()

        if node.client is not None:
            node.client.close()

        node.server.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
