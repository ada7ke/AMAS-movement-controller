import socket, math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


UDP_PORT = 5005

MOVE_SPEED = 0.6

KP = 2.0
MAX_ROTATION_SPEED = 1.0
ANGLE_TOLERANCE = 2.0


class GazeboReceiver(Node):
    def __init__(self):
        super().__init__("gazebo_receiver")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscriber = self.create_subscription(Odometry, "/model/mecanum_robot/odometry", self.odom_callback, 10)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", UDP_PORT))
        self.socket.setblocking(False)

        self.direction = "none"
        self.encoder_angle = 0.0

        self.current_yaw = None
        self.starting_yaw = None

        self.timer = self.create_timer(0.02, self.update)

        print(f"listening on UDP port {UDP_PORT}")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.starting_yaw is None:
            self.starting_yaw = self.current_yaw
            print(f"starting yaw: {math.degrees(self.starting_yaw):.1f}°")

    def update(self):
        try:
            while True:
                data, address = self.socket.recvfrom(1024)

                message = data.decode().strip()
                direction, angle = message.split(",")

                self.direction = direction
                self.encoder_angle = float(angle)

                print(f"received: {self.direction}, angle: {self.encoder_angle}")

        except BlockingIOError:
            pass

        msg = Twist()

        if self.direction == "forward":
            msg.linear.x = MOVE_SPEED
        elif self.direction == "backward":
            msg.linear.x = -MOVE_SPEED
        elif self.direction == "strafe_left":
            msg.linear.y = MOVE_SPEED
        elif self.direction == "strafe_right":
            msg.linear.y = -MOVE_SPEED

        if self.current_yaw is not None and self.starting_yaw is not None:
            # target_yaw = self.starting_yaw + math.radians(self.encoder_angle)
            target_yaw = math.radians(self.encoder_angle)

            error = target_yaw - self.current_yaw
            error = math.atan2(math.sin(error), math.cos(error))

            error_degrees = math.degrees(error)

            if abs(error_degrees) <= ANGLE_TOLERANCE:
                angular_speed = 0.0
            else:
                angular_speed = KP * error
                angular_speed = max(-MAX_ROTATION_SPEED, min(angular_speed, MAX_ROTATION_SPEED))

            msg.angular.z = angular_speed

        self.publisher.publish(msg)


def main():
    rclpy.init()

    node = GazeboReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()