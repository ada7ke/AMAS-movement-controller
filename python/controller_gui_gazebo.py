import socket
from controller_gui import ControllerGUI


class GazeboControllerGUI(ControllerGUI):
    ROBOT_IP = "172.17.149.227"
    ROBOT_PORT = 5005

    def __init__(self, esp32_receiver=None, title="AMAS Movement Controller - Gazebo"):
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        super().__init__(title=title, esp32_receiver=esp32_receiver)

    def send_controller_command(self, direction, speed, angle):
        message = f"{direction},{speed},{angle}"
        self.robot_socket.sendto(message.encode(), (self.ROBOT_IP, self.ROBOT_PORT))

    def close_output(self):
        self.robot_socket.close()


if __name__ == "__main__":
    GazeboControllerGUI().run()