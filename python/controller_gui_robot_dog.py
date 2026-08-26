import socket
from controller_gui import ControllerGUI


class RobotDogControllerGUI(ControllerGUI):
    ROBOT_IP = "127.0.0.1"
    ROBOT_PORT = 15005

    def __init__(self, esp32_receiver=None, title="AMAS Movement Controller - Robot Dog"):
        self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot_socket.connect((self.ROBOT_IP, self.ROBOT_PORT))
        super().__init__(title=title, esp32_receiver=esp32_receiver)

    def send_controller_command(self, direction, speed, angle):
        message = f"{direction},{speed},{angle}\n"
        self.robot_socket.sendall(message.encode())

    def close_output(self):
        self.robot_socket.close()


if __name__ == "__main__":
    RobotDogControllerGUI().run()
