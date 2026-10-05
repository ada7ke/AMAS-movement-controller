from controller_gui import ControllerOutput, ControllerGUI


class TemplateOutput(ControllerOutput):
    """Starting point for an output transport"""

    def send_controller_command(self, direction, speed, angle):
        """Send a command to the target device or simulator"""
        raise NotImplementedError("Implement send_controller_command for the selected output")


if __name__ == "__main__":
    ControllerGUI(output=TemplateOutput(), title="AMAS Movement Controller - Template").run()
