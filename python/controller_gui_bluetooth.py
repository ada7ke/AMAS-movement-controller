import asyncio, queue, threading
from bleak import BleakClient, BleakScanner

from controller_gui import ControllerGUI
from python.gazebo_output import GazeboControllerGUI
from python.robot_dog_output import RobotDogControllerGUI

COMMAND_OUTPUT = ""
OUTPUT_CLASSES = {
    "": ControllerGUI,    
    "Gazebo": GazeboControllerGUI,
    "RobotDog": RobotDogControllerGUI
}
BaseControllerGUI = OUTPUT_CLASSES[COMMAND_OUTPUT]

DEVICE_NAME = "AMAS_Pedal_Controller"
SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class BluetoothESP32Receiver:
    def __init__(self):
        self.left_pressures = [0] * 48
        self.right_pressures = [0] * 48

        self.encoder_sensor_a = 0
        self.encoder_sensor_b = 0
        self.encoder_sensor_c = 0

        self.encoder_position = 0
        self.encoder_angle = 0
        self.center_found = False
        self.emergency_brake_pressed = False

        self.bad_checksums = 0
        self.packets_read = 0

        self.buffer = bytearray()
        self.new_packet = False
        self.messages = queue.Queue()

        self.connected = False
        self.running = True

        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self.bluetooth_thread, daemon=True)
        self.thread.start()

    def log(self, message):
        print(message)
        self.messages.put(message)

    def bluetooth_thread(self):
        asyncio.run(self.bluetooth_loop())

    async def bluetooth_loop(self):
        while self.running:
            try:
                self.log("searching for ESP32...")

                devices = await BleakScanner.discover(timeout=5.0, return_adv=True)

                device = next((device for device, adv in devices.values() if device.name == DEVICE_NAME or adv.local_name == DEVICE_NAME), None)

                if device is None:
                    self.log(f"{DEVICE_NAME} not found")
                    await asyncio.sleep(1)
                    continue

                self.log(f"found target: {device.name}, {device.address}")

                async with BleakClient(device) as client:
                    self.connected = True

                    self.log("bluetooth connected")
                    self.log(f"MTU: {client.mtu_size}")

                    await client.start_notify(TX_UUID, self.notification_received)

                    while client.is_connected and self.running:
                        await asyncio.sleep(0.1)

            except Exception as error:
                self.log(f"bluetooth error: {error}")

            self.connected = False

            if self.running:
                self.log("bluetooth disconnected, reconnecting...")
                await asyncio.sleep(1)

    def notification_received(self, characteristic, data):
        with self.lock:
            self.buffer.extend(data)

    def update(self):
        with self.lock:
            latest_packet = None

            while True:
                header_index = self.buffer.find(b"\xAA\x55")

                if header_index == -1:
                    if len(self.buffer) > 1:
                        self.buffer = self.buffer[-1:]
                    break

                if header_index > 0:
                    del self.buffer[:header_index]

                if len(self.buffer) < 207:
                    break

                packet = bytes(self.buffer[:207])
                del self.buffer[:207]

                calculated_checksum = sum(packet[:206]) & 0xFF
                received_checksum = packet[206]

                if calculated_checksum != received_checksum:
                    self.bad_checksums += 1
                    continue

                if packet[2] & 0x7F != 1:
                    continue

                latest_packet = packet

            if latest_packet is None:
                return False

            packet = latest_packet

            self.emergency_brake_pressed = bool(packet[2] & 0x80)

            index = 3

            for i in range(48):
                self.left_pressures[i] = packet[index] | (packet[index + 1] << 8)
                index += 2

            for i in range(48):
                self.right_pressures[i] = packet[index] | (packet[index + 1] << 8)
                index += 2

            self.encoder_position = int.from_bytes(packet[index:index + 2], byteorder="little", signed=True)
            index += 2
            self.encoder_angle = int.from_bytes(packet[index:index + 2], byteorder="little", signed=True)
            index += 2
            self.center_found = bool(packet[index])
            index += 1
            self.encoder_sensor_a = int.from_bytes(packet[index:index + 2], byteorder="little")
            index += 2
            self.encoder_sensor_b = int.from_bytes(packet[index:index + 2], byteorder="little")
            index += 2
            self.encoder_sensor_c = int.from_bytes(packet[index:index + 2], byteorder="little")

            self.packets_read += 1

            return True

    def close(self):
        self.running = False

class BluetoothControllerGUI(BaseControllerGUI):
    def __init__(self):
        receiver = BluetoothESP32Receiver()

        super().__init__(esp32_receiver=receiver)

if __name__ == "__main__":
    BluetoothControllerGUI().run()