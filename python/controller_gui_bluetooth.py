import asyncio, threading
from bleak import BleakClient, BleakScanner

from controller_gui import ControllerGUI
from controller_gui_gazebo import GazeboControllerGUI
from controller_gui_robot_dog import RobotDogControllerGUI

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

        self.bad_checksums = 0
        self.packets_read = 0

        self.buffer = bytearray()
        self.new_packet = False

        self.connected = False
        self.running = True

        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self.bluetooth_thread, daemon=True)
        self.thread.start()

    def bluetooth_thread(self):
        asyncio.run(self.bluetooth_loop())

    async def bluetooth_loop(self):
        while self.running:
            try:
                print("searching for ESP32...")

                devices = await BleakScanner.discover(timeout=5.0, return_adv=True)

                for address, (device, adv) in devices.items():
                    print(f"name: {device.name} | address: {device.address}")
                    print(f"local name: {adv.local_name}")
                    print(f"service UUIDs: {adv.service_uuids}")
                    print()

                device = next((device for device, adv in devices.values() if device.name == DEVICE_NAME or adv.local_name == DEVICE_NAME), None)

                if device is None:
                    print(f"{DEVICE_NAME} not found")
                    await asyncio.sleep(1)
                    continue

                print(f"found target: {device.name}")

                async with BleakClient(device) as client:
                    self.connected = True

                    print("bluetooth connected")
                    print(f"MTU: {client.mtu_size}")

                    await client.start_notify(TX_UUID, self.notification_received)

                    while client.is_connected and self.running:
                        await asyncio.sleep(0.1)

            except Exception as error:
                print(f"bluetooth error: {error}")

            self.connected = False

            if self.running:
                print("bluetooth disconnected, reconnecting...")
                await asyncio.sleep(1)

    def notification_received(self, characteristic, data):
        with self.lock:
            self.buffer.extend(data)

    def update(self):
        with self.lock:
            while True:
                header_index = self.buffer.find(b"\xAA\x55")

                if header_index == -1:
                    if len(self.buffer) > 1:
                        self.buffer = self.buffer[-1:]

                    return False

                if header_index > 0:
                    del self.buffer[:header_index]

                if len(self.buffer) < 207:
                    return False

                packet = self.buffer[:207]
                del self.buffer[:207]

                calculated_checksum = sum(packet[:206]) & 0xFF
                received_checksum = packet[206]

                if calculated_checksum != received_checksum:
                    self.bad_checksums += 1
                    continue

                if packet[2] != 1:
                    continue

                index = 3

                for i in range(48):
                    low = packet[index]
                    high = packet[index + 1]
                    self.left_pressures[i] = low | (high << 8)
                    index += 2

                for i in range(48):
                    low = packet[index]
                    high = packet[index + 1]
                    self.right_pressures[i] = low | (high << 8)
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

                print(f"packet {self.packets_read} | angle: {self.encoder_angle} | position: {self.encoder_position} | center: {self.center_found}")
                print(f"A: {self.encoder_sensor_a} | B: {self.encoder_sensor_b} | C: {self.encoder_sensor_c}")
                print(f"left:  {self.left_pressures}")
                print(f"right: {self.right_pressures}")
                

                return True

    def close(self):
        self.running = False


class BluetoothControllerGUI(BaseControllerGUI):
    def __init__(self):
        receiver = BluetoothESP32Receiver()

        super().__init__(esp32_receiver=receiver)


if __name__ == "__main__":
    BluetoothControllerGUI().run()