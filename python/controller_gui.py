import serial, time, csv, joblib, math
import tkinter as tk
import pandas as pd
import common as cmn

class ESP32Receiver:
    def __init__(self, port, baud_rate=921600):
        self.serial = serial.Serial(port, baud_rate, timeout=0)
        self.buffer = bytearray()

        self.bad_checksums = 0
        self.packets_read = 0

        self.left_pressures = [0] * 48
        self.right_pressures = [0] * 48

        self.encoder_sensor_a = 0
        self.encoder_sensor_b = 0
        self.encoder_sensor_c = 0

        self.encoder_position = 0
        self.encoder_angle = 0
        self.center_found = False

    def update(self):
        if self.serial.in_waiting > 0:
            self.buffer.extend(self.serial.read(self.serial.in_waiting))

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

            next_header = self.buffer.find(b"\xAA\x55", 2)

            if next_header != -1 and len(self.buffer) - next_header >= 207:
                del self.buffer[:next_header]
                continue

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
            index += 2

            self.packets_read += 1
            return True

    def close(self):
        self.serial.close()

class FootGrid:
    def __init__(self, parent, name, pressures, sensor_numbers):
        self.pressures = pressures
        self.sensor_numbers = sensor_numbers
        self.cells = {}

        frame = tk.LabelFrame(parent, text=name, font=("Arial", 16, "bold"), padx=10, pady=10)
        frame.pack(side=tk.LEFT, padx=20, pady=20)

        for row in range(12):
            for col in range(4):
                sensor_number = sensor_numbers[row][col]
                cell = tk.Label(frame, text=f"{sensor_number}\n0", width=6, height=2, font=("Arial", 11, "bold"), relief="solid", borderwidth=1, bg="white")
                cell.grid(row=row, column=col, padx=2, pady=2)
                self.cells[sensor_number] = cell

    def pressure_to_color(self, pressure):
        max_pressure = 200
        pressure = max(0, min(pressure, max_pressure))

        intensity = pressure / max_pressure
        red = 255
        green = int(255 * (1 - intensity))
        blue = int(255 * (1 - intensity))

        return f"#{red:02x}{green:02x}{blue:02x}"

    def update(self):
        for sensor_number, cell in self.cells.items():
            value = self.pressures[sensor_number - 1]
            cell.config(text=f"{value}", bg=self.pressure_to_color(value))

class ControllerGUI:
    CONFIDENCE_THRESHOLD = 0.70
    ACCELERATION_RATE = 1.0
    DECELERATION_RATE = 1.5
    ESP32_TIMEOUT = 0.25

    left_indexes = [
        [12, 24, 36, 48],
        [11, 23, 35, 47],
        [10, 22, 34, 46],
        [9, 21, 33, 45],
        [8, 20, 32, 44],
        [7, 19, 31, 43],
        [6, 18, 30, 42],
        [5, 17, 29, 41],
        [4, 16, 28, 40],
        [3, 15, 27, 39],
        [2, 14, 26, 38],
        [1, 13, 25, 37]
    ]

    right_indexes = [
        [48, 36, 24, 12],
        [47, 35, 23, 11],
        [46, 34, 22, 10],
        [45, 33, 21, 9],
        [44, 32, 20, 8],
        [43, 31, 19, 7],
        [42, 30, 18, 6],
        [41, 29, 17, 5],
        [40, 28, 16, 4],
        [39, 27, 15, 3],
        [38, 26, 14, 2],
        [37, 25, 13, 1]
    ]

    def __init__(self, esp32_port="COM3", title="AMAS Movement Controller", esp32_receiver=None):
        if esp32_receiver is None:
            self.esp32 = ESP32Receiver(esp32_port)
        else:
            self.esp32 = esp32_receiver

        self.dir_data_file = f"datasets/pressure_training({cmn.dir_dataset}).csv"
        self.dir_model_file = f"models/pressure_svm({cmn.dir_dataset}).pkl"
        self.spd_data_file = f"datasets/speed_training({cmn.spd_dataset}).csv"
        self.spd_model_files = {
            "forward": f"models/fwd_spd_svr({cmn.spd_dataset}).pkl",
            "backward": f"models/bwd_spd_svr({cmn.spd_dataset}).pkl",
            "strafe_left": f"models/sl_spd_svr({cmn.spd_dataset}).pkl",
            "strafe_right": f"models/sr_spd_svr({cmn.spd_dataset}).pkl"
        }

        self.dir_model = None
        self.speed_models = {"forward": None, "backward": None, "strafe_left": None, "strafe_right": None}
        self.load_models()

        self.prediction = None
        self.current_speed = 0.0
        self.last_speed_update = time.time()
        self.movement_direction = "none"
        self.last_esp32_packet_time = time.time()
        self.esp32_watchdog_active = False
        self.last_stats_print = time.time()

        self.root = tk.Tk()
        self.root.title(title)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.create_gui()
        self.root.after(1, self.update_interface)

    def load_models(self):
        try:
            self.dir_model = joblib.load(self.dir_model_file)
            for direction, model_file in self.spd_model_files.items():
                self.speed_models[direction] = joblib.load(model_file)
        except FileNotFoundError:
            print("one or more models not found")

    def create_gui(self):
        grids_frame = tk.Frame(self.root)
        grids_frame.pack(padx=(10, 10))

        dir_btn_frame = tk.Frame(self.root)
        dir_btn_frame.pack(pady=(0, 10))

        spd_btn_frame = tk.Frame(self.root)
        spd_btn_frame.pack(pady=(0, 10))

        self.left_grid = FootGrid(grids_frame, "Left Foot", self.esp32.left_pressures, self.left_indexes)
        self.right_grid = FootGrid(grids_frame, "Right Foot", self.esp32.right_pressures, self.right_indexes)

        btn_width = 10
        tk.Button(dir_btn_frame, text="forward", width=btn_width, command=lambda: self.save_dir_sample("forward")).pack(side=tk.LEFT, padx=5)
        tk.Button(dir_btn_frame, text="backward", width=btn_width, command=lambda: self.save_dir_sample("backward")).pack(side=tk.LEFT, padx=5)
        tk.Button(dir_btn_frame, text="strafe left", width=btn_width, command=lambda: self.save_dir_sample("strafe_left")).pack(side=tk.LEFT, padx=5)
        tk.Button(dir_btn_frame, text="strafe right", width=btn_width, command=lambda: self.save_dir_sample("strafe_right")).pack(side=tk.LEFT, padx=5)
        tk.Button(dir_btn_frame, text="none", width=btn_width, command=lambda: self.save_dir_sample("none")).pack(side=tk.LEFT, padx=5)
        tk.Button(dir_btn_frame, text="undo", width=btn_width, command=lambda: self.undo_last_sample("direction")).pack(side=tk.LEFT, padx=5)

        tk.Button(spd_btn_frame, text="0%", width=btn_width, command=lambda: self.save_speed_sample(0.0)).pack(side=tk.LEFT, padx=5)
        tk.Button(spd_btn_frame, text="50%", width=btn_width, command=lambda: self.save_speed_sample(0.5)).pack(side=tk.LEFT, padx=5)
        tk.Button(spd_btn_frame, text="100%", width=btn_width, command=lambda: self.save_speed_sample(1.0)).pack(side=tk.LEFT, padx=5)
        tk.Button(spd_btn_frame, text="undo", width=btn_width, command=lambda: self.undo_last_sample("speed")).pack(side=tk.LEFT, padx=5)

        controls_frame = tk.Frame(self.root)
        controls_frame.pack(fill="both", padx=20, pady=(0, 20))
        controls_frame.grid_columnconfigure(0, weight=1)
        controls_frame.grid_columnconfigure(1, weight=0, minsize=300)
        controls_frame.grid_rowconfigure(0, weight=0)
        controls_frame.grid_rowconfigure(1, weight=0)

        prediction_frame = tk.LabelFrame(controls_frame, text="Prediction", font=("Arial", 14, "bold"), padx=10, pady=3)
        prediction_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 5))
        self.prediction_var = tk.StringVar()
        self.prediction_var.set("waiting...")
        prediction_label = tk.Label(prediction_frame, textvariable=self.prediction_var, font=("Arial", 20, "bold"), anchor="center", relief="flat", padx=8, pady=1)
        prediction_label.pack(fill="both", expand=True)

        speed_frame = tk.LabelFrame(controls_frame, text="Speed", font=("Arial", 14, "bold"), padx=10, pady=3)
        speed_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 5), pady=(5, 0))
        self.speed_canvas = tk.Canvas(speed_frame, height=60)
        self.speed_canvas.pack(fill="both", expand=True)
        self.speed_bar_bg = self.speed_canvas.create_rectangle(5, 5, 595, 55, fill="white", outline="black")
        self.speed_bar_fill = self.speed_canvas.create_rectangle(5, 5, 5, 55, fill="red", outline="")
        self.speed_text = self.speed_canvas.create_text(300, 30, text="0%", font=("Arial", 16, "bold"))

        self.gauge_width = 260
        self.gauge_height = 150
        self.gauge_center_x = self.gauge_width // 2
        self.gauge_center_y = 110
        self.gauge_radius = 90

        angle_frame = tk.LabelFrame(controls_frame, text="Angle", font=("Arial", 14, "bold"), padx=10, pady=3)
        angle_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(5, 0))
        self.angle_canvas = tk.Canvas(angle_frame, width=self.gauge_width, height=self.gauge_height)
        self.angle_canvas.pack(expand=True)
        self.angle_canvas.create_arc(self.gauge_center_x - self.gauge_radius, self.gauge_center_y - self.gauge_radius, self.gauge_center_x + self.gauge_radius, self.gauge_center_y + self.gauge_radius, start=0, extent=180, style=tk.ARC, width=5)

        for tick_angle in [-90, -45, 0, 45, 90]:
            radians = math.radians(tick_angle)
            x1 = self.gauge_center_x + math.sin(radians) * (self.gauge_radius - 8)
            y1 = self.gauge_center_y - math.cos(radians) * (self.gauge_radius - 8)
            x2 = self.gauge_center_x + math.sin(radians) * (self.gauge_radius + 8)
            y2 = self.gauge_center_y - math.cos(radians) * (self.gauge_radius + 8)
            self.angle_canvas.create_line(x1, y1, x2, y2, width=3)

        self.angle_canvas.create_text(self.gauge_center_x - self.gauge_radius - 18, self.gauge_center_y + 5, text="-90", font=("Arial", 11, "bold"))
        self.angle_canvas.create_text(self.gauge_center_x + self.gauge_radius + 18, self.gauge_center_y + 5, text="90", font=("Arial", 11, "bold"))
        self.angle_canvas.create_oval(self.gauge_center_x - 5, self.gauge_center_y - 5, self.gauge_center_x + 5, self.gauge_center_y + 5, fill="black")
        self.angle_pointer = self.angle_canvas.create_line(self.gauge_center_x, self.gauge_center_y, self.gauge_center_x, self.gauge_center_y - self.gauge_radius + 15, width=5, fill="red", capstyle=tk.ROUND, arrow=tk.LAST)
        self.angle_text = self.angle_canvas.create_text(self.gauge_center_x, self.gauge_center_y + 28, text="0°", font=("Arial", 20, "bold"))

        self.bluetooth_frame = tk.Frame(self.root)
        self.bluetooth_frame.pack(pady=(0, 10))

        self.bluetooth_symbol = tk.Label(self.bluetooth_frame, text="ᛒ", font=("Arial", 26, "bold"), fg="gray")
        self.bluetooth_symbol.pack(side=tk.LEFT)

        self.bluetooth_status_var = tk.StringVar()
        self.bluetooth_status_var.set("Bluetooth disconnected")

        self.bluetooth_status_label = tk.Label(self.bluetooth_frame, textvariable=self.bluetooth_status_var, font=("Arial", 12, "bold"), fg="gray")
        self.bluetooth_status_label.pack(side=tk.LEFT, padx=(5, 0))

        self.warning_var = tk.StringVar()
        self.warning_var.set("no warnings")
        warning_label = tk.Label(self.root, textvariable=self.warning_var, font=("Arial", 11), anchor="w", relief="sunken", padx=8, bg="yellow")
        warning_label.pack(fill="x", padx=20, pady=(0, 30))

    def smooth_speed(self, target_speed):
        current_time = time.time()
        dt = current_time - self.last_speed_update
        self.last_speed_update = current_time

        if target_speed > self.current_speed:
            self.current_speed += self.ACCELERATION_RATE * dt
            self.current_speed = min(self.current_speed, target_speed)
        elif target_speed < self.current_speed:
            self.current_speed -= self.DECELERATION_RATE * dt
            self.current_speed = max(self.current_speed, target_speed)

        self.current_speed = max(0.0, min(self.current_speed, 1.0))
        return self.current_speed

    def send_controller_command(self, direction, speed, angle):
        pass

    def save_dir_sample(self, direction):
        row = self.esp32.left_pressures + self.esp32.right_pressures + [direction]

        try:
            with open(self.dir_data_file, "x", newline="") as file:
                writer = csv.writer(file)
                header = ([f"left_{i}" for i in range(1, 49)] + [f"right_{i}" for i in range(1, 49)] + ["direction"])
                writer.writerow(header)
                writer.writerow(row)
        except FileExistsError:
            with open(self.dir_data_file, "a", newline="") as file:
                csv.writer(file).writerow(row)

        self.warning_var.set(f"{time.strftime('%H:%M:%S')} | saved training sample: {direction}")
        print(f"saved sample: {direction}")

    def save_speed_sample(self, speed):
        if self.prediction is None:
            self.warning_var.set(f"{time.strftime('%H:%M:%S')} | WARNING: no direction identified, sample not saved")
            return

        row = self.esp32.left_pressures + self.esp32.right_pressures + [self.prediction, speed]

        try:
            with open(self.spd_data_file, "x", newline="") as file:
                writer = csv.writer(file)
                header = ([f"left_{i}" for i in range(1, 49)] + [f"right_{i}" for i in range(1, 49)] + ["direction", "speed"])
                writer.writerow(header)
                writer.writerow(row)
        except FileExistsError:
            with open(self.spd_data_file, "a", newline="") as file:
                csv.writer(file).writerow(row)

        self.warning_var.set(f"{time.strftime('%H:%M:%S')} | saved training sample: {self.prediction} speed={speed}")
        print(f"saved sample: {self.prediction} speed={speed}")

    def undo_last_sample(self, sample_type):
        if sample_type == "direction":
            data_file = self.dir_data_file
        elif sample_type == "speed":
            data_file = self.spd_data_file
        else:
            return

        try:
            with open(data_file, "r", newline="") as file:
                rows = list(csv.reader(file))

            if len(rows) <= 1:
                self.warning_var.set(f"{time.strftime('%H:%M:%S')} | WARNING: no samples to undo")
                return

            removed_row = rows.pop()

            with open(data_file, "w", newline="") as file:
                csv.writer(file).writerows(rows)

            if sample_type == "direction":
                removed_dir = removed_row[-1]
                self.warning_var.set(f"{time.strftime('%H:%M:%S')} | removed last sample: {removed_dir}")
                print(f"removed last sample: {removed_dir}")
            else:
                removed_dir = removed_row[-2]
                removed_spd = removed_row[-1]
                self.warning_var.set(f"{time.strftime('%H:%M:%S')} | removed last sample: {removed_dir} speed={removed_spd}")
                print(f"removed last sample: {removed_dir} speed={removed_spd}")

        except FileNotFoundError:
            self.warning_var.set(f"{time.strftime('%H:%M:%S')} | WARNING: no training CSV found")

    def update_bluetooth_status(self, connected):
        if connected:
            self.bluetooth_symbol.config(fg="blue")
            self.bluetooth_status_label.config(fg="blue")
            self.bluetooth_status_var.set("Bluetooth connected")
        else:
            self.bluetooth_symbol.config(fg="gray")
            self.bluetooth_status_label.config(fg="gray")
            self.bluetooth_status_var.set("Bluetooth disconnected")

    def update_speed_bar(self, speed):
        speed = max(0.0, min(speed, 1.0))
        width = self.speed_canvas.winfo_width()

        if width <= 1:
            return

        left = 5
        right = width - 5
        fill_end = left + (right - left) * speed

        self.speed_canvas.coords(self.speed_bar_bg, left, 5, right, 55)
        self.speed_canvas.coords(self.speed_bar_fill, left, 5, fill_end, 55)
        self.speed_canvas.coords(self.speed_text, width / 2, 30)
        self.speed_canvas.itemconfig(self.speed_text, text=f"{speed * 100:.1f}%")

    def update_angle_gauge(self, raw_angle):
        signed_angle = ((raw_angle + 180) % 360) - 180
        display_angle = max(-90, min(90, signed_angle))
        radians = math.radians(display_angle)

        end_x = self.gauge_center_x + math.sin(radians) * (self.gauge_radius - 15)
        end_y = self.gauge_center_y - math.cos(radians) * (self.gauge_radius - 15)

        self.angle_canvas.coords(self.angle_pointer, self.gauge_center_x, self.gauge_center_y, end_x, end_y)
        self.angle_canvas.itemconfig(self.angle_text, text=f"{signed_angle:.0f}°")

    def update_prediction(self):
        values = self.esp32.left_pressures + self.esp32.right_pressures
        columns = ([f"left_{i}" for i in range(1, 49)] + [f"right_{i}" for i in range(1, 49)])
        sample = pd.DataFrame([values], columns=columns)

        probabilities = self.dir_model.predict_proba(sample)[0]
        classes = self.dir_model.classes_

        best_index = probabilities.argmax()
        self.prediction = classes[best_index]
        confidence = probabilities[best_index]
        target_speed = 0.0

        if confidence >= self.CONFIDENCE_THRESHOLD:
            speed_model = self.speed_models.get(self.prediction)

            if speed_model is not None:
                target_speed = speed_model.predict(sample)[0]

            target_speed = max(0.0, min(target_speed, 1.0))

            if self.prediction != "none":
                self.movement_direction = self.prediction
        else:
            self.prediction = None
            target_speed = 0.0

        speed = self.smooth_speed(target_speed)

        if speed <= 0.001:
            speed = 0.0

            if self.prediction is None or self.prediction == "none":
                self.movement_direction = "none"

        self.update_speed_bar(speed)

        if self.prediction is None:
            self.prediction_var.set(f"not identified ({confidence * 100:.1f}%)")
        else:
            self.prediction_var.set(f"{self.prediction} ({confidence * 100:.1f}%)")

        self.send_controller_command(self.movement_direction, speed, self.esp32.encoder_angle)

    def update_interface(self):
        if hasattr(self.esp32, "connected"):
            self.update_bluetooth_status(self.esp32.connected)

        new_data = self.esp32.update()

        if new_data:
            self.last_esp32_packet_time = time.time()
            self.esp32_watchdog_active = False

            self.left_grid.update()
            self.right_grid.update()

            if self.dir_model is None:
                self.prediction_var.set("Prediction: no model loaded")
            else:
                self.update_prediction()

            self.update_angle_gauge(self.esp32.encoder_angle)

        if time.time() - self.last_esp32_packet_time > self.ESP32_TIMEOUT:
            if not self.esp32_watchdog_active:
                self.send_controller_command("none", 0.0, 0.0)
                self.esp32_watchdog_active = True
                self.warning_var.set(f"{time.strftime('%H:%M:%S')} | WARNING: ESP32 data timeout, output stopped")

        if time.time() - self.last_stats_print >= 1:
            if not hasattr(self.esp32, "connected") or self.esp32.connected:
                print(f"ESP32 | bad checksums: {self.esp32.bad_checksums} | packets: {self.esp32.packets_read} | buffer: {len(self.esp32.buffer)} | position: {self.esp32.encoder_position} | angle: {self.esp32.encoder_angle} | center: {self.esp32.center_found} | A: {self.esp32.encoder_sensor_a} | B: {self.esp32.encoder_sensor_b} | C: {self.esp32.encoder_sensor_c}")

            self.last_stats_print = time.time()

        self.root.after(1, self.update_interface)

    def close_output(self):
        pass

    def close(self):
        try:
            self.send_controller_command("none", 0.0, 0.0)
            self.close_output()
            self.esp32.close()
        finally:
            self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    ControllerGUI().run()