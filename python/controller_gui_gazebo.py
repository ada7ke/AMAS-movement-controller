import serial, time, csv, joblib, socket, math
import tkinter as tk
import pandas as pd
import common as cmn

class FootPressureSensor:
    def __init__(self, name):
        self.name = name
        self.pressures = [0] * 48

class ESP32Receiver:
    def __init__(self, port, baud_rate=921600):
        self.serial = serial.Serial(port, baud_rate, timeout=0)
        self.buffer = bytearray()

        self.bad_checksums = 0
        self.packets_read = 0

        self.encoder_sensor_a = 0
        self.encoder_sensor_b = 0
        self.encoder_sensor_c = 0

        self.encoder_position = 0
        self.encoder_angle = 0
        self.center_found = False

    def update(self, left_sensor, right_sensor):
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
                left_sensor.pressures[i] = low | (high << 8)
                index += 2

            for i in range(48):
                low = packet[index]
                high = packet[index + 1]
                right_sensor.pressures[i] = low | (high << 8)
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

class FootGrid:
    def __init__(self, parent, name, sensor, sensor_numbers):
        self.sensor = sensor
        self.sensor_numbers = sensor_numbers
        self.cells = {}

        frame = tk.LabelFrame(parent, text=name, font=("Arial", 16, "bold"), padx=10, pady=10)
        frame.pack(side=tk.LEFT, padx=20, pady=20)

        for row in range(12):
            for col in range(4):
                sensor_number = sensor_numbers[row][col]

                cell = tk.Label(
                    frame,
                    text=f"{sensor_number}\n0",
                    width=6,
                    height=2,
                    font=("Arial", 11, "bold"),
                    relief="solid",
                    borderwidth=1,
                    bg="white"
                )

                cell.grid(row=row, column=col, padx=2, pady=2)
                self.cells[sensor_number] = cell

    def pressure_to_color(self, pressure):
        max_pressure = 200
        pressure = max(0, min(pressure, max_pressure))

        intensity_scale = 1
        intensity = (pressure / max_pressure) ** intensity_scale

        red = 255
        green = int(255 * (1 - intensity))
        blue = int(255 * (1 - intensity))

        return f"#{red:02x}{green:02x}{blue:02x}"

    def update(self):
        for sensor_number, cell in self.cells.items():
            value = self.sensor.pressures[sensor_number - 1]
            color = self.pressure_to_color(value)

            cell.config(text=f"{value}", bg=color)

left_sensor = FootPressureSensor("left")
right_sensor = FootPressureSensor("right")

esp32 = ESP32Receiver("COM5")
ROBOT_IP = "172.17.149.227"
ROBOT_PORT = 5005
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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

DIR_DATA_FILE = f"datasets/pressure_training({cmn.dir_dataset}).csv"
DIR_MODEL_FILE = f"models/pressure_svm({cmn.dir_dataset}).pkl"
SPD_DATA_FILE = f"datasets/speed_training({cmn.spd_dataset}).csv"
SPD_MODEL_FILES = {
    "forward": f"models/fwd_spd_svr({cmn.spd_dataset}).pkl",
    "backward": f"models/bwd_spd_svr({cmn.spd_dataset}).pkl",
    "strafe_left": f"models/sl_spd_svr({cmn.spd_dataset}).pkl",
    "strafe_right": f"models/sr_spd_svr({cmn.spd_dataset}).pkl"
}
dir_model = None
fwd_spd_model = None
bwd_spd_model = None
sl_spd_model = None
sr_spd_model = None
try:
    dir_model = joblib.load(DIR_MODEL_FILE)
    fwd_spd_model = joblib.load(SPD_MODEL_FILES["forward"])
    bwd_spd_model = joblib.load(SPD_MODEL_FILES["backward"])
    sl_spd_model = joblib.load(SPD_MODEL_FILES["strafe_left"])
    sr_spd_model = joblib.load(SPD_MODEL_FILES["strafe_right"])
except FileNotFoundError:
    print("one or more models not found")

prediction = None
CONFIDENCE_THRESHOLD = 0.70

ACCELERATION_RATE = 1.0
DECELERATION_RATE = 1.5
current_speed = 0.0
last_speed_update = time.time()
movement_direction = "none"

root = tk.Tk()
root.title("Foot Pressure Sensors")
grids_frame = tk.Frame(root)
grids_frame.pack(padx=(10, 10))
dir_btn_frame = tk.Frame(root)
dir_btn_frame.pack(pady=(0, 10))
spd_btn_frame = tk.Frame(root)
spd_btn_frame.pack(pady=(0, 10))

left_grid = FootGrid(grids_frame, "Left Foot", left_sensor, left_indexes)
right_grid = FootGrid(grids_frame, "Right Foot", right_sensor, right_indexes)

btn_width = 10
forward_btn = tk.Button(dir_btn_frame, text="forward", width=btn_width, command=lambda: save_dir_sample("forward"))
backward_btn = tk.Button(dir_btn_frame, text="backward", width=btn_width, command=lambda: save_dir_sample("backward"))
left_btn = tk.Button(dir_btn_frame, text="strafe left", width=btn_width, command=lambda: save_dir_sample("strafe_left"))
right_btn = tk.Button(dir_btn_frame, text="strafe right", width=btn_width, command=lambda: save_dir_sample("strafe_right"))
none_btn = tk.Button(dir_btn_frame, text="none", width=btn_width, command=lambda: save_dir_sample("none"))
undo_btn = tk.Button(dir_btn_frame, text="undo", width=btn_width, command=lambda: undo_last_sample("direction"))
forward_btn.pack(side=tk.LEFT, padx=5)
backward_btn.pack(side=tk.LEFT, padx=5)
left_btn.pack(side=tk.LEFT, padx=5)
right_btn.pack(side=tk.LEFT, padx=5)
none_btn.pack(side=tk.LEFT, padx=5)
undo_btn.pack(side=tk.LEFT, padx=5)

spd0_btn = tk.Button(spd_btn_frame, text="0%", width=btn_width, command=lambda: save_speed_sample(0.0))
spd50_btn = tk.Button(spd_btn_frame, text="50%", width=btn_width, command=lambda: save_speed_sample(0.5))
spd100_btn = tk.Button(spd_btn_frame, text="100%", width=btn_width, command=lambda: save_speed_sample(1.0))
undo_btn = tk.Button(spd_btn_frame, text="undo", width=btn_width, command=lambda: undo_last_sample("speed"))
spd0_btn.pack(side=tk.LEFT, padx=5)
spd50_btn.pack(side=tk.LEFT, padx=5)
spd100_btn.pack(side=tk.LEFT, padx=5)
undo_btn.pack(side=tk.LEFT, padx=5)
controls_frame = tk.Frame(root)

controls_frame.pack(fill="both", padx=20, pady=(0, 20))
controls_frame.grid_columnconfigure(0, weight=1)
controls_frame.grid_columnconfigure(1, weight=0, minsize=300)
controls_frame.grid_rowconfigure(0, weight=0)
controls_frame.grid_rowconfigure(1, weight=0)

prediction_frame = tk.LabelFrame(controls_frame, text="Prediction", font=("Arial", 14, "bold"), padx=10, pady=3)
prediction_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 5))
prediction_var = tk.StringVar()
prediction_var.set("waiting...")
prediction_label = tk.Label(prediction_frame, textvariable=prediction_var, font=("Arial", 20, "bold"), anchor="center", relief="flat", padx=8, pady=1)
prediction_label.pack(fill="both", expand=True)

speed_frame = tk.LabelFrame(controls_frame, text="Speed", font=("Arial", 14, "bold"), padx=10, pady=3)
speed_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 5), pady=(5, 0))
speed_canvas = tk.Canvas(speed_frame, height=60)
speed_canvas.pack(fill="both", expand=True)
speed_bar_bg = speed_canvas.create_rectangle(5, 5, 595, 55, fill="white", outline="black")
speed_bar_fill = speed_canvas.create_rectangle(5, 5, 5, 55, fill="red", outline="")
speed_text = speed_canvas.create_text(300, 30, text="0%", font=("Arial", 16, "bold"))

GAUGE_WIDTH = 260
GAUGE_HEIGHT = 150
GAUGE_CENTER_X = GAUGE_WIDTH // 2
GAUGE_CENTER_Y = 110
GAUGE_RADIUS = 90
angle_frame = tk.LabelFrame(controls_frame, text="Angle", font=("Arial", 14, "bold"), padx=10, pady=3)
angle_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(5, 0))
angle_canvas = tk.Canvas(angle_frame, width=GAUGE_WIDTH, height=GAUGE_HEIGHT)
angle_canvas.pack(expand=True)
angle_canvas.create_arc(GAUGE_CENTER_X - GAUGE_RADIUS, GAUGE_CENTER_Y - GAUGE_RADIUS, GAUGE_CENTER_X + GAUGE_RADIUS, GAUGE_CENTER_Y + GAUGE_RADIUS, start=0, extent=180, style=tk.ARC, width=5)
for tick_angle in [-90, -45, 0, 45, 90]:
    radians = math.radians(tick_angle)
    x1 = GAUGE_CENTER_X + math.sin(radians) * (GAUGE_RADIUS - 8)
    y1 = GAUGE_CENTER_Y - math.cos(radians) * (GAUGE_RADIUS - 8)
    x2 = GAUGE_CENTER_X + math.sin(radians) * (GAUGE_RADIUS + 8)
    y2 = GAUGE_CENTER_Y - math.cos(radians) * (GAUGE_RADIUS + 8)
    angle_canvas.create_line(x1, y1, x2, y2, width=3)
angle_canvas.create_text(GAUGE_CENTER_X - GAUGE_RADIUS - 18, GAUGE_CENTER_Y + 5, text="-90", font=("Arial", 11, "bold"))
angle_canvas.create_text(GAUGE_CENTER_X + GAUGE_RADIUS + 18, GAUGE_CENTER_Y + 5, text="90", font=("Arial", 11, "bold"))
angle_canvas.create_oval(GAUGE_CENTER_X - 5, GAUGE_CENTER_Y - 5, GAUGE_CENTER_X + 5, GAUGE_CENTER_Y + 5, fill="black")
angle_pointer = angle_canvas.create_line(GAUGE_CENTER_X, GAUGE_CENTER_Y, GAUGE_CENTER_X, GAUGE_CENTER_Y - GAUGE_RADIUS + 15, width=5, fill="red", capstyle=tk.ROUND, arrow=tk.LAST)
angle_text = angle_canvas.create_text(GAUGE_CENTER_X, GAUGE_CENTER_Y + 28, text="0°", font=("Arial", 20, "bold"))

warning_var = tk.StringVar()
warning_var.set("no warnings")
warning_label = tk.Label(root, textvariable=warning_var, font=("Arial", 11), anchor="w", relief="sunken", padx=8, bg="yellow")
warning_label.pack(fill="x", padx=20, pady=(0, 30))

def smooth_speed(target_speed):
    global current_speed, last_speed_update

    current_time = time.time()
    dt = current_time - last_speed_update
    last_speed_update = current_time

    if target_speed > current_speed:
        current_speed += ACCELERATION_RATE * dt
        current_speed = min(current_speed, target_speed)
    elif target_speed < current_speed:
        current_speed -= DECELERATION_RATE * dt
        current_speed = max(current_speed, target_speed)

    current_speed = max(0.0, min(current_speed, 1.0))

    return current_speed

def send_robot_command(direction, speed, angle):
    message = f"{direction},{speed},{angle}"
    robot_socket.sendto(message.encode(), (ROBOT_IP, ROBOT_PORT))

def save_dir_sample(dir):
    row = left_sensor.pressures + right_sensor.pressures + [dir]

    try:
        with open(DIR_DATA_FILE, "x", newline="") as file:
            writer = csv.writer(file)

            header = ([f"left_{i}" for i in range(1, 49)] + [f"right_{i}" for i in range(1, 49)] + ["direction"])

            writer.writerow(header)
            writer.writerow(row)

    except FileExistsError:
        with open(DIR_DATA_FILE, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(row)

    current_time = time.strftime("%H:%M:%S")
    warning_var.set(f"{current_time} | saved training sample: {dir}")

    print(f"saved sample: {dir}")

def save_speed_sample(spd):
    if prediction is None:
        warning_var.set(f"{time.strftime('%H:%M:%S')} | WARNING: no direction identified, sample not saved")
        return

    row = left_sensor.pressures + right_sensor.pressures + [prediction, spd]
    
    try:
        with open(SPD_DATA_FILE, "x", newline="") as file:
            writer = csv.writer(file)

            header = ([f"left_{i}" for i in range(1, 49)] + [f"right_{i}" for i in range(1, 49)] + ["direction", "speed"])

            writer.writerow(header)
            writer.writerow(row)

    except FileExistsError:
        with open(SPD_DATA_FILE, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(row)

    current_time = time.strftime("%H:%M:%S")
    warning_var.set(f"{current_time} | saved training sample: {prediction} speed={spd}")

    print(f"saved sample: {prediction} speed={spd}")

def undo_last_sample(sample_type):
    if sample_type == "direction":
        DATA_FILE = DIR_DATA_FILE
    elif sample_type == "speed":
        DATA_FILE = SPD_DATA_FILE
    else:
        return

    try:
        with open(DATA_FILE, "r", newline="") as file:
            rows = list(csv.reader(file))

        if len(rows) <= 1:
            warning_var.set(f"{time.strftime('%H:%M:%S')} | WARNING: no samples to undo")
            return

        removed_row = rows.pop()

        with open(DATA_FILE, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerows(rows)

        if sample_type == "direction":
            removed_dir = removed_row[-1]
            warning_var.set(f"{time.strftime('%H:%M:%S')} | removed last sample: {removed_dir}")
            print(f"removed last sample: {removed_dir}")

        elif sample_type == "speed":
            removed_dir = removed_row[-2]
            removed_spd = removed_row[-1]
            warning_var.set(f"{time.strftime('%H:%M:%S')} | removed last sample: {removed_dir} speed={removed_spd}")
            print(f"removed last sample: {removed_dir} speed={removed_spd}")

    except FileNotFoundError:
        warning_var.set(f"{time.strftime('%H:%M:%S')} | WARNING: no training CSV found")

def update_speed_bar(speed):
    speed = max(0.0, min(speed, 1.0))

    width = speed_canvas.winfo_width()
    if width <= 1:
        return

    left = 5
    right = width - 5
    bar_width = right - left

    fill_end = left + bar_width * speed

    speed_canvas.coords(speed_bar_bg, left, 8, right, 35)
    speed_canvas.coords(speed_bar_fill, left, 8, fill_end, 35)
    speed_canvas.coords(speed_text, width / 2, 21)

    speed_canvas.itemconfig(speed_text, text=f"{speed * 100:.1f}%")

def update_angle_gauge(raw_angle):
    signed_angle = ((raw_angle+180) % 360) - 180

    display_angle = max(-90, min(90, signed_angle))

    radians = math.radians(display_angle)

    end_x = GAUGE_CENTER_X + math.sin(radians) * (GAUGE_RADIUS - 15)
    end_y = GAUGE_CENTER_Y - math.cos(radians) * (GAUGE_RADIUS - 15)

    angle_canvas.coords(
        angle_pointer,
        GAUGE_CENTER_X,
        GAUGE_CENTER_Y,
        end_x,
        end_y
    )

    angle_canvas.itemconfig(angle_text, text=f"{signed_angle:.0f}°")

def update_prediction():
    global prediction, movement_direction

    values = left_sensor.pressures + right_sensor.pressures
    columns = ([f"left_{i}" for i in range(1, 49)] + [f"right_{i}" for i in range(1, 49)])
    sample = pd.DataFrame([values], columns=columns)

    probabilities = dir_model.predict_proba(sample)[0]
    classes = dir_model.classes_

    best_index = probabilities.argmax()
    prediction = classes[best_index]
    confidence = probabilities[best_index]

    target_speed = 0.0

    if confidence >= CONFIDENCE_THRESHOLD:
        if prediction == "forward" and fwd_spd_model is not None:
            target_speed = fwd_spd_model.predict(sample)[0]
        elif prediction == "backward" and bwd_spd_model is not None:
            target_speed = bwd_spd_model.predict(sample)[0]
        elif prediction == "strafe_left" and sl_spd_model is not None:
            target_speed = sl_spd_model.predict(sample)[0]
        elif prediction == "strafe_right" and sr_spd_model is not None:
            target_speed = sr_spd_model.predict(sample)[0]
        else:
            target_speed = 0.0

        target_speed = max(0.0, min(target_speed, 1.0))

        if prediction != "none":
            movement_direction = prediction

    else:
        prediction = None
        target_speed = 0.0

    speed = smooth_speed(target_speed)

    if speed <= 0.001:
        speed = 0.0

        if prediction is None or prediction == "none":
            movement_direction = "none"

    update_speed_bar(speed)

    if prediction is None:
        prediction_var.set(f"not identified ({confidence * 100:.1f}%)")
    else:
        prediction_var.set(f"{prediction} ({confidence * 100:.1f}%)")

    send_robot_command(movement_direction, speed, esp32.encoder_angle)

last_stats_print = time.time()
def update_interface():
    global last_stats_print

    new_data = esp32.update(left_sensor, right_sensor)

    if new_data:
        left_grid.update()
        right_grid.update()

        if dir_model is None:
            prediction_var.set("Prediction: no model loaded")
        else:
            update_prediction()

        update_angle_gauge(esp32.encoder_angle)
        
    if time.time() - last_stats_print >= 1:
        print(f"ESP32 | bad checksums: {esp32.bad_checksums} | in_waiting: {esp32.serial.in_waiting} | buffer: {len(esp32.buffer)} | position: {esp32.encoder_position} | angle: {esp32.encoder_angle} | center: {esp32.center_found} | A: {esp32.encoder_sensor_a} | B: {esp32.encoder_sensor_b} | C: {esp32.encoder_sensor_c}")

        last_stats_print = time.time()

    root.after(1, update_interface)

root.after(1, update_interface)
root.mainloop()