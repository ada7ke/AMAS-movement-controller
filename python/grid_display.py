import csv
import tkinter as tk
import pandas as pd
import common as cmn

LINE_NUMBER = 1002

DIR_DATA_FILE = f"datasets/pressure_training({cmn.dir_dataset}).csv"


class FootPressureSensor:
    def __init__(self, name):
        self.name = name
        self.pressures = [0] * 48


class FootGrid:
    def __init__(self, parent, name, sensor, sensor_numbers):
        self.sensor = sensor
        self.cells = {}

        frame = tk.LabelFrame(parent, text=name, font=("Arial", 16, "bold"), padx=10, pady=10)
        frame.pack(side=tk.LEFT, padx=20, pady=20)

        for row in range(12):
            for col in range(4):
                sensor_number = sensor_numbers[row][col]

                cell = tk.Label(
                    frame,
                    text="0",
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
        max_pressure = 500
        pressure = max(0, min(pressure, max_pressure))

        intensity_scale = 0.5
        intensity = (pressure / max_pressure) ** intensity_scale

        red = 255
        green = int(255 * (1 - intensity))
        blue = int(255 * (1 - intensity))

        return f"#{red:02x}{green:02x}{blue:02x}"

    def update(self):
        for sensor_number, cell in self.cells.items():
            value = self.sensor.pressures[sensor_number - 1]
            color = self.pressure_to_color(value)

            cell.config(text=str(value), bg=color)


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


def load_entry(line_number):
    data = pd.read_csv(DIR_DATA_FILE)

    # CSV line 1 is the header, so dataframe index is line_number - 2
    index = line_number - 2

    if index < 0 or index >= len(data):
        raise ValueError(f"Line {line_number} does not exist")

    row = data.iloc[index]

    left_values = [int(row[f"left_{i}"]) for i in range(1, 49)]
    right_values = [int(row[f"right_{i}"]) for i in range(1, 49)]
    direction = row["direction"]

    return left_values, right_values, direction


left_sensor = FootPressureSensor("left")
right_sensor = FootPressureSensor("right")

left_sensor.pressures, right_sensor.pressures, direction = load_entry(LINE_NUMBER)

root = tk.Tk()
root.title(f"Pressure Training Entry - Line {LINE_NUMBER}")

info_label = tk.Label(
    root,
    text=f"CSV line: {LINE_NUMBER} | Direction: {direction}",
    font=("Arial", 16, "bold")
)
info_label.pack(pady=(15, 0))

grids_frame = tk.Frame(root)
grids_frame.pack()

left_grid = FootGrid(grids_frame, "Left Foot", left_sensor, left_indexes)
right_grid = FootGrid(grids_frame, "Right Foot", right_sensor, right_indexes)

left_grid.update()
right_grid.update()

root.mainloop()