import serial, time

PORT = "COM8"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

print(f"listening on {PORT} at {BAUD} baud")

while True:
    data = ser.read(256)

    if data:
        print(f"{len(data)} bytes: {data.hex(' ')}")

    time.sleep(0.01)