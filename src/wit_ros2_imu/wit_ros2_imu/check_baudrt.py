import serial
import time

PORT = "/dev/ttyUSB0"

for baud in [9600, 115200, 230400, 460800, 921600]:

    print(f"\nTesting {baud} baud...")

    ser = serial.Serial(
        PORT,
        baudrate=baud,
        timeout=1
    )

    data = ser.read(200)
    ser.close()

    print("Bytes received:", len(data))
    print("First bytes:", data[:20].hex(" "))

    time.sleep(1)