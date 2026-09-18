#!/usr/bin/env python3
import serial
import struct

# Convert 2 bytes (little-endian) to signed int16
def to_int16(lo, hi):
    return struct.unpack('<h', bytes([lo, hi]))[0]

class IMUParser:
    def __init__(self):
        self.acc = [0, 0, 0]
        self.gyro = [0, 0, 0]
        self.angle = [0, 0, 0]
        self.mag = [0, 0, 0]
        self.quat = [0, 0, 0, 0]

    def parse_frame(self, frame):
        if frame[0] != 0x55:
            return
        
        ftype = frame[1]

        if ftype == 0x51:
            # Acceleration (g)
            self.acc = [
                to_int16(frame[2], frame[3]) / 32768.0 * 16,
                to_int16(frame[4], frame[5]) / 32768.0 * 16,
                to_int16(frame[6], frame[7]) / 32768.0 * 16,
            ]

        elif ftype == 0x52:
            # Gyro (deg/s)
            self.gyro = [
                to_int16(frame[2], frame[3]) / 32768.0 * 2000,
                to_int16(frame[4], frame[5]) / 32768.0 * 2000,
                to_int16(frame[6], frame[7]) / 32768.0 * 2000,
            ]

        elif ftype == 0x53:
            # Angle (degrees)
            self.angle = [
                to_int16(frame[2], frame[3]) / 32768.0 * 180,
                to_int16(frame[4], frame[5]) / 32768.0 * 180,
                to_int16(frame[6], frame[7]) / 32768.0 * 180,
            ]

        elif ftype == 0x54:
            # Magnetometer (raw µT)
            self.mag = [
                to_int16(frame[2], frame[3]),
                to_int16(frame[4], frame[5]),
                to_int16(frame[6], frame[7]),
            ]

        elif ftype == 0x5D:
            # Quaternion
            self.quat = [
                to_int16(frame[2], frame[3]) / 32768.0,
                to_int16(frame[4], frame[5]) / 32768.0,
                to_int16(frame[6], frame[7]) / 32768.0,
                to_int16(frame[8], frame[9]) / 32768.0,
            ]

    def print_data(self):
        print(f"ACC  : {self.acc}")
        print(f"GYRO : {self.gyro}")
        print(f"ANGLE: {self.angle}")
        print(f"MAG  : {self.mag}")
        print(f"QUAT : {self.quat}")
        print("-" * 50)


def read_imu(port="/dev/ttyUSB0", baud=115200):
    ser = serial.Serial(port, baud, timeout=0.1)
    parser = IMUParser()

    frame = []

    while True:
        b = ser.read(1)
        if not b:
            continue
        
        byte = b[0]

        # detect header
        if byte == 0x55 and len(frame) == 0:
            frame.append(byte)
            continue

        if len(frame) > 0:
            frame.append(byte)

            # Full frame is always 11 bytes
            if len(frame) == 11:
                parser.parse_frame(frame)
                parser.print_data()
                frame = []

if __name__ == "__main__":
    read_imu("/dev/ttyUSB0", 9600)
