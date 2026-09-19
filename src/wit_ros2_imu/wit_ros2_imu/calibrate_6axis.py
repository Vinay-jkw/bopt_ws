import serial
import time

def send_command(ser, cmd_name, cmd_bytes):
    print(f"Sending {cmd_name}...")
    ser.write(cmd_bytes)
    time.sleep(0.1)

port = '/dev/ttyUSB0'
baud = 230400

ser = serial.Serial(port, baud, timeout=1)
time.sleep(1)

# Unlock
send_command(ser, "Unlock", bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))
# Set 6-axis mode
send_command(ser, "Set 6-axis mode", bytes([0xFF, 0xAA, 0x24, 0x01, 0x00]))
time.sleep(0.5)
# Set Z-axis to zero
send_command(ser, "Set Z-axis to zero", bytes([0xFF, 0xAA, 0x01, 0x04, 0x00]))
time.sleep(0.5)
# Save
send_command(ser, "Save Configuration", bytes([0xFF, 0xAA, 0x00, 0x00, 0x00]))
ser.close()
print("Set to 6-axis mode and zeroed Z-axis.")
