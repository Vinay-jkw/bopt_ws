import serial
import time

def send_command(ser, cmd_name, cmd_bytes):
    print(f"Sending {cmd_name}...")
    ser.write(cmd_bytes)
    time.sleep(0.1)

port = '/dev/ttyUSB0'
baud = 9600

ser = serial.Serial(port, baud, timeout=1)
time.sleep(1)

# Unlock
send_command(ser, "Unlock", bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))
# Set Z-axis angle to zero
send_command(ser, "Set Z-axis to zero", bytes([0xFF, 0xAA, 0x01, 0x04, 0x00]))
# Save
send_command(ser, "Save Configuration", bytes([0xFF, 0xAA, 0x00, 0x00, 0x00]))
ser.close()
print("Z-axis reset to 0.")
