import serial
import time
import sys

def send_command(ser, cmd_name, cmd_bytes):
    print(f"Sending {cmd_name}...")
    ser.write(cmd_bytes)
    time.sleep(0.1)

def calibrate_accel_gyro(port, baud):
    print("\n--- Acceleration & Gyroscope Calibration ---")
    print("1. Place the IMU perfectly flat and completely stationary.")
    input("Press Enter when ready...")
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(1)
        
        # Unlock
        send_command(ser, "Unlock", bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))
        
        # Start Accel/Gyro Calibration
        send_command(ser, "Start Calibration", bytes([0xFF, 0xAA, 0x01, 0x01, 0x00]))
        
        print("Calibrating... DO NOT MOVE THE IMU!")
        time.sleep(4)
        
        # Save
        send_command(ser, "Save Configuration", bytes([0xFF, 0xAA, 0x00, 0x00, 0x00]))
        ser.close()
        print("Done! Acceleration & Gyroscope Calibration complete.")
    except Exception as e:
        print(f"Error: {e}")

def calibrate_magnetic(port, baud):
    print("\n--- Magnetic Field (Compass) Calibration ---")
    print("1. Keep the IMU away from magnetic interference (metals, magnets).")
    input("Press Enter to START calibration...")
    
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(1)
        
        # Unlock
        send_command(ser, "Unlock", bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))
        
        # Start Mag Calibration
        send_command(ser, "Start Mag Calibration", bytes([0xFF, 0xAA, 0x01, 0x07, 0x00]))
        
        print("\n>>> NOW ROTATE THE IMU <<<")
        print("Slowly rotate the IMU 360 degrees around all 3 axes (X, Y, Z).")
        print("Keep doing this for about 10-15 seconds in the air.")
        input("Press Enter when you have FINISHED rotating...")
        
        # End Mag Calibration
        send_command(ser, "End Mag Calibration", bytes([0xFF, 0xAA, 0x01, 0x00, 0x00]))
        
        # Save
        send_command(ser, "Save Configuration", bytes([0xFF, 0xAA, 0x00, 0x00, 0x00]))
        ser.close()
        print("Done! Magnetic Calibration complete.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    PORT = '/dev/ttyUSB0'
    BAUD = 9600 # Matching your current hardware speed
    
    print("WIT-Motion IMU Calibration Utility")
    print("1. Accelerometer & Gyroscope (Zeroing)")
    print("2. Magnetic Field (Compass)")
    print("3. Exit")
    
    choice = input("Select an option (1/2/3): ")
    
    if choice == '1':
        calibrate_accel_gyro(PORT, BAUD)
    elif choice == '2':
        calibrate_magnetic(PORT, BAUD)
    else:
        print("Exiting.")
        sys.exit(0)
