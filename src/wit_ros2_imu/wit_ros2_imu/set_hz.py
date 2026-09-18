import serial
import time
 
# Note: We are opening the port at your working 115200 speed now!
SERIAL_PORT = '/dev/imu_wit'
CURRENT_BAUD = 115200
 
try:
    print(f"Opening port {SERIAL_PORT} at {CURRENT_BAUD} bps...")
    ser = serial.Serial(SERIAL_PORT, CURRENT_BAUD, timeout=1)
    time.sleep(1)
    
    # Step 1: UNLOCK REGISTERS
    unlock_cmd = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
    
    # Step 2: SET RETURN RATE REGISTER (0x03)
    # Target address: 0x03 | Payload: 0x09 (for 100Hz) or 0x0B (for 200Hz)
    rate_100hz_cmd = bytes([0xFF, 0xAA, 0x03, 0x09, 0x00])
    
    # Step 3: SAVE TO FLASH
    save_cmd = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])
    
    # Execute commands sequentially
    print("Sending Unlock Command...")
    ser.write(unlock_cmd)
    time.sleep(0.1)
    
    print("Changing internal frequency to 100Hz...")
    ser.write(rate_100hz_cmd)
    time.sleep(0.1)
    
    print("Saving configuration to flash...")
    ser.write(save_cmd)
    time.sleep(0.1)
    
    print("\nFrequency change successful!")
    print("IMPORTANT: Power-cycle the IMU (unplug and replug the USB) to apply updates.")
    ser.close()
 
except Exception as e:
    print(f"Error: {e}")