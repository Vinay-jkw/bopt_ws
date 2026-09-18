import serial
import time
 
# Target device definition
SERIAL_PORT = '/dev/imu_wit'
INITIAL_BAUD = 9600
 
try:
    print(f"Opening port {SERIAL_PORT} at {INITIAL_BAUD} bps...")
    ser = serial.Serial(SERIAL_PORT, INITIAL_BAUD, timeout=1)
    time.sleep(1) # Allow serial engine link to stabilize
    
    # 1. UNLOCK REGISTERS (Required by WIT Standard Protocol)
    # Target address: 0x69 | Payload: 0x88, 0xB5
    unlock_cmd = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
    
    # 2. CONFIG BAUD RATE TO 115200
    # Target address: 0x04 (Baud control) | Payload: 0x06 (115200 mapping value)
    baud_115200_cmd = bytes([0xFF, 0xAA, 0x04, 0x06, 0x00])
    
    # 3. SAVE TO ONBOARD FLASH
    # Target address: 0x00 | Payload: 0x00, 0x00
    save_cmd = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])
    
    # Sequence Execution
    print("Step 1: Sending Device Unlock Sequence...")
    ser.write(unlock_cmd)
    time.sleep(0.1)
    
    print("Step 2: Writing new Baud Rate configuration (115200)...")
    ser.write(baud_115200_cmd)
    time.sleep(0.1)
    
    print("Step 3: Committing configurations permanently to Flash Memory...")
    ser.write(save_cmd)
    time.sleep(0.1)
    
    print("\n--- Command Sequence Completed ---")
    print("Power-cycle your IMU (unplug and replug the USB) to apply changes.")
    ser.close()
 
except Exception as e:
    print(f"Execution failed: {e}")