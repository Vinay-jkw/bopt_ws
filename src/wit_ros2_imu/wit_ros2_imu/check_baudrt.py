import serial
import time
 
def test_baud(port, baudrate):
    print(f"Testing {baudrate} bps...")
    try:
        # Open serial port with a short 1-second timeout
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.5) # Let connection settle
        ser.reset_input_buffer()
        
        # Read a chunk of incoming data
        data = ser.read(50)
        ser.close()
        
        # Check if we received the standard WITMOTION data packet header (0x55)
        if len(data) > 0 and b'\x55' in data:
            return True
        return False
    except Exception as e:
        print(f"Error opening port at {baudrate}: {e}")
        return False
 
# Target device port
port_name = '/dev/imu_wit'
 
if test_baud(port_name, 115200):
    print("\nSUCCESS: Your IMU is currently configured to 115200 baud rate!")
elif test_baud(port_name, 9600):
    print("\nNOTICE: Your IMU is still configured to the default 9600 baud rate.")
else:
    print("\nFAILED: Could not read structured data at either speed.")
    print("Please verify your USB cable connection or run: sudo chmod 666 /dev/imu_wit")