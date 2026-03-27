import serial
import time 
def read_from_port(port_name,baudrate):
        ser=serial.Serial(port_name,baudrate,timeout=1)
        while True:
            data_to_send="Hello from Raspberry Pi!\n"
            ser.write(data_to_send.encode())
            print(f"Sent: {data_to_send.strip()}")
            time.sleep(1)
            response=ser.readline().decode('utf-8').strip()
            print(f"Received: {response}")
if __name__=="__main__":
    port_name="/dev/ttyAMA0"  # Update with your port name
    baudrate=115200
    read_from_port(port_name,baudrate)


import serial
import time

def test_send_only(port_name, baudrate):
    ser = serial.Serial(port_name, baudrate, timeout=1)
    time.sleep(0.5)
    try:
    # Create test buffer (38 bytes) matching your protocol
        buffer = bytearray(38)
        buffer[0] = 0xAA  # Start marker
        for i in range(1, 37):
            uffer[i] = i % 256  # Some test data
        buffer[37] = 0x55  # End marker
    
        print("Sending data to STM32...")
    
        while(True):  # Send 10 times
            bytes_written = ser.write(buffer)
            ser.flush()  # Force it out immediately
            print(f"Test {i+1}: Sent {bytes_written} bytes successfully")
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
        ser.close()
    
    
if __name__ == "__main__":
    test_send_only("/dev/ttyAMA0", 115200)


