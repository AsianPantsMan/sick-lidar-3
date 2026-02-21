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
import random 
def read_from_port(port_name,baudrate):
    ser=serial.Serial(port_name,baudrate,timeout=1)
    buffer=bytearray(38)
    for i in range(38):
        buffer[i]=random.randint(32,126)
        while True:
            data_to_send="Hello\n"
            ser.write(buffer)
            print(f"Sent: {data_to_send.strip()}")
            time.sleep(1)
            response=ser.readline()
            print(f"Received: {response}")
if __name__=="__main__":
    port_name="/dev/ttyAMA0"  # Update with your port name
    baudrate=115200
    read_from_port(port_name,baudrate)
