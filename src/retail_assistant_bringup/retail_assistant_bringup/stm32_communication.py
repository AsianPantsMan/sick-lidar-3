#!/usr/bin/env python3
import serial
import struct
import time
import rclpy
from std_msgs.msg import Int64MultiArray
from rclpy.node import Node
from sensor_msgs.msg import Imu

class STM32_UART_Receiver(Node):   # inherit node
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200, timeout=1):
        super().__init__('stm32_uart_receiver')  

        self.base_frame = 'base_link'  

        self.Imu_pub = self.create_publisher(Imu, '/stm32_imu', 10)
        self.enocder_pub = self.create_publisher(Int64MultiArray, '/stm32_encoder', 10)

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            print(f"Connected to {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise

    def find_frame_start(self):
        while True:
            byte = self.ser.read(1)
            if len(byte) == 0:
                return False
            if byte[0] == 0xAA:
                return True

    def receive_data(self):
        if not self.find_frame_start():
            print("Timeout waiting for start marker")
            return None

        data = self.ser.read(101)
        if len(data) != 101:
            print(f"Incomplete frame received: {len(data)} bytes")
            return None

        if data[100] != 0x55:
            print(f"Invalid end marker: 0x{data[36]:02X} (expected 0x55)")
            return None

        try:
            imu_w = struct.unpack('<d', data[0:8])[0]
            imu_x = struct.unpack('<d', data[8:16])[0]
            imu_y = struct.unpack('<d', data[16:24])[0]
            imu_z = struct.unpack('<d', data[24:32])[0]
            imu_linear_w = struct.unpack('<d', data[32:40])[0]  
            imu_linear_x = struct.unpack('<d', data[40:48])[0]  
            imu_linear_y = struct.unpack('<d', data[48:56])[0]  
            imu_linear_z = struct.unpack('<d', data[56:64])[0]
            imu_angular_w = struct.unpack('<d', data[64:72])[0]
            imu_angular_x = struct.unpack('<d', data[72:80])[0]
            imu_angular_y = struct.unpack('<d', data[80:88])[0]
            imu_angular_z = struct.unpack('<d', data[88:96])[0]
            encoder1 = struct.unpack('<H', data[96:98])[0]
            encoder2 = struct.unpack('<H', data[98:100])[0]

            encoder_msg = Int64MultiArray()
            encoder_msg.data = [encoder1, encoder2]
            self.enocder_pub.publish(encoder_msg)

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.base_frame

            imu_msg.orientation.x = imu_x
            imu_msg.orientation.y = imu_y
            imu_msg.orientation.z = imu_z
            imu_msg.orientation.w = imu_w  # (left as-is since you said don't change code)

            imu_msg.angular_velocity.x = imu_angular_x
            imu_msg.angular_velocity.y = imu_angular_y
            imu_msg.angular_velocity.z = imu_angular_z
            imu_msg.linear_acceleration.x = imu_linear_x
            imu_msg.linear_acceleration.y = imu_linear_y
            imu_msg.linear_acceleration.z = imu_linear_z

            imu_msg.orientation_covariance[0] = -1.0
            imu_msg.angular_velocity_covariance=[
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02
            ]
            imu_msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1
                ]

            self.Imu_pub.publish(imu_msg)

            return {
                'imu': {'w': imu_w, 'x': imu_x, 'y': imu_y, 'z': imu_z},
                'encoder1': encoder1,
                'encoder2': encoder2,
                'valid': True
            }

        except struct.error as e:
            print(f"Error parsing data: {e}")
            return None

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")


def main():
    PORT = '/dev/ttyAMA0'
    BAUDRATE = 115200

    receiver = None  
    rclpy.init()

    try:
        receiver = STM32_UART_Receiver(port=PORT, baudrate=BAUDRATE)

        print("Waiting for data from STM32...")
        print("-" * 70)

        while True:
            # optional but harmless: lets ROS process internal stuff
            rclpy.spin_once(receiver, timeout_sec=0.0)

            data = receiver.receive_data()
            if data and data['valid']:
                #print(f"IMU Quaternion:")
                #print(f"  w: {data['imu']['w']:+.6f}")
                #print(f"  x: {data['imu']['x']:+.6f}")
                #print(f"  y: {data['imu']['y']:+.6f}")
                #print(f"  z: {data['imu']['z']:+.6f}")
                #print(f"Encoder 1: {data['encoder1']}")
                #print(f"Encoder 2: {data['encoder2']}")
                #print("-" * 70)
                donothing=1
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopped by user")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if receiver is not None:
            receiver.close()
            receiver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
