#!/usr/bin/env python3
import serial
import struct
import rclpy
import math as mt
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import Imu


class STM32_UART_Receiver(Node):
    FRAME_START = 0xAA
    FRAME_END = 0x55
    PAYLOAD_LEN = 100
    FRAME_LEN = 1 + PAYLOAD_LEN + 1  # start + payload + end

    # 12 doubles + 2 uint16 = 100 bytes
    PAYLOAD_STRUCT = struct.Struct('<12d2H')

    def __init__(self, port='/dev/ttyAMA0', baudrate=115200, timeout=0.02):
        super().__init__('stm32_uart_receiver')

        self.base_frame = 'base_link'
        self.buffer = bytearray()

        self.imu_pub = self.create_publisher(Imu, '/stm32_imu', 10)
        self.encoder_pub = self.create_publisher(Int64MultiArray, '/stm32_encoder', 10)

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected to {port} at {baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            raise

        # Timer-driven, but fast enough to keep up with incoming UART
        # 0.002 = 500 Hz callback scheduling, but work done only when bytes exist
        self.timer = self.create_timer(0.002, self.read_and_process_serial)

    def read_and_process_serial(self):
        try:
            # Read all currently available bytes, or at least 1 byte to allow blocking behavior
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting > 0 else 1)

            if chunk:
                self.buffer.extend(chunk)

            # Keep extracting frames while complete ones exist in the buffer
            while True:
                frame = self.extract_frame()
                if frame is None:
                    break
                self.handle_frame(frame)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in serial processing: {e}")

    def extract_frame(self):
        if not self.buffer:
            return None

        # Find the next start marker
        start_index = self.buffer.find(bytes([self.FRAME_START]))

        if start_index == -1:
            # No valid start byte anywhere; clear junk
            self.buffer.clear()
            return None

        # Discard junk before the start marker
        if start_index > 0:
            del self.buffer[:start_index]

        # Need a full frame before continuing
        if len(self.buffer) < self.FRAME_LEN:
            return None

        # Candidate frame is now at the front
        if self.buffer[0] != self.FRAME_START:
            del self.buffer[0]
            return None

        if self.buffer[self.FRAME_LEN - 1] != self.FRAME_END:
            # Misaligned or bad frame: discard one byte and resync
            del self.buffer[0]
            return None

        frame = bytes(self.buffer[:self.FRAME_LEN])
        del self.buffer[:self.FRAME_LEN]
        return frame

    def handle_frame(self, frame):
        payload = frame[1:-1]  # remove start and end markers

        try:
            (
                imu_w, imu_x, imu_y, imu_z,
                imu_linear_w, imu_linear_x, imu_linear_y, imu_linear_z,
                imu_angular_w, imu_angular_x, imu_angular_y, imu_angular_z,
                encoder1, encoder2
            ) = self.PAYLOAD_STRUCT.unpack(payload)

            # Convert angular values from deg/s to rad/s
            imu_angular_w *= mt.pi / 180.0
            imu_angular_x *= mt.pi / 180.0
            imu_angular_y *= mt.pi / 180.0
            imu_angular_z *= mt.pi / 180.0

            self.publish_encoder(encoder1, encoder2)
            self.publish_imu(
                imu_w, imu_x, imu_y, imu_z,
                imu_linear_x, imu_linear_y, imu_linear_z,
                imu_angular_x, imu_angular_y, imu_angular_z
            )

        except struct.error as e:
            self.get_logger().warn(f"Error parsing frame: {e}")

    def publish_encoder(self, encoder1, encoder2):
        encoder_msg = Int64MultiArray()
        encoder_msg.data = [int(encoder1), int(encoder2)]
        self.encoder_pub.publish(encoder_msg)

    def publish_imu(
        self,
        imu_w, imu_x, imu_y, imu_z,
        lin_x, lin_y, lin_z,
        ang_x, ang_y, ang_z
    ):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.base_frame

        imu_msg.orientation.x = imu_x
        imu_msg.orientation.y = imu_y
        imu_msg.orientation.z = imu_z
        imu_msg.orientation.w = imu_w

        imu_msg.angular_velocity.x = ang_x
        imu_msg.angular_velocity.y = ang_y
        imu_msg.angular_velocity.z = ang_z

        imu_msg.linear_acceleration.x = lin_x
        imu_msg.linear_acceleration.y = lin_y
        imu_msg.linear_acceleration.z = lin_z

        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]
        imu_msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        self.imu_pub.publish(imu_msg)

    def close(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed")


def main():
    rclpy.init()
    receiver = None

    try:
        receiver = STM32_UART_Receiver(
            port='/dev/ttyAMA0',
            baudrate=115200,
            timeout=0.02
        )
        rclpy.spin(receiver)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if receiver is not None:
            receiver.close()
            receiver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()