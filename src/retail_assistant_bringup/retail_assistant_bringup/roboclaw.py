import serial, struct, time, math as mt
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray

# --- RoboClaw CRC + Packet Serial helpers ---

def crc16_ccitt(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def rc_write_packet(ser: serial.Serial, body: bytes) -> bool:
    crc = crc16_ccitt(body)
    ser.write(body + struct.pack(">H", crc))
    ack = ser.read(1)
    return ack == b'\xFF'

def set_m1_speed(ser: serial.Serial, address: int, speed: int) -> bool:
    body = bytes([address, 35]) + struct.pack(">i", int(speed))
    return rc_write_packet(ser, body)

def set_m2_speed(ser: serial.Serial, address: int, speed: int) -> bool:
    body = bytes([address, 36]) + struct.pack(">i", int(speed))
    return rc_write_packet(ser, body)

def rc_read_exact(ser: serial.Serial, n: int) -> bytes:
    buf = ser.read(n)
    if len(buf) != n:
        raise TimeoutError(f"Timed out reading {n} bytes (got {len(buf)})")
    return buf

def read_encoder_count(ser: serial.Serial, address: int, cmd: int):
    body = bytes([address, cmd])
    ser.write(body + struct.pack(">H", crc16_ccitt(body)))

    resp = rc_read_exact(ser, 7)                # 4 count + 1 status + 2 CRC
    count = struct.unpack(">i", resp[0:4])[0]
    status = resp[4]
    crc_rx = struct.unpack(">H", resp[5:7])[0]

    if crc16_ccitt(body + resp[0:5]) != crc_rx:
        raise ValueError("CRC mismatch")
    return count, status


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# --- ROS2 Node ---

class PiToRoboClawNode(Node):
    def __init__(self):
        super().__init__("pi_to_roboclaw_node")
        self.encoder_pub = self.create_publisher(Int64MultiArray, "/roboclaw_encoder", 10)
        self.CMD_READ_M1_ENCODER = 16
        self.CMD_READ_M2_ENCODER = 17
        # Robot params
        self.dist_between = 0.23
        self.sprocket_radius = 0.06
        self.ppr = 1993.6

        # RoboClaw params
        self.ADDRESS = 128
        self.MAX_QPPS = int(9570 * 0.7)
        self._enc_div = 0   # divider to read encoders every N ticks since it’s a bit slow
        # Ramping params
        self.step = 250          # qpps per tick
        self.period = 0.02       # 50 Hz
        self.current_left = 0
        self.current_right = 0
        self.target_left = 0
        self.target_right = 0

        # Serial
        self.port = "/dev/ttyACM0"
        self.baudrate = 38400
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.05,        # IMPORTANT so reads don’t hang forever
            write_timeout=0.1,
        )

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.timer = self.create_timer(self.period, self.control_tick)

        self.get_logger().info(f"RoboClaw serial open on {self.port} @ {self.baudrate}")

    def cmd_vel_callback(self, msg: Twist):# converts cmd_vel to target qpps for each motor, called on each new cmd_vel message
        v = msg.linear.x
        w = msg.angular.z

        vl = v - (w * self.dist_between / 2.0)
        vr = v + (w * self.dist_between / 2.0)

        meters_per_rev = 2.0 * mt.pi * self.sprocket_radius
        rps_l = vl / meters_per_rev
        rps_r = vr / meters_per_rev

        qpps_l = int(rps_l * self.ppr)
        qpps_r = int(rps_r * self.ppr)

        self.target_left = int(np.clip(qpps_l, -self.MAX_QPPS, self.MAX_QPPS))
        self.target_right = int(np.clip(qpps_r, -self.MAX_QPPS, self.MAX_QPPS))

    def control_tick(self):# called every 20ms to ramp toward target speeds and send to RoboClaw
        # ramp one step toward target each tick
        self.current_left = self._step_toward(self.current_left, self.target_left, self.step)
        self.current_right = self._step_toward(self.current_right, self.target_right, self.step)

        ok1 = set_m1_speed(self.ser, self.ADDRESS, self.current_left)
        ok2 = set_m2_speed(self.ser, self.ADDRESS, self.current_right)

        if not (ok1 and ok2):
            # If ACK fails, stop and warn (don’t hard-crash the node instantly)
            set_m1_speed(self.ser, self.ADDRESS, 0)
            set_m2_speed(self.ser, self.ADDRESS, 0)
            self.current_left = 0
            self.current_right = 0
            self.get_logger().warn("RoboClaw ACK failed — commanded 0")
            return
        #### encoder reading every 5 ticks (100ms) since it’s a bit slow and we don’t need it super high rate####
        self._enc_div += 1
        if self._enc_div >= 5:
            self._enc_div = 0
            try:
                left_count, _ = read_encoder_count(self.ser, self.ADDRESS, self.CMD_READ_M1_ENCODER)
                right_count, _ = read_encoder_count(self.ser, self.ADDRESS, self.CMD_READ_M2_ENCODER)

                msg = Int64MultiArray()
                msg.data = [int(left_count), int(right_count)]
                self.encoder_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Encoder read failed: {e}")


    @staticmethod
    def _step_toward(curr: int, target: int, step: int) -> int:# slowly ramp function retrurns current qpps value
        if curr < target:
            return min(curr + step, target)
        if curr > target:
            return max(curr - step, target)
        return curr

    def destroy_node(self):
        # stop motors once on shutdown
        try:
            set_m1_speed(self.ser, self.ADDRESS, 0)
            set_m2_speed(self.ser, self.ADDRESS, 0)
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = PiToRoboClawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()