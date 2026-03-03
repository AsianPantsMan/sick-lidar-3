import serial
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.timer import Timer
import struct 
import math as mt
import numpy as np
class PiToMCUNode(Node):
    def __init__(self):
        super().__init__('pi_to_mcu_node')
        self.cmd_sub=self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.dist_between=0.23
        self.sprocket_radius=0.06
        self.port='/dev/ttyAMA0'
        self.baudrate=38400
        self.ser=serial.Serial(port=self.port,baudrate=self.baudrate,)
    def cmd_vel_callback(self,msg):
        linear_x=msg.linear.x# m/s
        angular_z=msg.angular.z# rad/s
        ppr=1993.6
        max_qqps=9570*.7# 70 percent of tuned max qqps 
        ########### TURN into sperate wheel speeds to send to MCU####
        vl=linear_x-((angular_z*self.dist_between)/2)
        vr=linear_x+((angular_z*self.dist_between)/2)
        meters_per_revolution=2*mt.pi*self.sprocket_radius
        rps_r=vr/meters_per_revolution
        rps_l=vl/meters_per_revolution
        ticks_rs=np.int32(rps_r*ppr*4)
        ticks_ls=np.int32(rps_l*ppr*4)
        ticks_rs=int(np.clip(ticks_rs,-max_qqps,max_qqps))
        ticks_ls=int(np.clip(ticks_ls,-max_qqps,max_qqps))
        tick_msg=struct.pack('<BiiB',0xAA,int(ticks_ls),int(ticks_rs),0x55)
        #self.ser.write(tick_msg)
        print(f"Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}")
        print(f"Ticks: Left={ticks_ls}, Right={ticks_rs}")
    import serial
import struct
import time

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
    return ser.read(1) == b'\xFF'

def rc_read_exact(ser: serial.Serial, n: int) -> bytes:
    buf = ser.read(n)
    if len(buf) != n:
        raise TimeoutError(f"Timed out reading {n} bytes (got {len(buf)})")
    return buf

def set_m1_speed(ser: serial.Serial, address: int, speed: int) -> bool:
    body = bytes([address, 35]) + struct.pack(">i", int(speed))
    return rc_write_packet(ser, body)

def set_m2_speed(ser: serial.Serial, address: int, speed: int) -> bool:
    body = bytes([address, 36]) + struct.pack(">i", int(speed))
    return rc_write_packet(ser, body)

def read_encoder_count(ser: serial.Serial, address: int, cmd: int):
    body = bytes([address, cmd])
    ser.write(body + struct.pack(">H", crc16_ccitt(body)))

    resp = rc_read_exact(ser, 7)
    count = struct.unpack(">i", resp[0:4])[0]
    status = resp[4]
    crc_rx = struct.unpack(">H", resp[5:7])[0]

    if crc16_ccitt(body + resp[0:5]) != crc_rx:
        raise ValueError("CRC mismatch")
    return count, status

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def ramp_to_speed(set_speed_fn, target, *,
                  current=0,
                  step=250,
                  period=0.05,
                  max_abs=6700):
    target = clamp(int(target), -max_abs, max_abs)
    v = int(current)

    while v != target:
        if v < target:
            v = min(v + step, target)
        else:
            v = max(v - step, target)

        if not set_speed_fn(v):
            set_speed_fn(0)
            raise RuntimeError("RoboClaw ACK failed during ramp")
        time.sleep(period)

    return v


    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        # Safety: stop both channels at start
        set_m1_speed(ser, ADDRESS, 0)
        set_m2_speed(ser, ADDRESS, 0)

        current = 0
        print("Starting one-motor test (M1)...")

        try:
            for target in speed_sequence:
                print(f"\nRamping to {target} QPPS")
                current = ramp_to_speed(lambda v: set_m1_speed(ser, ADDRESS, v),
                                        target, current=current)

                # Hold for 3 seconds and print encoder + delta
                t0 = time.time()
                last_count, _ = read_encoder_count(ser, ADDRESS, 16)
                last_t = time.time()

                while time.time() - t0 < 3.0:
                    count, _ = read_encoder_count(ser, ADDRESS, 16)
                    now = time.time()
                    dt = now - last_t
                    qpps_meas = (count - last_count) / dt if dt > 0 else 0.0
                    print(f"M1_count={count}  ~meas={qpps_meas:.0f} QPPS")
                    last_count, last_t = count, now
                    time.sleep(0.2)

        except KeyboardInterrupt:
            print("\nCtrl+C: stopping...")

        finally:
            set_m1_speed(ser, ADDRESS, 0)
            set_m2_speed(ser, ADDRESS, 0)
            print("Done.")
def main():
    rclpy.init()
    node= PiToMCUNode()
    try:   
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__=="__main__":
    PORT = "/dev/ttyACM0"   # or "/dev/ttyACM0" if USB
    BAUD = 38400
    ADDRESS = 128
    main()