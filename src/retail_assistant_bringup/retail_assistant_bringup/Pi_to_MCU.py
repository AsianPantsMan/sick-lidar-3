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
        self.baudrate=115200
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
        self.ser.write(tick_msg)
        print(f"Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}")
        print(f"Ticks: Left={ticks_ls}, Right={ticks_rs}")
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