import serial
import time
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
import struct 
import math as mt
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
        ########### TURN into sperate wheel speeds to send to MCU####
        vl=linear_x-((angular_z*self.dist_between)/2)
        vr=linear_x+((angular_z*self.dist_between)/2)
        meters_per_revolution=2*mt.pi*self.sprocket_radius
        ppr=1993.6
        gear_ratio=(1+(46/17)) * (1+(46/17)) * (1+(46/11))
        ticks_r=(vr/(meters_per_revolution)*ppr*4*gear_ratio)
        ticks_l=(vl/(meters_per_revolution)*ppr*4*gear_ratio)
        ticks=struct.pack('<BiiB',0xAA,int(ticks_l),int(ticks_r),0x55)
        self.ser.write(ticks)
        print(f"Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}")
        print(f"Ticks: Left={ticks_l}, Right={ticks_r}")
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
    main()