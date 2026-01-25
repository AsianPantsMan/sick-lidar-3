import serial
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.timer import Timer
import math as mt
class PiToMCUNode(Node):
    def __init__(self):
        super().__init__('pi_to_mcu_node')
        self.cmd_sub=self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
    
    def cmd_vel_callback(self,msg):
        linear_x=msg.linear.x# m/s
        angular_z=msg.angular.z# rad/s
        print(f"Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}")
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