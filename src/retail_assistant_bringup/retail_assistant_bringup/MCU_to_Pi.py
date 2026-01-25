import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.timer import Timer
import math as mt # What 
# Tread length wheel to wheel 14.5 inches 
# Tread from top is 12 inches
# Tread is 2 inches wide
class MCUToPiNode(Node):
    def __init__(self):
        super().__init__('MCU_to_PI_node')
        ##### Parameters
        self.declare_parameter('ticks_per_rev',2048)#Default 2048 ticks per revolution
                                                    #Must declare parameters in ros if want to overide with yaml files
        self.ticks_per_rev=int(self.get_parameter('ticks_per_rev').value)# get parameter value for to be int

        self.declare_parameter('radius_m',0.05) # Default radius in meters| effecitve radius of rotating part encoder measure
        self.radius_m=float(self.get_parameter('radius_m').value) #use drive sporket pitch radius or calibrate with meters_per_tick
        
        # distance between left and right track center lines
        self.declare_parameter('track_width_m',0.3)
        self.track_widht_m=float(self.get_parameter('track_width_m').value)

        # Frame names
        self.declare_parameter('odom_frame','odom')# Frame relaitve to the start
        self.declate_parameter('base_frame','base_link')# Relative to itself
        self.odom_frame=str(self.get_parameter('odom_frame').value)
        self.base_frame=str(self.get_parameter('base_frame').value) 
        self.declare_parameter('publish_tf',True)
        self.publish_tf=bool(self.get_parameter('publish_tf').value)

        self.x=0.0 # Robot position in x(starting at 0)
        self.y=0.0 # Robot position in y(starting at 0)
        self.theta=0.0 # Robot orientation (starting at 0 radians)

        # Last encoder readings
        self.last_left_ticks=None #initialize last ticks to none to detect first time
        self.last_right_ticks=None
        self.last_time=None
        self.odom_pub=self.create_publisher(Odometry'/odom',10)# Publisher for odometry
        self.tf_broadcaster=TransformBroadcaster(self)# TF broadcaster for coordinate frames

        self.rad_per_tick=(2.0*mt.pi)/float(self.ticks_per_rev)# Radians per tick for wheel rotation
        self.get_logger().info("Wheel odom node has been started")

    def yaw_to_quaternion(self,yaw):  
        q=Quaternion()
        q.x=0.0
        q.y=0.0
        q.z=math.sin(yaw/2.0)
        q.w=math.cos(yaw/2.0)
        return q # ROS compatible quaternion from yaw angle

    def Odom_update(self,left_ticks,right_ticks):# Help from chat and github for differential drive odometry
        now=self.get_clock().now()# gets current time

        if self.last_left_ticks is None:# first time function is called set grounds for previous and current
            self.last_left_ticks=left_ticks
            self.last_right_ticks=right_ticks
            self.last_time=now
            return
        
        
        dt=(now-self.last_time).nanoseconds*1e-9# time difference in nanoseconds
        if dt<=0.0:
            return #prevent division by zero or negative time
        delta_left_ticks=left_ticks -self.last_left_ticks# Change in left and right decoder ticks
        delta_right_ticks=right_ticks -self.last_right_ticks
        delta_left_angle=self.radians_per_tick * float(delta_left_ticks)# Change in left and right wheel angles
        delta_right_angle=self.radians_per_tick * float(delta_right_ticks)
        #################Angular velocity of wheels##################
        track_left=delta_left_angle/dt #Angular velocity of left wheel
        track_right=delta_right_angle/dt #Angular velocity of right wheel
        #################Linear velocity of wheels##################
        velocity_left=self.radius_m*track_left #Linear velocity of left wheel
        velocity_right=self.radius_m*track_right #Linear velocity of right wheel
       