import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.timer import Timer
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformStamped
from geometry_msgs.msg import Quaternion
from rclpy.timer import Timer
import math as mt 
from std_msgs.msg import Int64MultiArray
# Tread length wheel to wheel 14.5 inches 
# Tread from top is 12 inches
# Tread is 2 inches wide
class MCUToPiNode(Node):
    def __init__(self):
        super().__init__('MCU_to_PI_node')
        ##### Parameters
        self.declare_parameter('ticks_per_rev',1425.1)#Default 2048 ticks per revolution
                                                    #Must declare parameters in ros if want to overide with yaml files
        self.ticks_per_rev=int(self.get_parameter('ticks_per_rev').value)# get parameter value for to be int

        self.declare_parameter('radius_m',0.06) # Default radius in meters| effecitve radius of rotating part encoder measure
        self.radius_m=float(self.get_parameter('radius_m').value) #use drive sporket pitch radius or calibrate with meters_per_tick
        
        # distance between left and right track center lines
        self.declare_parameter('track_width_m',0.23)# Default track width in meters
        self.track_width_m=float(self.get_parameter('track_width_m').value)

        # Frame names
        self.declare_parameter('odom_frame','odom')# Frame relaitve to the start
        self.declare_parameter('base_frame','base_link')# Relative to itself 
        self.odom_frame=str(self.get_parameter('odom_frame').value)
        self.base_frame=str(self.get_parameter('base_frame').value) 
        self.declare_parameter('publish_tf',True)
        self.publish_tf=bool(self.get_parameter('publish_tf').value)

        self.x=0.0 # Robot position in x(starting at 0)
        self.y=0.0 # Robot position in y(starting at 0)
        self.theta=0.0 # Robot orientation (starting at 0 radians)
        ### TO DO ADD SUBSCRIBER FOR STM32 comminication
        # Last encoder readings
        self.last_left_ticks=0 #initialize last ticks to none to detect first time
        self.last_right_ticks=0
        self.last_time=self.get_clock().now() # Time of last update
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)# Publisher for odometry
        self.Imu_pub=self.create_publisher(Imu,'/imu/data',10)# Publisher for IMU
        self.tf_broadcaster=TransformBroadcaster(self)# TF broadcaster for coordinate frames

        self.radians_per_tick=(2.0*mt.pi)/float(self.ticks_per_rev)# Radians per tick for wheel rotation
        self.get_logger().info("Wheel odom node has been started")
        self.test_timer=self.create_timer(10, self.test_odom) # For testing odometry updates
        self.test_imu_timer=self.create_timer(5,self.imu_update) # For testing IMU updates
        self.test_left_ticks = 0
        self.test_right_ticks = 0
        self.test_step = 0
        self.test_mode = "straight"
        self.imu_read=self.create_subscription(Imu,'/stm32_imu',self.imu_update,10) # Subscribing to imu topic from stm32
        self.encoder_read=self.create_subscription(Int64MultiArray,'/stm32_encoder',self.Odom_update,10)
    def yaw_to_quaternion(self,yaw):  
        q=Quaternion()
        q.x=0.0
        q.y=0.0
        q.z=mt.sin(yaw/2.0)
        q.w=mt.cos(yaw/2.0)
        return q # ROS compatible quaternion from yaw angle

    def Odom_update(self,msg):#  for differential drive odometry
        now=self.get_clock().now()# gets current time
        left_ticks=msg.data[0] # left encoder ticks
        right_ticks=msg.data[1] # right encoder ticks
        if self.last_left_ticks is None:# first time function is called set grounds for previous and current
            self.last_left_ticks=left_ticks
            self.last_right_ticks=right_ticks
            self.last_time=now
            return
        
        
        dt=(now-self.last_time).nanoseconds*1e-9# time difference in nanoseconds
        if dt<=0.0:
            return #prevent division by zero or negative time
        delta_left_ticks=((left_ticks -self.last_left_ticks+32678)%65536)-32768# Change in left and right decoder ticks
        delta_right_ticks=((right_ticks -self.last_right_ticks+32678)%65536)-32768
        delta_left_angle=self.radians_per_tick * float(delta_left_ticks)# Change in left and right wheel angles
        delta_right_angle=self.radians_per_tick * float(delta_right_ticks)
        #################Angular velocity of wheels##################
        track_left=delta_left_angle/dt #Angular velocity of left wheel
        track_right=delta_right_angle/dt #Angular velocity of right wheel
        #################Linear velocity of wheels##################
        velocity_left=self.radius_m*track_left #Linear velocity of left wheel
        velocity_right=self.radius_m*track_right #Linear velocity of right wheel
        #################Linear and angular velocity of robot#######
        velocity=(velocity_left + velocity_right) / 2.0 # Average linear velocity
        angular_velocity=(velocity_right - velocity_left) / self.track_width_m # Angular velocity of
        ds =velocity * dt # Distance traveled in this time step
        dtheta=angular_velocity * dt # Change in orientation

        self.x+= ds*mt.cos(self.theta)# distance travled in x direction
        self.y+= ds*mt.sin(self.theta)# distance traveled in y direction
        self.theta += dtheta

        self.theta=mt.atan2(mt.sin(self.theta), mt.cos(self.theta)) # Normalize theta to [-pi, pi]

        q=self.yaw_to_quaternion(self.theta) # Convert yaw to quaternion

        if self.publish_tf:
            t=TransformStamped()# Ctreate Tf message which says where one frame is relative to another
            t.header.stamp=now.to_msg()# When the tf is available
            t.header.frame_id=self.odom_frame #Parent frame
            t.child_frame_id=self.base_frame
            t.transform.translation.x=float(self.x)### position orienation of base_link relative to odom
            t.transform.translation.y=float(self.y)
            t.transform.translation.z=0.0
            t.transform.rotation=q
            self.tf_broadcaster.sendTransform(t) # Send the transform   if self.publish_tf:

        odom= Odometry()# make odometry message object for topic
        odom.header.stamp=now.to_msg()
        odom.header.frame_id=self.odom_frame
        odom.child_frame_id=self.base_frame
        odom.pose.pose.position.x=float(self.x)# postion and orientation of robot
        odom.pose.pose.position.y=float(self.y)
        odom.pose.pose.position.z=0.0
        odom.pose.pose.orientation=q
        odom.twist.twist.linear.x=float(velocity)
        odom.twist.twist.linear.y=0.0
        odom.twist.twist.angular.z=float(angular_velocity)
        odom.pose.covariance = [
    0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,  0.1
                                    ] # Placeholder covariance values
        odom.twist.covariance = [
    0.1, 0.0,  0.0,  0.0,  0.0,  0.0,
    0.0, 0.1,  0.0,  0.0,  0.0,  0.0,
    0.0, 0.0,  1e6,  0.0,  0.0,  0.0,
    0.0, 0.0,  0.0,  1e6,  0.0,  0.0,
    0.0, 0.0,  0.0,  0.0,  1e6,  0.0,
    0.0, 0.0,  0.0,  0.0,  0.0,  0.2
                                ]  # Placeholder covariance values
        self.odom_pub.publish(odom)

        self.last_left_ticks=left_ticks  # Update last ticks and time
        self.last_right_ticks=right_ticks
        self.last_time=now
    def test_odom(self):
    # Reset test state
        self.test_left_ticks = 0
        self.test_right_ticks = 0
        self.test_step = 0
        self.test_mode = "straight"   # change to "spin" or "arc" to test others

    # Cancel one-shot timer if you're using one
        if hasattr(self, "test_timer"):
            self.test_timer.cancel()

    # Run at 50 Hz
        self.test_timer = self.create_timer(0.05, lambda: self.Odom_update(0,0))


    def _test_odom_step(self):

    # Stop test after ~6 seconds
        if self.test_step >= 300:
            self.get_logger().info("Odom test complete.")
            self.test_timer.cancel()
            return

        step_ticks = 20

    # ----- MOTION PROFILES -----
        if self.test_mode == "straight":
            dl = step_ticks
            dr = step_ticks

        elif self.test_mode == "spin":
            dl = step_ticks
            dr = -step_ticks

        elif self.test_mode == "arc":
            dl = step_ticks
            dr = int(step_ticks * 0.6)

        else:
            dl = step_ticks
            dr = step_ticks

    # Accumulate total ticks (absolute tick simulation)
        self.test_left_ticks += dl
        self.test_right_ticks += dr

    # Call your real odom function
        self.Odom_update(self.test_left_ticks, self.test_right_ticks)

        self.test_step += 1

    def imu_update(self,msg):
        # orientation xyzw
        # angular velocity xyz
        # linear acceleration xyz
        # get imu from port 
        self.test_imu_timer.cancel()  # Cancel the timer after first call
        orientation_x=msg.orientation.x 
        orientation_y=msg.orientation.y
        orientation_z=msg.orientation.z
        orientation_w=msg.orientation.w
        angular_velocity_x=float(0)# Dummy values for now 
        angular_velocity_y=float(0)
        angular_velocity_z=float(0)
        linear_acceleration_x=float(0)
        linear_acceleration_y=float(0)
        linear_acceleration_z=float(0)
        imu_msg = Imu()# IMU message object for topic
        imu_msg.header.stamp=self.get_clock().now().to_msg()
        imu_msg.header.frame_id=self.base_frame
        imu_msg.orientation.x=orientation_x
        imu_msg.orientation.y=orientation_y
        imu_msg.orientation.z=orientation_z
        imu_msg.orientation.w=orientation_w
        imu_msg.angular_velocity.x=angular_velocity_x
        imu_msg.angular_velocity.y=angular_velocity_y
        imu_msg.angular_velocity.z=angular_velocity_z
        imu_msg.linear_acceleration.x=linear_acceleration_x
        imu_msg.linear_acceleration.y=linear_acceleration_y
        imu_msg.linear_acceleration.z=linear_acceleration_z
        imu_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
            ]  # Indicate orientation not available Loook at convriances later 
        imu_msg.angular_velocity_covariance=[# ignore things that are not diagnoals dont know correlations between axes
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]  #convariance resembles how accurate the measurement is
        imu_msg .linear_acceleration_covariance=[0.20, 0.0, 0.0,
                                            0.0, 0.20, 0.0,
                                            0.0, 0.0, 0.20]# acceleratomer more noisy
        self.Imu_pub.publish(imu_msg)  # Publish the IMU message

def main():
    rclpy.init()
    node=MCUToPiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()