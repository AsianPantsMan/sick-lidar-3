import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import math as mt
from std_msgs.msg import Int64MultiArray


class MCUToPiNode(Node):
    def __init__(self):
        super().__init__('MCU_to_PI_node')

        ##### Parameters
        self.declare_parameter('ticks_per_rev', 1993.6)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)

        self.declare_parameter('radius_m', 0.06)
        self.radius_m = float(self.get_parameter('radius_m').value)

        self.declare_parameter('track_width_m', 0.3)
        self.track_width_m = float(self.get_parameter('track_width_m').value)

        self.refresh_rate_hz = 0.0

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.declare_parameter('publish_tf', False)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_z = 0.0
        self.last_imu_gyro_z = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 5)
        self.Imu_pub = self.create_publisher(Imu, '/imu/data', 5)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.radians_per_tick = (2.0 * mt.pi) / float(self.ticks_per_rev)
        self.get_logger().info("Wheel odom node has been started")

        # Callback-based subscriptions
        self.imu_read = self.create_subscription(
            Imu, '/stm32_imu', self.imu_callback, 5
        )
        self.encoder_read = self.create_subscription(
            Int64MultiArray, '/stm32_encoder', self.encoder_callback, 5
        )

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = mt.sin(yaw / 2.0)
        q.w = mt.cos(yaw / 2.0)
        return q

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return mt.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg):
        self.imu_update(msg)

    def encoder_callback(self, msg):
        self.Odom_update(msg)

    def Odom_update(self, msg):
        now = self.get_clock().now()
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.refresh_rate_hz = 1.0 / dt if dt > 0.0 else 0.0

        if dt <= 0.0:
            return

        delta_left_ticks = ((left_ticks - self.last_left_ticks + 32768) % 65536) - 32768
        delta_right_ticks = ((right_ticks - self.last_right_ticks + 32768) % 65536) - 32768

        delta_left_angle = self.radians_per_tick * float(delta_left_ticks)
        delta_right_angle = self.radians_per_tick * float(delta_right_ticks)

        track_left = delta_left_angle / dt
        track_right = delta_right_angle / dt

        velocity_left = self.radius_m * track_left
        velocity_right = self.radius_m * track_right

        velocity = (velocity_left + velocity_right) / 2.0
        angular_velocity = self.last_imu_gyro_z

        ds = velocity * dt

        # Use latest IMU yaw
        self.theta = self.imu_z
        self.x += ds * mt.cos(self.theta)
        self.y += ds * mt.sin(self.theta)

        self.theta = mt.atan2(mt.sin(self.theta), mt.cos(self.theta))
        q = self.yaw_to_quaternion(self.theta)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = float(velocity)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(angular_velocity)

        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        odom.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 100000.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 100000.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 100000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 100000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        self.odom_pub.publish(odom)

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = now

    def imu_update(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_frame"

        imu_msg.orientation = msg.orientation
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration

        self.last_imu_gyro_z = msg.angular_velocity.z
        self.imu_z = self.quaternion_to_yaw(msg.orientation)

        imu_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]
        imu_msg.linear_acceleration_covariance = [
            0.20, 0.0, 0.0,
            0.0, 0.20, 0.0,
            0.0, 0.0, 0.20
        ]

        self.Imu_pub.publish(imu_msg)


def main():
    rclpy.init()
    node = MCUToPiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()