import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from sensor_msgs.msg import Range
import rclpy

class ultrasonicSensor(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        self.front_pub = self.create_publisher(Range, 'ultrasonic/back', 10)
        self.timer = self.create_timer(0.1, self.front_distance_callback)
# GPIO pin numbers (BCM)
        self.front_TRIG = 17
        self.front_ECHO = 27
        #green/white =3
        #bluew/brown=2

        #blue=20
        # white=21
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.front_TRIG, GPIO.OUT)
        GPIO.setup(self.front_ECHO, GPIO.IN)

# Make sure trigger is low
        GPIO.output(self.front_TRIG, False)
        time.sleep(2)

    def front_distance_callback(self):
    # Send trigger pulse
        pulse_end=None
        pulse_start=None
        GPIO.output(self.front_TRIG, True)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.front_TRIG, False)
        timeout_s=0.035
        t0 = time.time()
    # Wait for echo to go high
        while GPIO.input(self.front_ECHO) == 0:
            if time.time() - t0 > timeout_s:
                self.get_logger().warn("Front ultrasonic sensor timeout waiting for echo high")
                return
            pulse_start = time.time()

    # Wait for echo to go low
        while GPIO.input(self.front_ECHO) == 1:
            if time.time() - t0 > timeout_s:
                self.get_logger().warn("Front ultrasonic sensor timeout waiting for echo low")
                return
            pulse_end = time.time()

    # Time difference
        if pulse_start is None or pulse_end is None:
            self.get_logger().warn("Front ultrasonic sensor failed to get valid pulse")
            return
        pulse_duration = pulse_end - pulse_start

    # Speed of sound = 34300 cm/s
        distance = pulse_duration * 34300 / 2
        self.publish_front(distance)
    def publish_front(self, distance):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_1_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1  # radians
        msg.min_range = 0.02  # meters
        msg.max_range = 4.0   # meters
        msg.range = distance / 100.0  # convert cm to meters
        self.front_pub.publish(msg)
def main():
    rclpy.init()
    node=ultrasonicSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Measurement stopped")
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
