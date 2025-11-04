import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose             
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from rclpy.timer import Timer
from time import sleep
# NEED FILE PROCESSING 
#NEED to edit config file to have higher tolerances for avoiding stuff
# NEED edit orientation to make navigation smoother
class AutoNav(Node):
    def __init__(self):
        super().__init__('auto_nav')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.prox_front=self.create_subscription(Range,'/ultrasonic/front',self.proximity_callback,10)#msg type , topic, callback function,10)
        self.prox_back=self.create_subscription(Range,'/ultrasonic/back',self.proximity_callback,10)
        self.goals= ((4.94 ,0.138),(-1.83,0.13),(-10.68,-0.136),
                    (4.92,5.00),(-1.61,4.59),(-10.43,4.59),
                    (4.91,9.66),(-1.84,9.01),(-9.30,9.30))# len for size
        self.goal_in_progress=False
        self.goal_index=0
        self.cycle()
        self.orientation=1.0
        self.interaction=False
    def cycle(self):
        if self.goal_index==len(self.goals)-1:
            self.goal_index=0   
        if not self.goal_in_progress:
            x=self.goals[self.goal_index][0]
            y=self.goals[self.goal_index][1]
            print(f"about to send {x} and {y}")
            self.send_nav_goal(x,y)

    def send_nav_goal(self,x, y):
        # goal creation 
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x =x
        goal.pose.pose.position.y =y
        goal.pose.pose.orientation.w = 1.0
        print(f"Sending nav2 a goal {x},{y}")
        self.nav_client.wait_for_server()# wait until the action server is available
        send_future=self.nav_client.send_goal_async(goal)# send the goal
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        goal_handle =future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("goal rejected by nav2")
            self.goal_in_progress=False
            return
        self.get_logger().info("Goal accepted navigating")
        self.goal_in_progress=True
        result_future=goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self,future):
        result=future.result().result# waits for nav2 to complete goal
        self.get_logger().info(f"Goal completed with result: {result}")
        self.goal_index+=1
        self.goal_in_progress=False
        self.cycle()


    def proximity_callback(self,sensor): # make proximity for interaction detetection a foot away from robot for now
        direction=sensor.header.frame_id
        if sensor.range< 0.3048:
            self.goal_in_progress=False
            self.get_logger().info(f"Person detected close to {direction}")
            self.get_logger().info("Cancelling nav2 destination and waiting for interaction")
            cancel_future=self.nav_client._cancel_goal_async(self._goal_handle)
            self.get_logger().info("Waiting for customer to finish interaction")
            self.resume_timer = self.create_timer(5.0, self.resume_navigation) # wait for interaction and is non blocking

    def resume_navigation(self):
        self.resume_timer.cancel()# cancel timer 
        if self.interaction:
            self.get_logger().info("Waiting for customer to finish interaction")
        else:
            self.get_logger().info("No interaction continue navigation")
            self.send_nav_goal(self.goals[self.goal_index][0],self.goals[self.goal_index][1])
def main():
    rclpy.init()
    nav_node = AutoNav()
    rclpy.spin(nav_node)
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
        