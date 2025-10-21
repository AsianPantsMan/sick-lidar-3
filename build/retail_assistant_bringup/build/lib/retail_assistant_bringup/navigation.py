import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose             
from geometry_msgs.msg import PoseStamped

class AutoNav(Node):
    def __init__(self):
        super().__init__('auto_nav')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_dict= {"Coordinate1":(4.94 ,0.138),"Coordinate2":(-1.83,0.13),"Coordinate3":(-10.68,-0.136),
                         "Coordinate4":(4.92,5.00), "Coordinate5":(-1.61,4.59), "Coordinate6": (-10.43,4.59),
                         "Coordinate7":(4.91,9.66), "Coordinate8":(-1.84,9.01), "Coordinate9":(-9.30,9.30)}# len for size
        self.cycle()
        
    def cycle(self):
        for key, (x,y) in self.goal_dict.items():
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
        result_future=goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self,future):
        result=future.result().result# waits for nav2 to complete goal
        self.get_logger().info(f"Goal completed with result: {result}")

def main():
    rclpy.init()
    nav_node = AutoNav()
    rclpy.spin(nav_node)
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
        