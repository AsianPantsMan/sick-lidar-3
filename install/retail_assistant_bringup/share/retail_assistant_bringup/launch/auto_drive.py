import rclpy
import math as mt
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
#node + subsscriber initalization
class MyMapNode(Node):
    robot_position=[0.0,0.0]
    def __init__(self):
        super().__init__('map_subscriber_node')
        # create_subscription(msg_type, topic, callback, queue_size)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)#Whenever /map type occupancy gird updated call function callback
        self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)#Topic responsible for robot position
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')# client for nav2 locations?

    #function subscribed and using pose topic made by slam toolbox to get robot position
    def pose_callback(self,msg):
        self.robot_position[0]=msg.pose.position.x # update robot position x
        self.robot_position[1]=msg.pose.position.y # update robot position y
        self.get_logger().info(f"Robot position from pose topic {self.robot_position}")

    def map_callback(self, msg):
        if self.robot_position == [0.0, 0.0]:
            self.get_logger().warn("Waiting for valid robot pose before processing map...")
            return## exit mapcallback early to go to next iteration
        self.get_logger().info(f"Current robot pose: {self.robot_position}")
        map_data=np.reshape(np.array(msg.data),(msg.info.height,msg.info.width))# update array to 2d array with current height and width 
        self.get_logger().info(f"This is the map dimesnsions {map_data.shape}")
        self.get_logger().info("This is the first 3 rows of the map data")
        self.get_logger().info(f"{map_data[:3][:10]}")
        #Need to find frontiers to explore
        # get robot position from the map message
        distance=-1 # used to find shortest distance or closest frontier start at neg one to ensure first value is taken
        found_frontier=False
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                if map_data[i][j]==-1:# unknown/frontier
                    physical_location=self.physical_location(i,j,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))
                    dist=mt.sqrt((physical_location[0]-self.robot_position[0])**2+(physical_location[1]-self.robot_position[1])**2)
                    if dist > 0.2 and (distance == -1 or distance > dist): # take first value 
                        distance=dist# instead of using robot position using slam map use phyical position instead cause that will not change
                        x=physical_location[0]# store the x and y of the frontier
                        y=physical_location[1]
                        found_frontier=True

        if found_frontier:
            self.get_logger().info(f"Sending ({x:.2f}, {y:.2f})")
            self.send_nav_goal(x,y)
        else:
            self.get_logger().info("No frontiers found — stopping map processing.")
            raise SystemExit
        pass

    def physical_location(self,i,j,map_resolution,map_origin):
        x=j*map_resolution+map_origin[0]
        y=i*map_resolution+map_origin[1]
        return (x,y)

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
        self.nav_client.send_goal_async(goal)# send the goal

def main():
    rclpy.init()
    node = MyMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
## initialize rclpy which sets up all the ros communication stuff
# create map node object
# keep listening for new values
# once finish clean things up 