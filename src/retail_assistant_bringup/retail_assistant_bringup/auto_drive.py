import rclpy
import math as mt
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose             #ADD CLAMP TO PREVENT BAD COORDINATES OUTSIDE OF THE MAP
from geometry_msgs.msg import PoseStamped
#node + subsscriber initalization
class MyMapNode(Node):
    def __init__(self):
        super().__init__('map_subscriber_node')
        # create_subscription(msg_type, topic, callback, queue_size)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)#Whenever /map type occupancy gird updated call function callback
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')# client for nav2 locations?
        self.goal_in_progress = False
            # prevent cycling through same points
        self.visited_frontiers = set()
        self.points_to_score=set()
        self.high_socre=0
    #function subscribed and using pose topic made by slam toolbox to get robot position

    def map_callback(self, msg):
        #if self.robot_position == [0.0, 0.0]:
           # self.get_logger().warn("Waiting for valid robot pose before processing map...")
           # return## exit mapcallback early to go to next iteration
        if self.goal_in_progress:### prevent spamming goals
            return
        
        map_data=np.reshape(np.array(msg.data),(msg.info.height,msg.info.width))# update array to 2d array with current height and width 
        self.get_logger().info(f"This is the map dimesnsions {map_data.shape}")
        origin_x=msg.info.origin.position.x
        origin_y=msg.info.origin.position.y
        if self.visited_frontiers:
            last_x, last_y = list(self.visited_frontiers)[-1]
        else:  # no last goal yet
            last_x, last_y = (None, None)#get last visited for threshold distance
        found_frontier=False
        for i in range(msg.info.height):
            for j in range(msg.info.width):
                if map_data[i][j]==0:# free space can travel to unknown}
                    neighborhood = map_data[max(0,i-1):min(i+2,msg.info.height), max(0,j-1):min(j+2,msg.info.width)]#check this
                    if -1 in neighborhood:# free space has frontier near it
                        physical_location=self.physical_location(i,j,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))# compute physical map cell coordinate
                        if (physical_location[0],physical_location[1]) not in self.visited_frontiers:# distance is sufficient far from origin and shortest and the hasent been to this frontie
                           if(len(self.points_to_score)<min(max(int(len(map_data) * .05), 100), 2000)):#takes 5 % of map points up to 2000 points to score
                            result=self.points_to_score.add(i,j,msg)
                            if result[2]>self.high_socre:
                                x=result[0]
                                y=result[1]
                                self.high_socre=result[2] 
                            continue
                          
                        physical_location=self.physical_location(x,y,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))# convert highest scoring point to real coordinates
                        x=physical_location[0]# store the x and y of the frontier
                        y=physical_location[1]
                        self.visited_frontiers.add(physical_location)
                        found_frontier=True
            if found_frontier:##################### CHECK out of bounds conditions ALSO add scoring mechanism for better point selectionm
                break


        if found_frontier:# if nav2 fails some reason will send new coordinate
            self.goal_in_progress=True
            self.get_logger().info(f"Sending ({x:.2f}, {y:.2f})")
            self.edge_prevention(x,y,msg)
            self.send_nav_goal(x,y)
        else:
            self.get_logger().info("No frontiers found — stopping map processing.")
            # ADD MAP SAVING TO SPECIFIED DIRECTORY
            raise SystemExit

    def physical_location(self,i,j,map_resolution,map_origin):
        x=j*map_resolution+map_origin[0]
        y=i*map_resolution+map_origin[1]
        return (x,y)
    

    def scoring(self,i,j,msg): # Scoring function to select best frontier or a good enough frontier to limit back point selection
        score=0
        # if last_x is not none this is if it has anything in previous frontiers
        map_data=np.reshape(np.array(msg.data),(msg.info.height,msg.info.width))
        neighborhood = map_data[max(0,i-5):min(i+6,msg.info.height), max(0,j-5):min(j+6,msg.info.width)]# neighborhood 11x11
        for neighborhood_x in range(neighborhood.shape[0]):
            for neighborhood_y in range(neighborhood.shape[1]):
                if neighborhood[x][y]==0:# scoring how reachable/connected to free space
                    score+=1
                if neighborhood[x][y]==-1:# scoring on information gain
                    score+=2
                if self.visited_frontiers: #penalty for too close to previous fronti
                    print("placeholder")
                elif self.visited_frontiers:
                    print("placeholder")

            # add score from how close it is to wall blocked objects
            # include a timer/delay to allow slam to update occupancy grid
        return x,y,score

        #Score based on 
            

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
        result=future.result().result
        self.get_logger().info(f"Goal completed with result: {result}")
        self.goal_in_progress=False # so next frontier can begin
    
def main():
    rclpy.init()## initialize rclpy which sets up all the ros communication stuff
    node = MyMapNode()# create map node object
    try:
        rclpy.spin(node)# keep listening for new values
    except KeyboardInterrupt:
        pass
    node.destroy_node()# once finish clean things up 
    rclpy.shutdown()

if __name__ == "__main__":
    main()
