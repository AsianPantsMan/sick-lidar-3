import rclpy
import math as mt
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose             
from geometry_msgs.msg import PoseStamped
from rclpy.timer import Timer
from nav2_msgs.srv import SaveMap
import firebase_admin
from firebase_admin import credentials, storage as fb_storage, firestore
import os
import datetime
#node + subsscriber initalization
class MyMapNode(Node):
    def __init__(self):
        super().__init__('map_subscriber_node')
        # create_subscription(msg_type, topic, callback, queue_size)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)#Whenever /map type occupancy gird updated call function callback
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')# client for nav2 locations?
        self.goal_in_progress = False
        self.goal_handle = None
            # prevent cycling through same points
        self.visited_frontiers= []
        self.points_to_score=[]
        self.points_scored=0
        self.skip_point=False
    #function subscribed and using pose topic made by slam toolbox to get robot position

    def map_callback(self, msg):
        if self.goal_in_progress:### prevent spamming goals
            return
        
        high_score= -9999 
        map_data=np.reshape(np.array(msg.data),(msg.info.height,msg.info.width))# update array to 2d array with current height and width 
        self.get_logger().info(f"This is the map dimesnsions {map_data.shape}")
        loop_counter=int((map_data.shape[0]*map_data.shape[1])*.05)
        for i in range(msg.info.height):
            if loop_counter<=0:
                break
            for j in range(msg.info.width):
                if loop_counter<=0:
                    break
                if map_data[i][j]==0:# free space can travel to unknown}
                    neighborhood = map_data[max(0,i-1):min(i+2,msg.info.height), max(0,j-1):min(j+2,msg.info.width)]#check this
                    if (neighborhood == -1).any():# free space has frontier near it
                        physical_location=self.physical_location(i,j,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))# compute physical map cell coordinate
                        if (physical_location[0],physical_location[1]) not in self.visited_frontiers:# distance is sufficient far from origin and shortest and the hasent been to this frontie
                            if(loop_counter>0 ):#takes 5 % of map point
                                loop_counter-=1
                                self.points_scored+=1 # maybe opt for counter for increased efficiency
                                result=self.scoring(i,j,map_data,msg)
                                if result[2]>high_score:
                                    x=result[0]
                                    y=result[1]
                                    high_score=result[2] 
                                continue
                                            
        if self.points_scored>0:# if nav2 fails some reason will send new coordinate
            physical_location=self.physical_location(x,y,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))# convert highest scoring point to real coordinates
            x=physical_location[0]# store the x and y of the frontier
            y=physical_location[1]
            self.visited_frontiers.append(physical_location)
            self.get_logger().info(f'Selected frontier at map cell ({i}, {j}) with score {high_score} compared agasint {self.points_scored} points to score')
            self.points_scored=0
            self.goal_in_progress=True
            self.get_logger().info(f"Sending ({x:.2f}, {y:.2f})")
            self.send_nav_goal(x,y)
        else:
            self.get_logger().info("No frontiers found — stopping map processing.")
            self.save_map()
            raise SystemExit
    def save_map(self):
        self.get_logger().info("Saving map to Slam_maps folder...")
        client = self.create_client(SaveMap, '/map_saver/save_map')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /map_saver/save_map service...')
        req = SaveMap.Request()
        req.map_topic = '/map'
        req.map_url = '/home/retail-assistant/SLAM/src/retail_assistant_bringup/Slam_maps/auto_map' # change
        req.image_format = 'pgm'
        req.map_mode = 'trinary'
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Map saved successfully.")
            self.send_to_firebase(req.map_url + '.yaml')  # Upload the YAML file to Firebase
        else:
            self.get_logger().error("Failed to save map.")
    def physical_location(self,i,j,map_resolution,map_origin):
        x=j*map_resolution+map_origin[0]
        y=i*map_resolution+map_origin[1]
        return (x,y)
    

    def scoring(self,i,j,map_data,msg): # Scoring function to select best frontier or a good enough frontier to limit back point selection
        score=0
        free=0
        occupied=0
        frontier=0
        distance_to_wall=10
        base_radius = int(0.5 / msg.info.resolution)  # about 0.5m window
        if self.visited_frontiers:# sim world lenght between aisles and height =20m
            previous_location=self.visited_frontiers[-1]
        else:
            previous_location=(0,0)
        physical_location=self.physical_location(i,j,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))# compute physical map cell coordinate
        distance=mt.sqrt((physical_location[0]-previous_location[0])**2+(physical_location[1]-previous_location[1])**2) 
        neighborhood = map_data[max(0,i-base_radius):min(i+(base_radius+1),msg.info.height), max(0,j-base_radius):min(j+(1+base_radius),msg.info.width)]# neighborhood 11x11
        for x in range(neighborhood.shape[0]):
            for y in range(neighborhood.shape[1]):
                if neighborhood[x][y]==0:
                    free+=1
                if neighborhood[x][y]==-1:# gathering context of the point to score it
                    frontier+=1
                if neighborhood[x][y]==100: # m
                    occupied_physical_location=self.physical_location(i-base_radius+x,j-base_radius+y,msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))
                    d=mt.sqrt((physical_location[0]-occupied_physical_location[0])**2+(physical_location[1]-occupied_physical_location[1])**2)
                    distance_to_wall=min(d,distance_to_wall)
                    occupied+=1
        #scoring logic
        free_ratio=free/(neighborhood.shape[0]*neighborhood.shape[1])
        frontier_ratio=frontier/(neighborhood.shape[0]*neighborhood.shape[1])
        occupied_ratio=occupied/(neighborhood.shape[0]*neighborhood.shape[1])
        score-=occupied_ratio*10 # penalize occupied space
        if distance_to_wall<0.45:#frontier inside wall
            self.points_scored-=1
            return 0,0,-99999#inside wall 
        if free_ratio>0.75 or free_ratio<0.3:# information gain to low
            score-=10
        else:
            score+=frontier_ratio*10 # encourage information gain
        if distance<1.0 or distance>8.0: # too close or too far from last point
            score-=40 
        if frontier_ratio<0.25:
            score-=10 # not enough information gain
        return i,j,score

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
        send_future=self.nav_client.send_goal_async(goal)#feedback_callback=self.recovery_skip_callback)# send the goal
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        self.goal_handle =future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("goal rejected by nav2")
            self.goal_in_progress=False
            return
        self.get_logger().info("Goal accepted navigating")
        result_future=self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self,future):
        result=future.result().result
        self.get_logger().info(f"Goal completed with result: {result}")
        self.wait_timer=self.create_timer(3.0,self.wait_then_send_goal)#non blocking 
         # so next frontier can begin
    def recovery_skip_callback(self,msg):
        if self.skip_point:
            return
        recoveries=msg.feedback.number_of_recoveries
        if recoveries>3:
            self.get_logger().warn(f"Skipping goal due to too many recoveries")
            self.goal_in_progress=False
            self.skip_point=True
            cancel_future=self.nav_client._cancel_goal_async(self.goal_handle)

    def wait_then_send_goal(self):
        self.wait_timer.cancel()
        self.goal_in_progress=False
        self.skip_point=False

    def send_to_firebase(self,file_path):
        if not _FIREBASE_OK:
            self.get_logger().warn("☁️ Firebase not initialised — skipping map upload.")
        return

    try:
        now = datetime.datetime.utcnow()
        ts_str = now.strftime("%Y%m%d_%H%M%S_") + f"{now.microsecond:06d}"

        bucket = fb_storage.bucket()
        db = firestore.client()

        yaml_name = os.path.basename(yaml_path)
        base_name = os.path.splitext(yaml_name)[0]
        pgm_path = os.path.join(os.path.dirname(yaml_path), base_name + ".pgm")

        folder = f"slam_maps/{ts_str}_{base_name}"

        yaml_url = None
        pgm_url = None

        # Upload YAML
        if os.path.exists(yaml_path):
            yaml_blob = bucket.blob(f"{folder}/{yaml_name}")
            yaml_blob.upload_from_filename(yaml_path, content_type="text/yaml")
            yaml_blob.make_public()
            yaml_url = yaml_blob.public_url
        else:
            self.get_logger().warn(f"YAML file not found: {yaml_path}")

        # Upload PGM if it exists
        if os.path.exists(pgm_path):
            pgm_blob = bucket.blob(f"{folder}/{base_name}.pgm")
            pgm_blob.upload_from_filename(pgm_path, content_type="image/x-portable-graymap")
            pgm_blob.make_public()
            pgm_url = pgm_blob.public_url
        else:
            self.get_logger().warn(f"PGM file not found: {pgm_path}")

        # Save metadata in Firestore
        db.collection("slam_maps").add({
            "timestamp": now.isoformat() + "Z",
            "map_name": base_name,
            "yaml_url": yaml_url,
            "pgm_url": pgm_url,
            "folder": folder,
        })

        self.get_logger().info(f"☁️ SLAM map uploaded successfully → {folder}")

    except Exception as e:
        self.get_logger().error(f"SLAM map upload failed: {e}")
        

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