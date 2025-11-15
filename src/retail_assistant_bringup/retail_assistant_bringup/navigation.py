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
        self.goals= ([(4.94 ,0.138),(-1.83,0.13),(-10.68,-0.136)],
                    [(4.92,5.00),(-1.61,4.59),(-10.43,4.59)],
                    [(4.91,9.66),(-1.84,9.01),(-9.30,9.30)])
        self.goal_in_progress=False
        self.goal_index=0 # keeps track of where to go next
        self.orientation=1.0
        self.interaction=False
        self.paused=False
        self.goal_handle = None
        self.distance_to_goal=0
        self.aisle_index=0
        self.hold_index=False
        self.cycle()# loop function

    def cycle(self): 
        if not self.goal_in_progress:
            x=self.goals[self.aisle_index][self.goal_index][0]
            y=self.goals[self.aisle_index][self.goal_index][1]
            print(f"about to send {x} and {y}")
            self.send_nav_goal(x,y)


    def send_nav_goal(self,x, y):
        # goal creation 
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x =x
        goal.pose.pose.position.y =y
        goal.pose.pose.orientation.w = self.orientation
        if(self.hold_index):# increase aisle on the hold index becuase when aisles change
            if(self.aisle_index<=len(self.goals)-1):
                self.aisle_index+=1
        print(f"Sending nav2 a goal {x},{y}")
        self.nav_client.wait_for_server()# wait until the action server is available
        send_future=self.nav_client.send_goal_async(goal)#,feedback_callback=self.feedback)# send the goal
        send_future.add_done_callback(self.goal_response_callback)#when this future finishes, call this function

    def feedback(self,goal_handle,feedback):
        feedback.distance_remaining=self.distance_to_goal
        if(self.distance_to_goal>16):#TODO come up with better threshold distance
            #add goal cancellation here and send a goal and increment goal index
            self.get_logger().info("Cancelling nav2 destination to prevent routing through another aisle")
            cancel_future=self.nav_client._cancel_goal_async(self.goal_handle)# cancel current goal
        if(self.goal_index==0):# begining of aisle is blocked
            if(self.aisle_index==len(self.goals)-1):
                aisle_index-=1
                self.logger().warn("Beginning of aisle is blocked, moving to previous aisle")
            else:
               aisle_index+=1
               self.logger().warn("Beginning of aisle is blocked, moving to next aisle")
        elif(len(self.goals[self.aisle_index])-1):# end of aisle is blocked
            if(self.aisle_index==len(self.goals)-1):
                aisle_index-=1
                self.logger().warn("End of aisle is blocked, moving to previous aisle")
            else:
                aisle_index+=1
                self.logger().warn("End of aisle is blocked, moving to next aisle")
        else: # middle of the aisle
            if(self.aisle_index==len(self.goals)-1):
                aisle_index-=1
                self.goal_index=0
                self.logger().warn("Middle of aisle is blocked, moving to the beginning of previous aisle")
            else:
                aisle_index+=1
                self.goal_index=0
                self.logger().warn("Middle of aisle is blocked, moving to the beginning of next aisle")
        self.cycle()# start from new points effectively skipping the blocked aisle
    
            
    def goal_response_callback(self,future):
        self.goal_handle =future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("goal rejected by nav2")
            self.goal_in_progress=False
            return
        self.get_logger().info("Goal accepted navigating")
        self.goal_in_progress=True
        result_future=self.goal_handle.get_result_async()
        print("Future Result", result_future)
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self,future):
        result=future.result().result# waits for nav2 to complete goal
        if self.paused:# goal_result_callback runs anytime a goal is updated including cancellations prevents spam of goals when prox detect
            self.get_logger().warn("Goal finished but robot is paused — waiting before continuing.")
            return
        self.get_logger().info(f"Goal completed with result: {result}")
        self.goal_in_progress=False
        if self.hold_index:# repeat goal_index when reach ends
            if self.aisle_index>=len(self.goals):# reset aisle index
                self.aisle_index=0
            self.hold_index=False
            self.orientation*=-1.0# change orientation when changing aisles
        else:# if not changing aisles update goal index
            if self.orientation>0:
                self.goal_index+=1
                if self.goal_index==len(self.goals[self.aisle_index])-1:#reaches end of aisle
                    self.hold_index=True
            else:# odd aisle go backward
                self.goal_index-=1
                if self.goal_index==0:# at beginning of aisle
                    self.hold_index=True
        self.cycle()


    def proximity_callback(self,sensor): # Human interaction detection
        direction=sensor.header.frame_id
        if sensor.range < 0.5 and not self.paused:
            self.goal_in_progress=False     # consider a cooldown timer to prevent infinite triggers 
            self.paused=True
            self.get_logger().info(f"Person detected close to {direction}")
            if self.goal_handle is not None:
                self.get_logger().info("Cancelling nav2 destination and waiting for interaction")
                cancel_future=self.nav_client._cancel_goal_async(self.goal_handle)
            else:
                self.get_logger().warn("Proximity triggered but NO active Nav2 goal exists yet — skipping cancel.")
            self.get_logger().info("Waiting for customer to finish interaction")

            self.resume_timer = self.create_timer(5.0, self.resume_navigation) # wait for interaction and is non blocking

    def resume_navigation(self):
        self.resume_timer.cancel()# cancel timer 
        if self.interaction:
            self.get_logger().info("Waiting for customer to finish interaction")
        else:
            self.get_logger().info("No interaction continue navigation")
            self.send_nav_goal(self.goals[self.aisle_index][self.goal_index][0],self.goals[self.aisle_index][self.goal_index][1])
            self.prox_wait_timer=self.create_timer(10.0,self.enable_proximity)# Allow robot to move for a little before resuming proximity detection

    def enable_proximity(self):
        self.prox_wait_timer.cancel()# cancel timer this  prevents from staying stuck due to proximity sensor
        self.paused=False
        self.get_logger().info("Resuming proximity detection")

def main():
    rclpy.init()
    nav_node = AutoNav()
    rclpy.spin(nav_node)
    nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
        