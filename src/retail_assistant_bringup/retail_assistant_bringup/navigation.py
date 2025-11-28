import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose             
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from rclpy.timer import Timer
from action_msgs.msg import GoalStatus
import math as mt
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
        self.unstuck=False
        self.goal_handle = None
        self.distance_to_goal=0
        self.aisle_index=0
        self.hold_index=False
        self.previous_waypoint=None
        self.current_goal=None
        self.skip=False
        self.reverse=False
        self.skip_counter=0
        self.in_aisle=True
        self.back_to_start=False
        self.changing_aisle=False
        self.closest_distance_to_goal=99999
        self.stuck=False
        self.progression_counter=0
        self.progression_timer = self.create_timer(30, self.check_progression)
        #self.robot_x=0
        #self.robot_y=0
        self.cycle()# loop function

    def cycle(self):
        print(f"Goal_index= {self.goal_index} Aisle_index= {self.aisle_index} ")
        if not self.goal_in_progress:
            x=self.goals[self.aisle_index][self.goal_index][0]
            y=self.goals[self.aisle_index][self.goal_index][1]
            self.current_goal=(x,y)
            print(f"about to send {x} and {y}")
            print(f' Changing aisle: {self.changing_aisle}')
            self.last_aisle=len(self.goals[self.aisle_index])-1
            self.send_nav_goal(x,y)


    def send_nav_goal(self,x,y):
        # goal creation 
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x=x
        goal.pose.pose.position.y=y
        goal.pose.pose.orientation.w = self.orientation  
        print(f"Sending nav2 a goal {x},{y}")
        self.nav_client.wait_for_server()# wait until the action server is available
        send_future=self.nav_client.send_goal_async(goal,feedback_callback=self.aisle_skip_callback)# send the goal
        send_future.add_done_callback(self.goal_response_callback)#when this future finishes, call this function

    def aisle_skip_callback(self,msg): # add last aisle behavior 
        self.distance_to_goal=msg.feedback.distance_remaining
        self.robot_x=msg.feedback.current_pose.pose.position.x
        self.robot_y=msg.feedback.current_pose.pose.position.y
        if(self.closest_distance_to_goal>msg.feedback.distance_remaining):
            self.closest_distance_to_goal=msg.feedback.distance_remaining
        if self.skip or self.back_to_start:#  prevent
            return
        distance_from_goal=msg.feedback.distance_remaining# set timer so robot can localize where it is
        start_aisle=(self.goals[self.aisle_index][0][0],self.goals[self.aisle_index][0][1])# beginin of aisle
        finish_aisle=(self.goals[self.aisle_index][len(self.goals[self.aisle_index])-1][0],self.goals[self.aisle_index][len(self.goals[self.aisle_index])-1][1])# end of aisle 
        max_distance=mt.sqrt((finish_aisle[0]-start_aisle[0])**2+(finish_aisle[1]-start_aisle[1])**2)
            #max_distance+=distance_between_aisle
        max_distance+=max_distance*.25
        if(max_distance<distance_from_goal):
            print(f"distance to goal {distance_from_goal} and the max distance {max_distance}")
            self.previous_waypoint=(self.robot_x,self.robot_y)# change previous goal
            self.skip=True
            self.reverse=True
            self.goal_in_progress=False
            cancel_future=self.nav_client._cancel_goal_async(self.goal_handle)# cancel goal

                
        
            
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

    def goal_result_callback(self,future):# logic for indexing
        result=future.result()# waits for nav2 to complete goal
        if((self.goal_index==0 and self.orientation>0) or (self.goal_index==self.last_aisle and self.orientation<0)):
                self.changing_aisle=True
        else:
            self.changing_aisle=False
        self.back_to_start=False
        if self.previous_waypoint is not None:
            print(f"Current waypoint is {self.current_goal} Previous waypoint is {self.previous_waypoint}")
        if self.paused:# goal_result_callback runs anytime a goal is updated including cancellations prevents spam of goals when prox detect
            self.get_logger().warn("Goal finished but robot is paused — waiting before continuing.")
            return
        if self.reverse:# skip aisle indexing behavior
            self.get_logger().warn("Ailse blocked reversing to next aisle")
            self.planner_timer = self.create_timer(10, self.allow_planner_update)# timer to wait till check aisles again 
            self.orientation*=-1.0
            self.hold_index=False
            self.reverse=False
            if self.changing_aisle:
                self.aisle_index+=1
                self.changing_aisle=True
                self.orientation*=-1
                print("The entrance of the aisle is blocked headed to the next aisle instead")
                if self.aisle_index>len(self.goals)-1:
                    self.aisle_index=0
                    self.back_to_start=True
            elif self.orientation>0:
                self.goal_index+=1
                self.changing_aisle=False
                if self.goal_index==len(self.goals[self.aisle_index])-1:
                    self.hold_index=True
            else:
                self.goal_index-=1
                self.changing_aisle=False
                if self.goal_index==0:
                    self.hold_index=True
            self.cycle()
            return 
        
        self.get_logger().info(f"Goal completed with result: {result.status}")
        self.previous_waypoint=self.goals[self.aisle_index][self.goal_index]# save waypoint before changing
        self.goal_in_progress=False
        if self.hold_index:# repeat goal_index when reach ends
            self.hold_index=False
            self.aisle_index+=1
            if(self.aisle_index==len(self.goals)):
                self.aisle_index=0
                self.back_to_start=True
            self.orientation*=-1.0# change orientation when changing aisles
        else:# if not changing aisles update goal index
            if self.orientation>0:
                self.goal_index+=1
                if self.goal_index==len(self.goals[self.aisle_index])-1: #and not self.reverse:#reaches end of aisle
                    self.hold_index=True
                        #self.reverse=False
            else:# odd aisle go backward
                self.goal_index-=1
                if self.goal_index==0:#and not self.reverse:# at beginning of aisle
                    self.hold_index=True
                        #self.reverse=False
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

    def allow_planner_update(self):
        self.planner_timer.cancel()
        self.get_logger().info("Allowing robot to move and the planner to update before checking Aisle blockage")
        self.skip=False
        #self.reverse=True
    def check_progression(self):
        if self.distance_to_goal>=self.closest_distance_to_goal:
            self.progression_counter+=1
        if self.progression_counter>2:
            self.progression_counter=0
            self.get_logger().warn("I am stuck and in need of assistance")
            self.goal_in_progress=False
            self.paused=True
            cancel_future=self.nav_client._cancel_goal_async(self.goal_handle)
            self.resume_timer = self.create_timer(15.0, self.resume_navigation)
            self.unstuck=True

    def resume_navigation(self):
        self.resume_timer.cancel()# cancel timer 
        if self.interaction:
            self.get_logger().info("Waiting for customer to finish interaction")
        else:
            if self.unstuck:
                self.get_logger().info("Was given assistance resuming navigation")
                self.unstuck=False
            else:
                self.get_logger().info("No interaction continue navigation")
                self.prox_wait_timer=self.create_timer(10.0,self.enable_proximity)
            #self.send_nav_goal(self.goals[self.aisle_index][self.goal_index][0],self.goals[self.aisle_index][self.goal_index][1]) # consider just doing the self.cycle() function
            self.cycle()
           # Allow robot to move for a little before resuming proximity detection
            
    def assisted(self):
        self.assistance_timer.cancel()
        self.get_logger().info("I was assisted")
        self.skip_counter=0
        self.skip=False
        self.cycle()

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
        
        