import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Pose

class ActorCollisionFollower(Node):
    def __init__(self):
        super().__init__('actor_collision_follower')

        self.actor_name = 'actor1'
        self.proxy_name = 'actor1_collision_proxy'

        self.sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.update_proxy, 10)

        self.pub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        self.get_logger().info('Actor collision follower started.')

    def update_proxy(self, msg: ModelStates):
        if self.actor_name not in msg.name or self.proxy_name not in msg.name:
            return

        actor_idx = msg.name.index(self.actor_name)
        proxy_idx = msg.name.index(self.proxy_name)

        actor_pose = msg.pose[actor_idx]
        proxy_pose = Pose()
        proxy_pose.position.x = actor_pose.position.x
        proxy_pose.position.y = actor_pose.position.y
        proxy_pose.position.z = actor_pose.position.z - 0.6  # adjust height
        proxy_pose.orientation = actor_pose.orientation

        state = ModelState()
        state.model_name = self.proxy_name
        state.pose = proxy_pose
        state.reference_frame = 'world'

        self.pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    node = ActorCollisionFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
