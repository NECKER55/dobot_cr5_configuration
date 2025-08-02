import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.time import Time

class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # Subscriber su /joint_states_robot
        self.subscriber = self.create_subscription(
            JointState,
            '/joint_states_robot',
            self.joint_state_callback,
            10
        )
        
        # Publisher su /joint_states
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

    def joint_state_callback(self, msg):
        # Aggiorna il timestamp prima di pubblicare
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
