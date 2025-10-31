import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.2
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Linear X: {msg.linear.x}, Angular Z: {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
