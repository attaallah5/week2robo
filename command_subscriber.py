import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')
        self.subscription = self.create_subscription(
            String, '/robot/command', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = CommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
