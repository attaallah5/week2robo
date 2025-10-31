import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type to subscribe to command messages

class CommandSubscriber(Node):
    def __init__(self):
        # Initialize the ROS 2 node with the name 'command_subscriber'
        super().__init__('command_subscriber')

        # Create a subscription to the /robot/command topic with the message type String
        # The '10' is the queue size, meaning the node can buffer up to 10 messages
        self.subscription = self.create_subscription(
            String,  # The message type we are subscribing to
            '/robot/command',  # The topic to listen to
            self.listener_callback,  # The callback function to process messages
            10  # QoS (Quality of Service) depth, defines how many messages to buffer
        )
        # The subscription object is stored to prevent unused variable warning

    def listener_callback(self, msg):
        # This method is called every time a message is received on the /robot/command topic
        # The 'msg' parameter contains the message (of type String)
        
        # Log the received command for debugging purposes
        self.get_logger().info(f'Received command: "{msg.data}"')

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the CommandSubscriber node
    node = CommandSubscriber()

    # Spin the node to keep it active and continuously listen for messages
    rclpy.spin(node)

    # Clean up and shut down the node when done
    node.destroy_node()
    rclpy.shutdown()

# Ensure the script runs when executed directly
if __name__ == '__main__':
    main()
