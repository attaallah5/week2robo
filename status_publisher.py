import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type for publishing status information

class StatusPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'status_publisher'
        super().__init__('status_publisher')

        # Create a publisher for the /robot/status topic with the message type String
        # The '10' is the queue size for storing messages
        self.publisher_ = self.create_publisher(String, '/robot/status', 10)

        # Create a timer that calls the publish_status method every 2 seconds
        # This will repeatedly publish a status message to the /robot/status topic
        self.timer = self.create_timer(2.0, self.publish_status)

    def publish_status(self):
        # Create a new String message to represent the robot's status
        msg = String()

        # Set the message data to "Robot is running"
        # This represents the robot's current status
        msg.data = "Robot is running"

        # Publish the status message to the /robot/status topic
        self.publisher_.publish(msg)

        # Log the published status message for debugging and verification
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the StatusPublisher node
    node = StatusPublisher()

    # Keep the node spinning to continuously publish status messages
    rclpy.spin(node)

    # Clean up and shut down the node when done
    node.destroy_node()
    rclpy.shutdown()

# Ensure the script runs when executed directly
if __name__ == '__main__':
    main()
