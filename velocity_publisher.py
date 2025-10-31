import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Import the Twist message type for publishing velocity commands

class VelocityPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'velocity_publisher'
        super().__init__('velocity_publisher')

        # Create a publisher for the /cmd_vel topic with the message type Twist
        # The '10' is the queue size for storing messages
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a timer that calls the publish_velocity method every 1 second
        # This will repeatedly publish velocity commands to the /cmd_vel topic
        self.timer = self.create_timer(1.0, self.publish_velocity)

    def publish_velocity(self):
        # Create a new Twist message to represent velocity commands
        msg = Twist()

        # Set the linear velocity in the x direction (forward movement)
        # For example, this will move the robot at 0.5 meters per second
        msg.linear.x = 0.5

        # Set the angular velocity around the z-axis (turning movement)
        # For example, this will rotate the robot at 0.2 radians per second
        msg.angular.z = 0.2

        # Publish the velocity message to the /cmd_vel topic
        self.publisher_.publish(msg)

        # Log the published values (useful for debugging and verification)
        self.get_logger().info(f'Publishing: Linear X: {msg.linear.x}, Angular Z: {msg.angular.z}')

def main(args=None):
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the VelocityPublisher node
    node = VelocityPublisher()

    # Keep the node spinning to continuously publish messages
    rclpy.spin(node)

    # Clean up and shut down the node when done
    node.destroy_node()
    rclpy.shutdown()

# Ensure the script runs when executed directly
if __name__ == '__main__':
    main()

