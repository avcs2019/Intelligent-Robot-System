import rclpy  # Import the ROS 2 Python library
from rclpy.node import Node  # Import the Node class to create a ROS 2 node
from std_msgs.msg import String  # Import the standard ROS 2 message type for strings


# Define a class for a minimal publisher node
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')  # Initialize the node with the name 'minimal_publisher'
        self.publisher_ = self.create_publisher(String, 'topic',
                                                10)  # Create a publisher for the 'topic' topic with a queue size of 10
        timer_period = 0.5  # Set the timer period to 0.5 seconds
        self.timer = self.create_timer(timer_period,
                                       self.timer_callback)  # Create a timer to call `timer_callback` every 0.5 seconds
        self.i = 0  # Initialize a counter variable to keep track of the number of messages published

    # Define the callback function to be called by the timer
    def timer_callback(self):
        msg = String()  # Create a new String message
        msg.data = 'Publishing....: %d' % self.i  # Set the message data with the counter value
        self.publisher_.publish(msg)  # Publish the message to the topic
        self.get_logger().info('Publishing: "%s"' % msg.data)  # Log the published message
        self.i += 1  # Increment the counter


# Define the main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python library

    minimal_publisher = MinimalPublisher()  # Create an instance of the MinimalPublisher node

    rclpy.spin(minimal_publisher)  # Keep the node running and listening for events

    # Destroy the node explicitly (optional)
    # This ensures proper cleanup of the node
    minimal_publisher.destroy_node()
    rclpy.shutdown()  # Shut down the ROS 2 Python library


# Run the main function if this script is executed directly
if __name__ == '__main__':
    main()
