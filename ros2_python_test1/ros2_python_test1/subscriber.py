import rclpy  # Import the ROS 2 Python library
from rclpy.node import Node  # Import the Node class to create a ROS 2 node

from std_msgs.msg import String  # Import the standard ROS 2 message type for strings


# Define a class for a minimal subscriber node
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')  # Initialize the node with the name 'minimal_subscriber'
        self.subscription = self.create_subscription(  # Create a subscription to a topic
            String,  # The message type to subscribe to (String)
            'topic',  # The name of the topic to subscribe to
            self.listener_callback,  # The callback function to call when a message is received
            10)  # The queue size (limits how many messages are stored for processing)
        self.subscription  # Prevent unused variable warning (though not strictly necessary)

    # Define the callback function that will be triggered upon receiving a message
    def listener_callback(self, msg):
        self.get_logger().info('I HEARD: "%s"' % msg.data)  # Log the received message's data


# Define the main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python library

    minimal_subscriber = MinimalSubscriber()  # Create an instance of the MinimalSubscriber node

    rclpy.spin(minimal_subscriber)  # Keep the node running and listening for incoming messages

    # Destroy the node explicitly (optional)
    # This ensures proper cleanup of the node
    minimal_subscriber.destroy_node()
    rclpy.shutdown()  # Shut down the ROS 2 Python library


# Run the main function if this script is executed directly
if __name__ == '__main__':
    main()
