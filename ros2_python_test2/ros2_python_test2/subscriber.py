import rclpy  # Import the ROS 2 Python library
from rclpy.node import Node  # Import the Node class to create a ROS 2 node
from sensor_msgs.msg import Image  # Import the Image message type


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')  # Initialize the node with the name 'image_subscriber'
        # Create a subscription to the '/image' topic
        self.subscription = self.create_subscription(
            Image,  # Message type
            '/image_topic',  # Topic name
            self.listener_callback,  # Callback function to process the received message
            10)  # Queue size
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received an image with height: %d and width: %d' % (msg.height, msg.width))
        # Here you can add further processing of the image data


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library

    image_subscriber = ImageSubscriber()  # Create an instance of the ImageSubscriber node

    rclpy.spin(image_subscriber)  # Keep the node running and listening for incoming messages

    # Destroy the node explicitly (optional)
    image_subscriber.destroy_node()
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library


if __name__ == '__main__':
    main()

