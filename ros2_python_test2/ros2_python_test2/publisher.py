import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from glob import glob
class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Declare parameters for folder path and image publishing rate
        self.declare_parameter('image_folder', '/home/abhi/Pictures')
        self.declare_parameter('publish_rate', 1.0)

        # Get the parameters
        self.image_folder = self.get_parameter('image_folder').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Create a publisher for Image messages
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)

        # Initialize CV bridge for OpenCV to ROS image conversion
        self.bridge = CvBridge()

        # Get the list of images in the folder
        self.image_files = [f for f in os.listdir(self.image_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
        self.image_files.sort()

        if not self.image_files:
            self.get_logger().error(f"No images found in folder: {self.image_folder}")
            return

        self.current_image_index = 0

        # Create a timer to publish images at the specified rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_image)

    def publish_image(self):
        # Get the current image path
        image_path = os.path.join(self.image_folder, self.image_files[self.current_image_index])
        
        # Read the image using OpenCV
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.get_logger().error(f"Failed to read image: {image_path}")
            return

        # Convert the OpenCV image to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Publish the image
        self.publisher_.publish(ros_image)
        self.get_logger().info(f"Published image: {image_path}")

        # Update the index to loop through the images
        self.current_image_index = (self.current_image_index + 1) % len(self.image_files)

def main(args=None):
    rclpy.init(args=args)

    image_publisher_node = ImagePublisherNode()

    try:
        rclpy.spin(image_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


