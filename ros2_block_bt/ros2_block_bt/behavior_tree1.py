import rclpy
from rclpy.node import Node
import py_trees
import time
import os
import random


# Custom Behavior: Pickup Item
class PickUpItem(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(PickUpItem, self).__init__(name)
        self.item = name

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info(f"Setup: Preparing to pick up {self.item}")

    def update(self):
        """Simulate a 30% chance of failure when picking up an item."""
        if random.random() < 0.01:  # 30% failure chance
            self.logger.warning(f"Failed to pick up {self.item}!")
            return py_trees.common.Status.FAILURE
        else:
            self.logger.info(f"Successfully picked up {self.item}!")
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """Runs after execution completes (success or failure)."""
        self.logger.info(f"Terminate: Cleaning up after picking up {self.item}")


# Custom Behavior: Place Items in Basket
class PlaceInBasket(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super(PlaceInBasket, self).__init__("Place Items in Basket")
        self.node = node  # Reference to ROS 2 node

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup: Preparing to place items in the laundry basket")

    def update(self):
        """Simulate placing items in the basket."""
        self.logger.info("Placing all items in the laundry basket...")
        time.sleep(1)  # Simulate action duration
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate: Finished placing items in the laundry basket")
        self.logger.info("All items are placed. Destroying node...")
        self.node.get_logger().info("Stopping behavior tree...")
        self.node.tree.interrupt()  # Interrupt the tree execution

        # Destroy the ROS 2 node, shutdown is handled in `main()`
        self.node.get_logger().info("Destroying ROS 2 node...")
        raise SystemExit


# ROS 2 Node for running the Behavior Tree
class LaundryRobotBT(Node):
    def __init__(self):
        super().__init__('laundry_robot_bt')

        # Root of the behavior tree
        root = py_trees.composites.Sequence("Root Sequence", memory=True)

        pickup_socks = PickUpItem("Shock")  # Normal execution (no retries)
        pickup_shirt = PickUpItem("Shirt")  # Normal execution (no retries)
        pickup_pants = PickUpItem("Pants")  # Normal execution (no retries)

        # Define Placement Action
        place_in_basket = PlaceInBasket(self)  # Pass ROS 2 node reference

        # Build the Behavior Tree
        root.add_children([pickup_socks, pickup_shirt, pickup_pants, place_in_basket])

        # Save behavior tree diagram
        self.save_tree_diagram(root)

        # Initialize and execute the tree
        self.tree = py_trees.trees.BehaviourTree(root)
        self.run_behavior_tree()


    def run_behavior_tree(self):
        self.get_logger().info("Starting Laundry Robot Behavior Tree...")
        while rclpy.ok():
            self.tree.tick()
            time.sleep(1)  # Tick rate for the tree

    def save_tree_diagram(self, root):
        """Generate and save the behavior tree diagram."""
        dot_file = "~/bt1.dot"
        py_trees.display.render_dot_tree(root, name=dot_file.replace(".dot", ""))
        self.get_logger().info(f"Behavior tree saved as {dot_file}")

        # Convert DOT file to PNG image
        os.system(f"dot -Tpng {dot_file} -o {dot_file.replace('.dot', '.png')}")
        self.get_logger().info(f"Behavior tree diagram saved as {dot_file.replace('.dot', '.png')}")


def main(args=None):
    rclpy.init(args=args)
    node = LaundryRobotBT()

    try:
        rclpy.spin(node)
    except SystemExit:  # <--- process the exception
        node.get_logger().info('Quitting ... Done')
    # rclpy.spin(node)
    # node.get_logger().info("Interrupted by user, shutting down...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
