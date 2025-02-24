import rclpy
from rclpy.node import Node
import py_trees


# Condition Nodes
class AtGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name="AtGoal"):
        super(AtGoal, self).__init__(name)

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup")

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate")

    def update(self):
        """Check if the robot is already at the goal."""
        self.logger.info("[AtGoal] Checking if at goal...")
        return py_trees.common.Status.FAILURE  # Simulated failure


class HoldsObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="HoldsObject"):
        super(HoldsObject, self).__init__(name)

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup")

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate")

    def update(self):
        """Check if the robot is already holding the object."""
        self.logger.info("[HoldsObject] Checking if holding the object...")
        return py_trees.common.Status.FAILURE  # Simulated failure


class AtObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="AtObject"):
        super(AtObject, self).__init__(name)

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup")

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate")

    def update(self):
        """Check if the robot is at the object location."""
        self.logger.info("[AtObject] Checking if at the object...")
        return py_trees.common.Status.FAILURE  # Simulated failure


class DetectedObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="DetectedObject"):
        super(DetectedObject, self).__init__(name)

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup")

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate")

    def update(self):
        """Check if an object is detected."""
        self.logger.info("[DetectedObject] Checking if object detected...")
        return py_trees.common.Status.SUCCESS  # Simulated success


# Action Nodes
class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveToGoal"):
        super(MoveToGoal, self).__init__(name)

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup")

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate")

    def update(self):
        """Move the robot to the goal."""
        self.logger.info("[MoveToGoal] Moving to goal...")
        return py_trees.common.Status.SUCCESS  # Simulated success


class MoveToObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="MoveToObject"):
        super(MoveToObject, self).__init__(name)

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup")

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate")

    def update(self):
        """Move the robot to the detected object."""
        self.logger.info("[MoveToObject] Moving to object...")
        return py_trees.common.Status.SUCCESS  # Simulated success


class PickObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="PickObject"):
        super(PickObject, self).__init__(name)

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup")

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate")

    def update(self):
        """Pick up the object."""
        self.logger.info("[PickObject] Picking up object...")
        return py_trees.common.Status.SUCCESS  # Simulated success


class PlaceObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="PlaceObject"):
        super(PlaceObject, self).__init__(name)

    def setup(self, **kwargs):
        """Runs once before execution starts."""
        self.logger.info("Setup")

    def terminate(self, new_status):
        """Runs after execution completes."""
        self.logger.info("Terminate")

    def update(self):
        """Place the object at the goal location."""
        self.logger.info("[PlaceObject] Placing object at goal...")
        return py_trees.common.Status.SUCCESS  # Simulated success


# ROS2 Node for Behavior Tree Execution
class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__("behavior_tree_node")
        self.get_logger().info("Initializing Behavior Tree Node...")

        # Create the behavior tree
        self.tree = self.create_behavior_tree()

        # Create a ROS2 Timer to tick the behavior tree
        self.timer = self.create_timer(1.0, self.tick_tree)  # Tick every 1 second

    def create_behavior_tree(self):
        """Constructs the behavior tree exactly as per the provided image."""
        place_object = PlaceObject("PlaceObject")

        # Move to Goal Fallback
        at_goal = AtGoal("AtGoal")
        move_to_goal = MoveToGoal("MoveToGoal")


        # Holds Object Fallback
        holds_object = HoldsObject("HoldsObject")

        # Move to Object Fallback
        at_object = AtObject("AtObject")
        detected_object = DetectedObject("DetectedObject")
        move_to_object = MoveToObject("MoveToObject")

        # Pick Object Sequence
        pick_object = PickObject("PickObject")

        move_to_goal_fallback = py_trees.composites.Selector("MoveToGoal?", children=[at_goal, move_to_goal],
                                                             memory=True)
        move_to_object_seq = py_trees.composites.Sequence("MoveToObjectSeq", children=[detected_object, move_to_object],
                                                          memory=True)
        move_to_object_fallback = py_trees.composites.Selector("MoveToObject?",
                                                               children=[at_object, move_to_object_seq], memory=True)
        pick_object_seq = py_trees.composites.Sequence("PickObjectSeq", children=[move_to_object_fallback, pick_object],
                                                       memory=True)
        pick_object_fallback = py_trees.composites.Selector("PickObject?", children=[holds_object, pick_object_seq],
                                                            memory=True)

        # Main Behavior Tree
        root = py_trees.composites.Sequence("MainBT",
                                            children=[move_to_goal_fallback, pick_object_fallback, place_object],
                                            memory=True)

        return py_trees.trees.BehaviourTree(root)

    def tick_tree(self):
        """Tick the behavior tree at a set interval."""
        self.get_logger().info("Ticking Behavior Tree...")
        self.tree.tick()


def main(args=None):
    """Initialize the ROS2 behavior tree node."""
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
