import rclpy
from rclpy.node import Node
import smach
import smach_ros


# Define state and condition classes
class AtGoal(smach.State):
    def __init__(self):
        super().__init__(outcomes=["success", "failure"])

    def execute(self, userdata):
        """Check if the robot is already at the goal."""
        print("[AtGoal] Checking if at goal...")
        return "failure"  # Simulated failure


class MoveToGoal(smach.State):
    def __init__(self):
        super().__init__(outcomes=["success"])

    def execute(self, userdata):
        """Move the robot to the goal."""
        print("[MoveToGoal] Moving to goal...")
        return "success"


class HoldsObject(smach.State):
    def __init__(self):
        super().__init__(outcomes=["success", "failure"])

    def execute(self, userdata):
        """Check if the robot is already holding the object."""
        print("[HoldsObject] Checking if holding the object...")
        return "failure"  # Simulated failure


class AtObject(smach.State):
    def __init__(self):
        super().__init__(outcomes=["success", "failure"])

    def execute(self, userdata):
        """Check if the robot is at the object location."""
        print("[AtObject] Checking if at the object...")
        return "failure"  # Simulated failure


class DetectedObject(smach.State):
    def __init__(self):
        super().__init__(outcomes=["success", "failure"])

    def execute(self, userdata):
        """Check if an object is detected."""
        print("[DetectedObject] Checking if object detected...")
        return "success"  # Simulated success


class MoveToObject(smach.State):
    def __init__(self):
        super().__init__(outcomes=["success"])

    def execute(self, userdata):
        """Move the robot to the detected object."""
        print("[MoveToObject] Moving to object...")
        return "success"


class PickObject(smach.State):
    def __init__(self):
        super().__init__(outcomes=["success"])

    def execute(self, userdata):
        """Pick up the object."""
        print("[PickObject] Picking up object...")
        return "success"


class PlaceObject(smach.State):
    def __init__(self):
        super().__init__(outcomes=["success"])

    def execute(self, userdata):
        """Place the object at the goal location."""
        print("[PlaceObject] Placing object at goal...")
        return "success"


# ROS2 Node for SMACH State Machine Execution
class StateMachineNode(Node):
    def __init__(self):
        super().__init__("smach_state_machine_node")
        self.get_logger().info("Initializing SMACH State Machine Node...")

        # Create the State Machine
        sm = self.create_state_machine()

        # Execute the state machine
        outcome = sm.execute()
        self.get_logger().info(f"State Machine Finished with outcome: {outcome}")

    def create_state_machine(self):
        """Constructs the equivalent state machine for the Behavior Tree."""
        sm = smach.StateMachine(outcomes=["task_complete"])

        with sm:
            # Move to Goal Fallback (AtGoal -> MoveToGoal)
            smach.StateMachine.add("AtGoal", AtGoal(),
                                   transitions={"success": "PickObjectFlow",
                                                "failure": "MoveToGoal"})
            smach.StateMachine.add("MoveToGoal", MoveToGoal(),
                                   transitions={"success": "PickObjectFlow"})

            # Pick Object Flow (HoldsObject -> MoveToObject -> PickObject)
            pick_object_sm = smach.StateMachine(outcomes=["object_picked"])
            with pick_object_sm:
                smach.StateMachine.add("HoldsObject", HoldsObject(),
                                       transitions={"success": "object_picked",
                                                    "failure": "MoveToObjectFlow"})

                move_to_object_sm = smach.StateMachine(outcomes=["at_object"])
                with move_to_object_sm:
                    smach.StateMachine.add("AtObject", AtObject(),
                                           transitions={"success": "at_object",
                                                        "failure": "DetectedObject"})
                    smach.StateMachine.add("DetectedObject", DetectedObject(),
                                           transitions={"success": "MoveToObject",
                                                        "failure": "DetectedObject"})
                    smach.StateMachine.add("MoveToObject", MoveToObject(),
                                           transitions={"success": "at_object"})

                smach.StateMachine.add("MoveToObjectFlow", move_to_object_sm,
                                       transitions={"at_object": "PickObject"})
                smach.StateMachine.add("PickObject", PickObject(),
                                       transitions={"success": "object_picked"})

            smach.StateMachine.add("PickObjectFlow", pick_object_sm,
                                   transitions={"object_picked": "PlaceObject"})

            # Place Object at the Goal
            smach.StateMachine.add("PlaceObject", PlaceObject(),
                                   transitions={"success": "task_complete"})

        return sm


def main(args=None):
    """Initialize the ROS2 SMACH state machine node."""
    rclpy.init(args=args)
    node = StateMachineNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
