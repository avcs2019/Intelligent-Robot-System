import rclpy
from rclpy.node import Node
import smach
import time
import os


# Define State A
class StateA(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_state_b'])

    def execute(self, userdata):
        rclpy.logging.get_logger('StateA').info("Executing State A")
        time.sleep(2)  # Simulate processing
        return 'to_state_b'


# Define State B
class StateB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_state_a'])

    def execute(self, userdata):
        rclpy.logging.get_logger('StateB').info("Executing State B")
        time.sleep(2)  # Simulate processing
        return 'to_state_a'


# ROS 2 Node for running the State Machine
class StateMachineNode(Node):
    def __init__(self):
        super().__init__('smach2_state_machine')

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['shutdown'])

        # Add states
        with self.sm:
            smach.StateMachine.add('STATE_A', StateA(), transitions={'to_state_b': 'STATE_B'})
            smach.StateMachine.add('STATE_B', StateB(), transitions={'to_state_a': 'STATE_A'})

        # Generate state machine visualization
        self.save_state_machine_diagram()

        # Execute state machine
        self.get_logger().info("Starting State Machine...")
        self.execute_sm()

    def execute_sm(self):
        self.sm.execute()

    def save_state_machine_diagram(self):
        """Manually generate a DOT file for the state machine."""
        dot_file = "/tmp/state_machine.dot"
        with open(dot_file, "w") as f:
            f.write("digraph StateMachine {\n")
            f.write("    STATE_A -> STATE_B [label=\"to_state_b\"];\n")
            f.write("    STATE_B -> STATE_A [label=\"to_state_a\"];\n")
            f.write("}\n")

        self.get_logger().info(f"State machine DOT file saved to {dot_file}")
        self.convert_dot_to_image(dot_file)

    def convert_dot_to_image(self, dot_file):
        """Convert the DOT file to an image format (PNG)."""
        output_file = dot_file.replace(".dot", ".png")
        os.system(f"dot -Tpng {dot_file} -o {output_file}")
        self.get_logger().info(f"State machine diagram saved as {output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
