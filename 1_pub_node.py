#!/usr/bin/python3

# Import the ROS2 Python client library
import rclpy
# Import the Node class, which is the base for all ROS2 nodes
from rclpy.node import Node
# Import the Twist message type, which is used to send velocity commands
from geometry_msgs.msg import Twist

# Define a class DummyNode, inheriting from Node
class DummyNode(Node):
    def __init__(self):
        # Initialize the node with the name 'dummy_node'
        super().__init__('pub_node')
        # Set the timer period for cyclic tasks (in seconds)
        self.timer_period = 0.1
        # Create a publisher to send commands to a topic
        # '/turtle1/cmd_vel' is the topic where velocity commands are sent
        # Twist is the message type, and 10 is the queue size
        self.pub_cmd_vel = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # Create a timer to call the timmer_loop function at each timer interval
        self.create_timer(self.timer_period, self.timmer_loop)

    # Define a method to construct and send velocity commands
    def cmd_vel(self, v, w):
        # Create a new Twist message
        cmd_msgs = Twist()
        # Set linear velocity (forward/backward)
        cmd_msgs.linear.x = v
        # Set angular velocity (turning rate)
        cmd_msgs.angular.z = w
        # Publish the velocity command to the specified topic
        self.pub_cmd_vel.publish(cmd_msgs)

    # Define the timer loop function
    def timmer_loop(self):
        # Call cmd_vel with both linear and angular velocities set to 0.5
        self.cmd_vel(0.5, 0.5)

# Define the main function that initializes and runs the node
def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of DummyNode
    node = DummyNode()
    # Keep the node running until it's shut down
    rclpy.spin(node)
    # Clean up the node's resources
    node.destroy_node()
    # Shut down the ROS client library
    rclpy.shutdown()

# Python boilerplate to ensure the main function runs when the script is executed
if __name__ == '__main__':
    main()
