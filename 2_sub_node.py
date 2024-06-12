#!/usr/bin/python3

# Import the ROS2 Python client library.
import rclpy
from rclpy.node import Node  # Import the Node class, which is the base for all ROS2 nodes.
from geometry_msgs.msg import Twist, Point  # Import message types for velocity and 2D points.
from turtlesim.msg import Pose  # Import the Pose message type from the turtlesim package.

# Define the DummyNode class, which inherits from the ROS2 Node class.
class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')  # Initialize the node with the name 'dummy_node'.
        self.timer_period = 0.1  # Set the timer period for cyclic tasks (in seconds).
        # Create a publisher to send commands to the '/turtle1/cmd_vel' topic, using the Twist message type.
        self.pub_cmd_vel = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # Create a subscription to listen to Pose messages on the 'turtle1/pose' topic.
        self.create_subscription(Pose, "turtle1/pose", self.sub_pose_callback, 10)
        # Create a subscription to listen to Point messages on the 'mouse_position' topic.
        self.create_subscription(Point, 'mouse_position', self.mouse_sub_callback, 10)
        # Create a timer to call the timer_loop function at each timer interval.
        self.create_timer(self.timer_period, self.timmer_loop)
    
    # Define a callback method for Pose messages. This gets called every time a new Pose message is received.
    def sub_pose_callback(self, msg):
        print(msg)  # Print the Pose message.

    # Define a callback method for Point messages. This gets called every time a new Point message is received.
    def mouse_sub_callback(self, msg: Point):
        print([msg.x, msg.y])  # Print the x and y coordinates from the Point message.

    # Define a method to construct and send velocity commands.
    def cmd_vel(self, v, w):
        cmd_msgs = Twist()  # Create a new Twist message.
        cmd_msgs.linear.x = v  # Set the linear velocity (forward/backward).
        cmd_msgs.angular.z = w  # Set the angular velocity (turning rate).
        self.pub_cmd_vel.publish(cmd_msgs)  # Publish the velocity command to the specified topic.

    # Define the timer loop function that periodically sends velocity commands.
    def timmer_loop(self):
        self.cmd_vel(0.5, 0.5)  # Call cmd_vel with both linear and angular velocities set to 0.5.

# Define the main function that initializes and runs the node.
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS client library.
    node = DummyNode()  # Create an instance of DummyNode.
    rclpy.spin(node)  # Keep the node running until it's shut down.
    node.destroy_node()  # Clean up the node's resources after shutting down.
    rclpy.shutdown()  # Shut down the ROS client library.

# Ensure the main function runs when the script is executed directly.
if __name__ == '__main__':
    main()
