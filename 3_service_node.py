#!/usr/bin/python3

# Import the ROS2 Python client library
import rclpy
# Import the Node class, which is the base for all ROS2 nodes
from rclpy.node import Node
# Import message types for velocity and 2D points
from geometry_msgs.msg import Twist, Point
# Import the Pose message type from the turtlesim package
from turtlesim.msg import Pose
# Import the service type for giving a position
from turtlesim_plus_interfaces.srv import GivePosition

# Define the DummyNode class, which inherits from the ROS2 Node class
class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')  # Initialize the node with the name 'dummy_node'
        self.timer_period = 0.1  # Set the timer period for cyclic tasks (in seconds)
        # Create a publisher to send commands to the '/turtle1/cmd_vel' topic, using the Twist message type
        self.pub_cmd_vel = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # Create a subscription to listen to Pose messages on the 'turtle1/pose' topic
        self.create_subscription(Pose, "turtle1/pose", self.sub_pose_callback, 10)
        # Create a subscription to listen to Point messages on the 'mouse_position' topic
        self.create_subscription(Point, 'mouse_position', self.mouse_sub_callback, 10)
        # Create a timer to call the timer_loop function at each timer interval
        self.create_timer(self.timer_period, self.timmer_loop)
        # Create a client to call the 'spawn_pizza' service
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')
    
    # Define a method to send a position request to the 'spawn_pizza' service
    def spawn(self, position):
        # Create a request object for the GivePosition service
        position_request = GivePosition.Request()
        # Set the x and y position in the request
        position_request.x = position[0]
        position_request.y = position[1]
        # Call the service asynchronously
        future = self.spawn_pizza_client.call_async(position_request)
        # Log an info message indicating a pizza spawn request
        self.get_logger().info('spawn a pizza')

    # Define a callback method for Point messages
    def mouse_sub_callback(self, msg: Point):
        # Call the spawn method with the x and y coordinates from the Point message
        self.spawn([msg.x, msg.y])

    # Define a callback method for Pose messages
    def sub_pose_callback(self, msg):
        # Print the Pose message
        print(msg)

    # Define a method to construct and send velocity commands
    def cmd_vel(self, v, w):
        # Create a new Twist message
        cmd_msgs = Twist()
        # Set the linear velocity (forward/backward)
        cmd_msgs.linear.x = v
        # Set the angular velocity (turning rate)
        cmd_msgs.angular.z = w
        # Publish the velocity command to the specified topic
        self.pub_cmd_vel.publish(cmd_msgs)

    # Define the timer loop function that periodically sends velocity commands
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
    # Clean up the node's resources after shutting down
    node.destroy_node()
    # Shut down the ROS client library
    rclpy.shutdown()

# Ensure the main function runs when the script is executed directly
if __name__ == '__main__':
    main()
