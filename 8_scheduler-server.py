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
# Import the standard Empty service
from std_srvs.srv import Empty
# Import numpy for numerical operations
import numpy as np
# Import custom services for setting a goal, getting a random goal, and getting the pose
from adv_interface.srv import SetGoal, RandGoal, GetPose

# Define the DummyNode class, which inherits from the ROS2 Node class
class DummyNode(Node):
    def __init__(self):
        super().__init__('controller')  # Initialize the node with the name 'controller'
        
        # Declare parameters for the node
        self.declare_parameters(namespace='', parameters=[('gain', 5.0)])
        self.declare_parameters(namespace='', parameters=[('tolerance', 0.1)])
        
        self.timer_period = 0.1  # Set the timer period for cyclic tasks (in seconds)
        
        # Create a publisher to send commands to the '/turtle1/cmd_vel' topic, using the Twist message type
        self.pub_cmd_vel = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        # Create a subscription to listen to Pose messages on the 'turtle1/pose' topic
        self.create_subscription(Pose, "turtle1/pose", self.sub_pose_callback, 10)
        
        # Create a subscription to listen to Point messages on the 'mouse_position' topic
        self.create_subscription(Point, 'mouse_position', self.mouse_sub_callback, 10)
        
        # Create a timer to call the timmer_loop function at each timer interval
        self.create_timer(self.timer_period, self.timmer_loop)
        
        # Create services to handle goal setting, random goal, and getting pose requests
        self.set_goal_service = self.create_service(SetGoal, '/set_goal', self.set_goal_callback)
        self.rand_goal_service = self.create_service(RandGoal, '/rand_goal', self.rand_goal_callback)
        self.get_pose_service = self.create_service(GetPose, '/get_pose', self.get_pose_callback)
        
        # Create clients to call the 'spawn_pizza' and 'eat' services
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, "/turtle1/eat")
        
        # Initialize control flags and positions
        self.is_start = False  # Flag to indicate if the robot should start moving
        self.target = np.array([0.0, 0.0])  # Target position for the robot
        self.current_position = np.array([0.0, 0.0, 0.0])  # Current position of the robot
    
    # Define a callback method for the GetPose service
    def get_pose_callback(self, request, response):
        response.position = Point()
        response.position.x = self.current_position[0]
        response.position.y = self.current_position[1]
        return response
        
    # Define a callback method for the RandGoal service
    def rand_goal_callback(self, request, response):
        # Generate a random goal position within a specified range
        goal = 9 * np.random.rand(2) + 0.5
        # Set the response position to the generated goal
        response.position = Point()
        response.position.x = goal[0]
        response.position.y = goal[1]
        # Call the spawn method to update the target position
        self.spawn(goal)
        # Set the start flag to true
        self.is_start = True
        return response
    
    # Define a callback method for the SetGoal service
    def set_goal_callback(self, request, response):
        # Set the target position based on the request
        self.target = np.array([request.position.x, request.position.y])
        # Call the spawn method to update the target position
        self.spawn([self.target[0], self.target[1]])
        # Set the start flag to true
        self.is_start = True
        return response
    
    # Define a method to send a position request to the 'spawn_pizza' service
    def spawn(self, position):
        # Create a request object for the GivePosition service
        position_request = GivePosition.Request()
        # Set the x and y position in the request
        position_request.x = position[0]
        position_request.y = position[1]
        # Update the target position
        self.target[0] = position[0]
        self.target[1] = position[1]
        # Call the service asynchronously
        future = self.spawn_pizza_client.call_async(position_request)
        # Log an info message indicating a pizza spawn request
        self.get_logger().info('spawn a pizza')

    # Define a method to send a request to the 'eat' service
    def eat(self):
        # Create an empty request object for the Empty service
        empty = Empty.Request()
        # Call the service asynchronously
        future = self.eat_pizza_client.call_async(empty)
        # Log an info message indicating an eat request
        self.get_logger().info('eat a pizza')

    # Define a callback method for Point messages
    def mouse_sub_callback(self, msg: Point):
        # Call the spawn method with the x and y coordinates from the Point message
        self.spawn([msg.x, msg.y])
        # Set the start flag to true
        self.is_start = True

    # Define a callback method for Pose messages
    def sub_pose_callback(self, msg):
        # Update the current position with the values from the Pose message
        self.current_position[0] = msg.x
        self.current_position[1] = msg.y
        self.current_position[2] = msg.theta

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
        if self.is_start:
            # Calculate the difference between the target and current positions
            dp = self.target - self.current_position[:2]
            # Calculate the angular error
            e = np.arctan2(dp[1], dp[0]) - self.current_position[2]
            # Get the gain parameter
            K = self.get_parameter('gain').get_parameter_value().double_value
            # Calculate the angular velocity
            w = K * np.arctan2(np.sin(e), np.cos(e))
            # Get the tolerance parameter
            tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
            # If the distance to the target is greater than the tolerance, set linear velocity
            if np.linalg.norm(dp) > tolerance:
                v = 1.0
            else:
                # Otherwise, stop the robot and call the eat method
                v = 0.0
                self.eat()
            # Send the velocity command
            self.cmd_vel(v, w)

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
