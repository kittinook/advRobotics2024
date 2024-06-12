#!/usr/bin/python3
# Import the ROS2 Python client library
import rclpy
# Import the Node class, which is the base for all ROS2 nodes
from rclpy.node import Node
# Import custom services for getting and setting the pose
from adv_interface.srv import GetPose, SetGoal
# Import numpy for numerical operations
import numpy as np

# Define the DummyNode class, which inherits from the ROS2 Node class
class DummyNode(Node):
    def __init__(self):
        super().__init__('controller')  # Initialize the node with the name 'controller'
        
        # Create clients for the GetPose and SetGoal services
        self.get_pose_client = self.create_client(GetPose, "/get_pose")
        self.set_pose_client = self.create_client(SetGoal, "/set_goal")
        
        # Create a timer to call the timmer_loop function at each timer interval
        self.create_timer(0.1, self.timmer_loop)
        
        # Initialize the current pose and waypoints
        self.current_pose = np.array([0.0, 0.0])  # Current pose of the robot
        self.via_points = np.array(  # Waypoints for the robot to follow
            [[1, 1],
             [2, 2],
             [5, 5],
             [3, 3],
             [7, 3],
             [10, 5],
             [2, 2]]
        )
        self.n_point = self.via_points.shape[0]  # Number of waypoints
        self.i = 0  # Index of the current waypoint
        
        # Call the service to set the initial pose to the first waypoint
        self.call_set_pose_service(self.via_points[self.i, 0], self.via_points[self.i, 1])

    # Timer loop function that is called periodically
    def timmer_loop(self):
        # Wait for the services to be available
        if self.get_pose_client.wait_for_service(timeout_sec=1.0) and self.set_pose_client.wait_for_service(timeout_sec=1.0):
            # Call the service to get the current pose
            self.call_get_pose_service()
            if self.i < self.n_point:
                # Calculate the difference between the current waypoint and the current pose
                dp = self.via_points[self.i, :] - self.current_pose
                # Call the service to set the pose to the current waypoint
                self.call_set_pose_service(self.via_points[self.i, 0], self.via_points[self.i, 1])
                # If the robot is close enough to the current waypoint, move to the next waypoint
                if np.linalg.norm(dp) < 0.1:
                    self.i += 1
            else:
                print("end")  # Print "end" when all waypoints have been visited
        else:
            self.get_logger().info("Still waiting for /get_pose service...")

    # Call the GetPose service asynchronously
    def call_get_pose_service(self):
        self.future = self.get_pose_client.call_async(GetPose.Request())
        self.future.add_done_callback(self.handle_get_pose_response)

    # Call the SetGoal service asynchronously
    def call_set_pose_service(self, x, y):
        cmd = SetGoal.Request()
        cmd.position.x = float(x)
        cmd.position.y = float(y)
        self.future = self.set_pose_client.call_async(cmd)
        self.future.add_done_callback(self.handle_set_pose_response)

    # Handle the response from the SetGoal service
    def handle_set_pose_response(self, future):
        try:
            response = future.result()
            # self.get_logger().info(f'Received response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    # Handle the response from the GetPose service
    def handle_get_pose_response(self, future):
        try:
            response = future.result()
            self.current_pose[0] = response.position.x
            self.current_pose[1] = response.position.y
            # self.get_logger().info(f'Received response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

# Main function that initializes and runs the node
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS client library
    node = DummyNode()  # Create an instance of DummyNode
    rclpy.spin(node)  # Keep the node running until it's shut down
    node.destroy_node()  # Clean up the node's resources after shutting down
    rclpy.shutdown()  # Shut down the ROS client library

# Ensure the main function runs when the script is executed directly
if __name__ == '__main__':
    main()
