import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[{'use_sim_time': False}, {'publish_tf': True}],
            arguments=['/home/kittinook/advRobotics_ws/src/day7/config/ekf.yaml']  # Replace with your actual path to ekf.yaml
        )
    ])
