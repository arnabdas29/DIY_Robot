from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_car_controller',
            executable='lidar_controller_node',
            name='lidar_controller_node',
            output='screen',
        )
    ])
