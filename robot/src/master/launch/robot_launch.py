from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    launch_file_path = os.path.join(current_dir, '../ros_odrive/odrive_node/launch/example_launch.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource('/home/robot/Autonomous-Robot/robot/src/ros_odrive/odrive_node/launch/example_launch.yaml')
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='master',
            executable='watchdog_feeder',
            name='watchdog_feed_node',
        ),
    ])