from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='odrive_can',
            executable='odrive_can_node',
            name='odrive_can_node',
            parameters=['/home/robot/Person-Following-Robot/robot/src/odrive_can/odrive_node/launch']
        ),
        
    ])