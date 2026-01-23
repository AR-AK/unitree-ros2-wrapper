##
# Based on tutlesim launch from: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html#write-the-launch-file
# Later modified to work for unitree_ros2_cpp package.
#
# Unitree Go1 Ros wrapper launcher for control
# 
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    namespace = 'go1'
    
    legged_sdk_node = Node(
        package='unitree_ros2_cpp',
        executable='legged_controller',
        name='legged_controller',
    )
    
    return LaunchDescription([
        PushRosNamespace(namespace),
        legged_sdk_node,
    ])


