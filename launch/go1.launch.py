##
# Unitree Go1 Ros wrapper launcher for control
# 
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    namespace = 'go1'
    
    # Declare launch arguments for configuration
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.161',
        description='IP Address of the Unitree Go1'
    )

    local_port_arg = DeclareLaunchArgument(
        'local_port',
        default_value='8090',
        description='Local UDP port for listening'
    )

    remote_port_arg = DeclareLaunchArgument(
        'remote_port',
        default_value='8082',
        description='Remote UDP port on the robot'
    )

    # Options: 
    # 'individual': Standard ROS topics (odom, imu, etc)
    # 'combined': One big HighState message
    # 'both': Publishes both
    publish_mode_arg = DeclareLaunchArgument(
        'publish_mode',
        default_value='individual',
        description='Publishing mode: "individual", "combined", or "both"'
    )

    legged_sdk_node = Node(
        package='unitree_ros2_cpp',
        executable='legged_controller',
        name='legged_controller',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'local_port': LaunchConfiguration('local_port'),
            'remote_port': LaunchConfiguration('remote_port'),
            'publish_mode': LaunchConfiguration('publish_mode')
        }]
    )
    
    return LaunchDescription([
        PushRosNamespace(namespace),
        robot_ip_arg,
        local_port_arg,
        remote_port_arg,
        publish_mode_arg,
        legged_sdk_node,
    ])