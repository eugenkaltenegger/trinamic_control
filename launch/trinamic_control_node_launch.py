import os

from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_file_path = os.path.join(
            FindPackageShare('trinamic_control').find('trinamic_control'),
            'config',
            'trinamic_control_config.yaml')

    return LaunchDescription([
        SetLaunchConfiguration('config_file_path', config_file_path),
        Node(
            package='trinamic_control',
            executable='trinamic_control_node',
            name='trinamic_control_node',
            parameters=[{'config_file_path': config_file_path}]
        )
    ])
