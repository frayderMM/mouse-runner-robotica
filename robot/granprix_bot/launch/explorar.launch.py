"""Ronda 1 -- exploracion (flood fill, celda por celda, LiDAR real).

No lanza el bringup base del robot (driver LiDAR, motores, microROS):
eso debe correr antes, por separado.

Ejemplo:
    ros2 launch granprix_bot explorar.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('granprix_bot')
    default_params_file = os.path.join(pkg_share, 'config', 'granprix_bot_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Archivo YAML de parametros.',
    )
    params_file = LaunchConfiguration('params_file')

    explorer_node = Node(
        package='granprix_bot',
        executable='explorer_node',
        name='explorer',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        params_file_arg,
        explorer_node,
    ])
