"""Ronda 2 -- speed run (BFS sobre el mapa que dejo explorar.launch.py,
tramos comprimidos, sin pausa entre celdas).

Requiere haber corrido ``explorar.launch.py`` antes (o tener un
``mapa_descubierto.yaml`` valido en la ruta de ``mapa_entrada``).

Ejemplo:
    ros2 launch granprix_bot speedrun.launch.py
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

    speedrun_node = Node(
        package='granprix_bot',
        executable='speedrun_node',
        name='speedrun',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        params_file_arg,
        speedrun_node,
    ])
