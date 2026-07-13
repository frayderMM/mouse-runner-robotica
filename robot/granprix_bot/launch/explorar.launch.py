"""Ronda 1 -- exploracion (flood fill, celda por celda, LiDAR real).

No lanza el bringup base del robot (driver LiDAR, motores, microROS):
eso debe correr antes, por separado.

Argumentos:
    usar_dashboard (true|false) activa el emisor del dashboard web
                                 (ver robot/README.md seccion 7 y
                                 web/index.html, se abre en la laptop
                                 apuntando a la IP del robot)

Ejemplo:
    ros2 launch granprix_bot explorar.launch.py
    ros2 launch granprix_bot explorar.launch.py usar_dashboard:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('granprix_bot')
    default_params_file = os.path.join(pkg_share, 'config', 'granprix_bot_params.yaml')
    default_mapa_referencia = os.path.join(pkg_share, 'config', 'pista_ejemplo_referencia.json')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Archivo YAML de parametros.',
    )
    usar_dashboard_arg = DeclareLaunchArgument(
        'usar_dashboard',
        default_value='true',
        description='Activa el emisor del dashboard web (ver web/index.html).',
    )

    params_file = LaunchConfiguration('params_file')
    usar_dashboard = LaunchConfiguration('usar_dashboard')

    explorer_node = Node(
        package='granprix_bot',
        executable='explorer_node',
        name='explorer',
        output='screen',
        parameters=[params_file],
    )

    web_dashboard_node = Node(
        package='granprix_bot',
        executable='web_dashboard_node',
        name='web_dashboard',
        output='screen',
        parameters=[params_file, {'mapa_referencia': default_mapa_referencia}],
        condition=IfCondition(usar_dashboard),
    )

    return LaunchDescription([
        params_file_arg,
        usar_dashboard_arg,
        explorer_node,
        web_dashboard_node,
    ])
