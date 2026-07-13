"""Ronda 2 -- speed run (BFS sobre el mapa que dejo explorar.launch.py,
tramos comprimidos, sin pausa entre celdas).

Requiere haber corrido ``explorar.launch.py`` antes (o tener un
``mapa_descubierto.yaml`` valido en la ruta de ``mapa_entrada``).

Argumentos:
    usar_dashboard (true|false) activa el emisor del dashboard web
                                 (ver robot/README.md seccion 7 y
                                 web/index.html)

Ejemplo:
    ros2 launch granprix_bot speedrun.launch.py
    ros2 launch granprix_bot speedrun.launch.py usar_dashboard:=false
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

    speedrun_node = Node(
        package='granprix_bot',
        executable='speedrun_node',
        name='speedrun',
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
        speedrun_node,
        web_dashboard_node,
    ])
