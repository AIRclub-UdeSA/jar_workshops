"""Publica el mapa del laberinto vacío en /map.

Uso:
  ros2 launch mapa_laberinto mapa.launch.py
  ros2 launch mapa_laberinto mapa.launch.py mapa:=/ruta/a/otro_mapa.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapa_por_defecto = os.path.join(
        get_package_share_directory('mapa_laberinto'), 'mapas', 'laberinto_vacio.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('mapa', default_value=mapa_por_defecto,
                              description='yaml del mapa (formato map_server)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true al reproducir un bag con --clock'),
        Node(
            package='mapa_laberinto',
            executable='publicar_mapa',
            output='screen',
            parameters=[{
                'mapa': LaunchConfiguration('mapa'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
