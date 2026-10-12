"""Levanta el evaluador de /victimas con los centros de las cajas del bag."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cajas = os.path.join(get_package_share_directory('evaluador_victimas'), 'config', 'cajas.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='evaluador_victimas', executable='evaluador', name='evaluador', output='screen',
             parameters=[cajas, {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
