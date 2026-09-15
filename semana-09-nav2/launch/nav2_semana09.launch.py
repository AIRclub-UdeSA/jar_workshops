#!/usr/bin/env python3
"""Bringup de semana-09-nav2 — Checkpoints 1-4 (workshop completo): levanta
el simulador (laberinto_simple_victimas), el stack de localización de Nav2
(map_server + amcl, vía localization_launch.py) y el stack de navegación
(costmaps + controller + planner + bt_navigator, vía navigation_launch.py),
ambos de nav2_bringup, con RViz. Se corre directo por ruta, sin paquete
propio: `ros2 launch nav2_semana09.launch.py`. Un solo bringup para los 4
checkpoints — los Checkpoints 3-4 no agregan nodos nuevos, controller y
planner ya están arriba desde el arranque; lo que cambia entre checkpoints
es lo que probás sobre ese mismo bringup (2D Pose Estimate, Nav2 Goal)."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('yahboom_rosmaster_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    # Sin paquete propio: el params_file por default vive junto a este
    # launch file (../nav2_params.yaml), no en un share/ instalado.
    workshop_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    default_world = os.path.join(pkg_gazebo, 'worlds', 'laberinto_simple_victimas.world')
    default_map = os.path.join(pkg_gazebo, 'maps', 'laberinto_simple.yaml')
    default_params_file = os.path.join(workshop_dir, 'nav2_params.yaml')
    default_rviz = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_map = DeclareLaunchArgument('map', default_value=default_map)
    declare_params_file = DeclareLaunchArgument('params_file', default_value=default_params_file)
    declare_motion_profile = DeclareLaunchArgument('motion_profile', default_value='ideal')
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_rviz_config = DeclareLaunchArgument('rviz_config', default_value=default_rviz)
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')

    simulador = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yahboom_rosmaster_bringup'),
                'launch', 'rosmaster_x3_sim.launch.py',
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'motion_profile': LaunchConfiguration('motion_profile'),
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
        }.items(),
    )

    localizacion_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    navegacion_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_world,
        declare_map,
        declare_params_file,
        declare_motion_profile,
        declare_gui,
        declare_rviz_config,
        declare_autostart,
        simulador,
        localizacion_nav2,
        navegacion_nav2,
        rviz,
    ])
