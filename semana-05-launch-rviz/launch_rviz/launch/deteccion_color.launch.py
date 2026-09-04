"""
Semana 05 — launch de la semana 04 (detección de color).

Igual que evasion.launch.py, pero levantando los DOS nodos de la semana 04 a
la vez (`detector` y `detector_scan`) más RViz:

    ros2 launch launch_rviz deteccion_color.launch.py

Leé evasion.launch.py primero: ahí está resuelto y comentado exactamente el
mismo patrón que tenés que reproducir acá — un Node por nodo del workshop, más
RViz, todos sumados al LaunchDescription del final. Solo vienen resueltos acá
el simulador y el armado de los parámetros compartidos.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_sim = get_package_share_directory('yahboom_rosmaster_bringup')
    pkg_gazebo = get_package_share_directory('yahboom_rosmaster_gazebo')
    pkg_launch_rviz = get_package_share_directory('launch_rviz')

    launch_simulador = os.path.join(
        pkg_sim, 'launch', 'rosmaster_x3_sim.launch.py')
    mundo_por_defecto = os.path.join(
        pkg_gazebo, 'worlds', 'laberinto_simple_victimas.world')

    # Este archivo todavía no existe: lo vas a armar vos desde la GUI de RViz
    # (ver el README). Hasta que lo guardes con este nombre exacto y vuelvas a
    # buildear, RViz va a abrir vacío y a quejarse de que no encuentra el
    # archivo.
    config_rviz = os.path.join(pkg_launch_rviz, 'rviz', 'deteccion_color.rviz')

    declarar_world = DeclareLaunchArgument(
        'world', default_value=mundo_por_defecto,
        description='Ruta al .world de Gazebo que se va a cargar.')
    declarar_saturacion = DeclareLaunchArgument(
        'saturacion_min', default_value='120.0',
        description='Saturación mínima del filtro HSV, compartida por los dos nodos.')

    simulador = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_simulador),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'motion_profile': 'ideal',
            'rviz': 'false',
        }.items(),
    )

    # Acá se ve una de las ventajas reales de un launch file. En la semana 04,
    # calibrar el filtro de color a mano quería decir acordarse de pasarle el
    # mismo `-p saturacion_min:=...` a los dos nodos, en dos terminales
    # distintas, sin equivocarse. Definido una vez acá, el mismo valor va a los
    # dos: se calibra en un solo lugar y no pueden quedar desincronizados.
    parametros_color = [{
        'use_sim_time': True,
        'saturacion_min': ParameterValue(
            LaunchConfiguration('saturacion_min'), value_type=float),
    }]

    # TODO 3: el nodo `detector` del paquete `deteccion_color`, con
    # parameters=parametros_color. Mirá cómo evasion.launch.py arma el Node
    # del evasor — es exactamente la misma forma, solo cambian package,
    # executable, name y la lista de parámetros.

    # TODO 4: el nodo `detector_scan`, del mismo paquete, también con
    # parameters=parametros_color.
    #
    # Ojo con este: detector_scan busca la transformada entre el lidar y la
    # cámara en tf2, y esa búsqueda se hace por timestamp. Si el nodo corre con
    # el reloj de la computadora mientras el simulador publica con el reloj de
    # la simulación, los tiempos no coinciden y la transformada nunca aparece
    # ("lookup would require extrapolation into the past"). El use_sim_time que
    # ya viene en parametros_color es justamente lo que evita eso.

    # TODO 5: RViz, con arguments=['-d', config_rviz] y use_sim_time en True.
    # Es el mismo Node de rviz2 que ya viene resuelto en evasion.launch.py —
    # cambiá solo qué config_rviz le pasás.

    # TODO 6: sumá acá los tres nodos de los TODO 3, 4 y 5. Mientras estén
    # afuera de esta lista, el launch los ignora por completo aunque las
    # variables estén perfectamente escritas arriba — mismo detalle que en
    # evasion.launch.py.
    return LaunchDescription([
        declarar_world,
        declarar_saturacion,
        simulador,
    ])
