"""
Semana 05 — launch de la semana 03 (evasión de obstáculos).

Reemplaza las tres terminales de la semana 03 (simulador, nodo, RViz) por un
solo comando:

    ros2 launch launch_rviz evasion.launch.py

Este archivo viene completo, sin TODOs: es el que hay que leer antes de
encarar deteccion_color.launch.py, que sí los tiene. La lógica es la misma
en los dos — acá está resuelta y explicada, allá la tienen que reproducir.
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

    # get_package_share_directory devuelve la carpeta *instalada* del paquete
    # (install/<paquete>/share/<paquete>), no la del código fuente. Por eso
    # importa el TODO 1 de setup.py: si los archivos no se instalan, no están
    # acá y estos os.path.join apuntan a rutas que no existen.
    pkg_gazebo = get_package_share_directory('yahboom_rosmaster_gazebo')
    pkg_bringup = get_package_share_directory('launch_rviz')

    launch_simulador = os.path.join(
        pkg_gazebo, 'launch', 'rosmaster_gazebo_fortress.launch.py')
    mundo_por_defecto = os.path.join(pkg_gazebo, 'worlds', 'cafe.world')
    config_rviz = os.path.join(pkg_bringup, 'rviz', 'evasion.rviz')

    # Los argumentos de launch son los que se pasan por línea de comandos con
    # `nombre:=valor`, igual que ya venían haciendo con el `world:=` del
    # simulador. Son distintos de los parámetros ROS: un argumento de launch lo
    # lee este archivo, un parámetro ROS lo lee el nodo.
    declarar_world = DeclareLaunchArgument(
        'world', default_value=mundo_por_defecto,
        description='Ruta al .world de Gazebo que se va a cargar.')
    declarar_distancia = DeclareLaunchArgument(
        'distancia_choque_m', default_value='0.6',
        description='Distancia (m) a la que el evasor considera el choque inminente.')
    declarar_angulo_giro = DeclareLaunchArgument(
        'angulo_giro_deg', default_value='110.0',
        description='Cuántos grados gira el robot cada vez que ve un obstáculo.')

    # IncludeLaunchDescription mete otro launch file entero adentro de este.
    # Es lo que evita tener que reescribir acá todo lo que hace el simulador.
    #
    # Dos detalles que importan:
    #   - launch_arguments quiere pares (clave, valor) — de ahí el .items().
    #     Y los valores van siempre como texto, incluso los booleanos: 'false'
    #     entre comillas, no False de Python.
    #   - rviz:='false' apaga el RViz que el simulador abre por su cuenta. Sin
    #     esto se abrirían dos RViz: el suyo con la config por defecto del
    #     simulador, y el nuestro con la config de este workshop.
    simulador = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_simulador),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'motion_profile': 'ideal',
            'rviz': 'false',
        }.items(),
    )

    # Los parámetros ROS del nodo evasor, los mismos que en la semana 03 venían
    # sueltos en la línea de comandos detrás de `--ros-args -p`.
    #
    # ParameterValue(..., value_type=float) es necesario porque LaunchConfiguration
    # devuelve *texto*: sin el value_type, el nodo recibiría la cadena "0.6"
    # donde declaró un double y arrancaría con un error de tipo. Los valores
    # fijos, en cambio, se escriben directamente como números de Python.
    #
    # use_sim_time es el parámetro más importante de todos acá: le dice al nodo
    # que use el reloj de la simulación (/clock) en vez del reloj de la
    # computadora. Con el simulador corriendo va SIEMPRE en True.
    parametros_evasor = [{
        'use_sim_time': True,
        'distancia_choque_m': ParameterValue(
            LaunchConfiguration('distancia_choque_m'), value_type=float),
        'angulo_giro_deg': ParameterValue(
            LaunchConfiguration('angulo_giro_deg'), value_type=float),
        'angulo_vision_deg': 90.0,
    }]

    # El nodo evasor: el mismo `ros2 run evasion_obstaculos evasor` de la
    # semana 03, declarado en vez de tipeado.
    #   - package/executable: igual que los dos primeros argumentos de
    #     ros2 run.
    #   - name: con qué nombre se registra el nodo en ROS 2 (lo que ven en
    #     `ros2 node list`).
    #   - output='screen': sin esto los logs van a un archivo y parece que el
    #     nodo no hace nada.
    #   - parameters: la lista que se armó arriba.
    evasor = Node(
        package='evasion_obstaculos',
        executable='evasor',
        name='evasor',
        output='screen',
        parameters=parametros_evasor,
    )

    # RViz también es un nodo, así que también se lanza con Node(...):
    #   - package y executable son los dos 'rviz2'.
    #   - la config se pasa con arguments=['-d', config_rviz] — es exactamente
    #     el `rviz2 -d archivo.rviz` que se escribiría a mano.
    #   - parameters=[{'use_sim_time': True}], por lo mismo que el evasor: sin
    #     esto RViz compara los timestamps de /scan contra el reloj de la
    #     compu, no encuentra transformadas válidas, y el LaserScan aparece y
    #     desaparece o directamente nunca se dibuja.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_rviz],
        parameters=[{'use_sim_time': True}],
    )

    # El LaunchDescription es la lista de todo lo que hay que lanzar. No es
    # una secuencia de pasos: no espera a que el simulador "termine" para
    # arrancar el evasor y RViz, los lanza a todos en paralelo. Mientras algo
    # no esté en esta lista, el launch lo ignora por completo aunque esté
    # perfectamente escrito arriba.
    return LaunchDescription([
        # Los DeclareLaunchArgument van primero: definen los valores que el
        # resto de la descripción va a leer.
        declarar_world,
        declarar_distancia,
        declarar_angulo_giro,
        simulador,
        evasor,
        rviz,
    ])
