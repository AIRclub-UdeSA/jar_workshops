from glob import glob

from setuptools import find_packages, setup

package_name = 'launch_rviz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # TODO 1: instalar las carpetas launch/ y rviz/.
        #
        # A diferencia de las semanas anteriores, este paquete no tiene nodos:
        # no hay nada que registrar en entry_points. Lo que hay que instalar
        # son *archivos*: los .launch.py y los .rviz.
        #
        # `colcon build` no copia el paquete entero a install/ — copia solo lo
        # que está declarado acá, en data_files. Si estas dos líneas no están,
        # los archivos existen en tu carpeta de código pero NO en
        # install/launch_rviz/share/, y entonces
        # `ros2 launch launch_rviz evasion.launch.py` falla con
        # "file not found" aunque el archivo esté ahí, delante tuyo.
        #
        # Cada tupla es (destino_dentro_de_share, [lista_de_archivos]). glob()
        # arma esa lista sola, así que no hay que ir agregando cada archivo
        # nuevo a mano.
        #
        # Son estas dos de abajo — descomentalas (borrá el # de adelante):

        # ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR Club UdeSA',
    maintainer_email='rafadiaz71@gmail.com',
    description='Semana 05 del Challenge JAR: launch files y configuraciones de RViz para levantar de una sola vez los workshops anteriores.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Vacío a propósito: este paquete no expone ejecutables propios,
            # solo archivos de launch y de RViz que lanzan nodos de otros
            # paquetes.
        ],
    },
)
