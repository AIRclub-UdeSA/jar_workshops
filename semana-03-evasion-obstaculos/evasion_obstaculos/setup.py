from setuptools import find_packages, setup

package_name = 'evasion_obstaculos'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR Club UdeSA',
    maintainer_email='rafadiaz71@gmail.com',
    description='Semana 03 del Challenge JAR: maquina de estados para avanzar, detectar un choque inminente con el lidar y girar un angulo fijo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # TODO: registrar acá el ejecutable "evasor" para que
            # `ros2 run evasion_obstaculos evasor` lo encuentre. El formato es
            # 'nombre_del_ejecutable = paquete.modulo:funcion' — el paquete es
            # evasion_obstaculos, el módulo es evasor (evasor.py), y la función
            # es main() (la que llama rclpy.init() / rclpy.spin()).
            #
            # Es esta linea de abajo — descomentala (borrá el # de adelante). En este
            # workshop te la damos como ayuda, pero cuando armes un nodo
            # propio de cero la vas a tener que escribir a mano vos:
            
            # 'evasor = evasion_obstaculos.evasor:main',
        ],
    },
)