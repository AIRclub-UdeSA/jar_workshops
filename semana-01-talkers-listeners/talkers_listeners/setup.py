from setuptools import find_packages, setup

package_name = 'talkers_listeners'

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
    description='Semana 01 del Challenge JAR: un talker y un listener en Python con rclpy.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Estos son los ejecutables, al declararlos acá, ROS 2 los va a poder encontrar y ejecutar.
            'talker = talkers_listeners.talker:main',
            'listener = talkers_listeners.listener:main',
        ],
    },
)
