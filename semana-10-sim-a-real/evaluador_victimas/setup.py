from glob import glob

from setuptools import find_packages, setup

package_name = 'evaluador_victimas'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR Club UdeSA',
    maintainer_email='airclub@udesa.edu.ar',
    description='Semana 10 del Challenge JAR: puntúa los reportes en /victimas contra las cajas del bag.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'evaluador = evaluador_victimas.evaluador:main',
        ],
    },
)
