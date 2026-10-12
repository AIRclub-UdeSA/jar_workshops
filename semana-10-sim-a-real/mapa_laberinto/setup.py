from glob import glob

from setuptools import find_packages, setup

package_name = 'mapa_laberinto'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/mapas', glob('mapas/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIR Club UdeSA',
    maintainer_email='airclub@udesa.edu.ar',
    description='Semana 10 del Challenge JAR: publica el mapa del laberinto vacío en /map, retenido (transient local).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'publicar_mapa = mapa_laberinto.publicar_mapa:main',
        ],
    },
)
