from setuptools import find_packages, setup

package_name = 'deteccion_color'

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
    description='Semana 04 del Challenge JAR: deteccion de cuadrados rojos con la camara (HSV + OpenCV) y publicacion del resultado en un topico.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # TODO: registrar acá los ejecutables "detector" y "detector_scan"
            # para que `ros2 run deteccion_color detector` / `detector_scan`
            # los encuentren. El formato es
            # 'nombre_del_ejecutable = paquete.modulo:funcion' 
            # los ejecutables tienen que llamarse "detector" y "detector_scan"
        ],
    },
)
