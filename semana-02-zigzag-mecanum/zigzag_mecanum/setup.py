from setuptools import find_packages, setup

package_name = 'zigzag_mecanum'

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
    description='Semana 02 del Challenge JAR: zigzag aprovechando el movimiento lateral de las ruedas mecanum.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zigzag = zigzag_mecanum.zigzag:main',
        ],
    },
)
