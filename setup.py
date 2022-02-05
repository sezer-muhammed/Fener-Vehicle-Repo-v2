from setuptools import setup
import os
from glob import glob

package_name = 'fener_package_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sezer',
    maintainer_email='e230531@metu.edu.tr',
    description='Launch and Run files for Fener Research Kit',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'vehicle_driver = fener_package_v2.fener_arduino_comm:main',
            'bno055_publisher = fener_package_v2.fener_bno055_pub:main',
            'camera_node = fener_package_v2.fener_camera_node:main',
            'lidar_publisher = fener_package_v2.fener_lidar_pub:main',
        ],
    },
)
