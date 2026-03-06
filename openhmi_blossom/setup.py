from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'openhmi_blossom'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'sequences'),
            glob('config/sequences/*.yaml') + glob('config/sequences/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for OpenHMI/Blossom robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blossom_controller = openhmi_blossom.blossom_controller:main',
            'sequence_player = openhmi_blossom.sequence_player:main',
            'motor_interface = openhmi_blossom.motor_interface:main',
            'gesture_server = openhmi_blossom.gesture_server:main',
            'example_client = examples.example_client:main',
        ],
    },
)
