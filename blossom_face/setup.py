from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'blossom_face'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Animated face display for Blossom robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_display = blossom_face.face_display:main',
            'face_animator = blossom_face.face_animator:main',
        ],
    },
)
