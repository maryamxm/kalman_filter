import os
from glob import glob

from setuptools import setup

package_name = 'kalman_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    description=(
        'Standalone ROS2 recreation of the B31YS Kalman filter assignment'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_filter = kalman_demo.kalman_filter:main',
            'student_trajectory = kalman_demo.student_trajectory:main',
            'fake_gps = kalman_demo.fake_gps:main',
        ],
    },
)
