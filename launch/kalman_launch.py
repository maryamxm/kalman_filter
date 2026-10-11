"""
Usage:
    ros2 launch kalman_demo kalman_launch.py
    ros2 launch kalman_demo kalman_launch.py student_name:="Your Name"
    ros2 launch kalman_demo kalman_launch.py pattern:=<valid patterns>

Valid values for `pattern`: auto (default, picks from student_name's
hash like the original assignment), circle, square, figure8, curvy,
spiral.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kalman_demo')
    rviz_config = os.path.join(pkg_share, 'rviz', 'kalman_demo.rviz')

    student_name_arg = DeclareLaunchArgument(
        'student_name',
        default_value='Maryam Mosadegh',
        description='Used only when pattern=auto: selects the '
                     'CIRCLE/SQUARE/FIGURE-8 trajectory via a hash of this '
                     'name, same as the original assignment.',
    )
    pattern_arg = DeclareLaunchArgument(
        'pattern',
        default_value='auto',
        description='Trajectory to drive: auto, circle, square, figure8, '
                     'curvy, or spiral.',
    )

    return LaunchDescription([
        student_name_arg,
        pattern_arg,
        Node(
            package='kalman_demo',
            executable='fake_gps',
            name='fake_odometry_and_gps',
            output='screen',
        ),
        Node(
            package='kalman_demo',
            executable='student_trajectory',
            name='student_trajectory_node',
            output='screen',
            parameters=[{
                'student_name': LaunchConfiguration('student_name'),
                'pattern': LaunchConfiguration('pattern'),
            }],
        ),
        Node(
            package='kalman_demo',
            executable='kalman_filter',
            name='kalman_filter_simple',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
