"""Launch the full collection stack: publisher + sync monitor + (optional) MCAP recorder.

    ros2 launch tactile_umi collect.launch.py sim:=true record:=true session_root:=/data/sessions
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tactile_umi')
    default_params = os.path.join(share, 'config', 'params.yaml')

    sim = LaunchConfiguration('sim')
    record = LaunchConfiguration('record')
    session_root = LaunchConfiguration('session_root')
    params_file = LaunchConfiguration('params_file')
    max_jitter_ms = LaunchConfiguration('max_jitter_ms')
    stall_every = LaunchConfiguration('sim_serial_stall_every')
    stall_ms = LaunchConfiguration('sim_serial_stall_ms')
    cam_jitter_ms = LaunchConfiguration('sim_camera_jitter_ms')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='Use simulated cameras and firmware emulator'),
        DeclareLaunchArgument('record', default_value='true',
                              description='Start the MCAP recorder'),
        DeclareLaunchArgument('session_root', default_value='/data/sessions'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('max_jitter_ms', default_value='2.0'),
        DeclareLaunchArgument('sim_serial_stall_every', default_value='0'),
        DeclareLaunchArgument('sim_serial_stall_ms', default_value='0.0'),
        DeclareLaunchArgument('sim_camera_jitter_ms', default_value='0.0'),

        Node(
            package='tactile_umi', executable='visuo_tactile_publisher',
            name='visuo_tactile_publisher', output='screen',
            parameters=[params_file, {
                'sim': PythonExpression(["'", sim, "'.lower() in ('true', '1', 'yes')"]),
                'sim_serial_stall_every': PythonExpression(['int(', stall_every, ')']),
                'sim_serial_stall_ms': PythonExpression(['float(', stall_ms, ')']),
                'sim_camera_jitter_ms': PythonExpression(['float(', cam_jitter_ms, ')']),
            }],
        ),
        Node(
            package='tactile_umi', executable='sync_monitor',
            name='sync_monitor', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='tactile_umi', executable='mcap_recorder',
            name='mcap_recorder', output='screen',
            condition=IfCondition(record),
            parameters=[params_file, {
                'session_root': session_root,
                'max_jitter_ms': PythonExpression(['float(', max_jitter_ms, ')']),
            }],
        ),
    ])
