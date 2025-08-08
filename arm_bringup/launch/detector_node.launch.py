import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('arm_bringup'), 'launch'))


def generate_launch_description():
    from common import launch_params, node_params
    from launch_ros.actions import Node
    from launch import LaunchDescription

    detector_node = Node(
        package='exchange_slot_detector',
        executable='exchange_slot_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'exchange_slot_detector:='+launch_params['detector_log_level']],
    )

    return LaunchDescription([
        detector_node,
    ])
