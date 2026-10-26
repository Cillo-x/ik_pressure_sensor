"""Launch file for the IKMSIK pressure sensor node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = get_package_share_directory('ik_pressure_sensor')
    default_params_file = os.path.join(
        share_directory, 'config', 'pressure_sensor.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Absolute path to the parameter YAML. Defaults to the '
                        'one installed with this package.',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial device of the USB-RS485 adapter. Overrides the '
                        'value in params_file.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace, e.g. "sensors". Topics resolve inside it.',
        ),
        Node(
            package='ik_pressure_sensor',
            executable='pressure_node',
            name='ik_pressure_sensor',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {'port': port},
            ],
        ),
    ])
