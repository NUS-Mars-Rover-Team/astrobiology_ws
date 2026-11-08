import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _raman_node(context):
    parameters = [LaunchConfiguration('config').perform(context)]
    port = LaunchConfiguration('port').perform(context)
    if port:
        parameters.append({'port': port})
    return [
        Node(
            package='raman_spectrometer',
            executable='raman_node',
            name='raman_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=parameters,
            output='screen',
        )
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('raman_spectrometer'), 'config', 'raman.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Parameter YAML file'),
        DeclareLaunchArgument('port', default_value='',
                              description='Override the serial port'),
        DeclareLaunchArgument('namespace', default_value='raman'),
        OpaqueFunction(function=_raman_node),
    ])
