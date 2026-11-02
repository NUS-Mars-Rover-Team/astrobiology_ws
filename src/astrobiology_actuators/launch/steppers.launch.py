import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _steppers_node(context):
    parameters = [LaunchConfiguration('config').perform(context)]
    dry_run = LaunchConfiguration('dry_run').perform(context)
    if dry_run:
        parameters.append({'dry_run': dry_run.lower() in ('true', '1', 'yes')})
    return [
        Node(
            package='astrobiology_actuators',
            executable='steppers_node',
            name='steppers_node',
            parameters=parameters,
            output='screen',
        )
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('astrobiology_actuators'), 'config', 'steppers.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Stepper node parameter YAML file'),
        DeclareLaunchArgument('dry_run', default_value='',
                              description='Override dry_run (true/false); empty uses the YAML value'),
        OpaqueFunction(function=_steppers_node),
    ])
