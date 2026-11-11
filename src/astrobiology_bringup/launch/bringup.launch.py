import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, **launch_arguments):
    """Include a launch file in its own scope.

    The included files all declare a `config` argument (and some `namespace`).  Without a
    scope, the first include's values leak into the later ones, whose defaults then don't
    apply: lights and spectro_cam were loading steppers.yaml.
    """
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments=launch_arguments.items(),
        ),
    ])


def generate_launch_description():
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='',
                              description='Override dry_run for actuator nodes (true/false)'),

        _include('astrobiology_actuators', 'steppers.launch.py', dry_run=dry_run),
        _include('astrobiology_actuators', 'lights.launch.py', dry_run=dry_run),
        _include('spectro_cam', 'spectro_cam.launch.py'),
    ])
