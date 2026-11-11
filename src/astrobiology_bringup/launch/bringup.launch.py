import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    dry_run = LaunchConfiguration('dry_run')

    actuators_dir = get_package_share_directory('astrobiology_actuators')
    spectro_dir = get_package_share_directory('spectro_cam')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='',
                              description='Override dry_run for actuator nodes (true/false)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(actuators_dir, 'launch', 'steppers.launch.py')),
            launch_arguments={'dry_run': dry_run}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(actuators_dir, 'launch', 'lights.launch.py')),
            launch_arguments={'dry_run': dry_run}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(spectro_dir, 'launch', 'spectro_cam.launch.py')),
        ),
    ])
