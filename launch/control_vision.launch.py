from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    package_path = get_package_share_path('position_control')
    kf_params_file_path = str(package_path / 'config/kalman_filter_params.yaml')

    group = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration('vehicle_name')),
            Node(
                executable='kalman_filter.py',     # not yet coded lol
                package='position_control',
                parameters=[
                    LaunchConfiguration(
                        'kf_params_file_path', default=kf_params_file_path
                    )
                ],
            ),
            Node(executable='setpoint.py', package='position_control'),
            Node(executable='x_controller_vision.py', package='position_control'),
            Node(executable='y_controller_vision.py', package='position_control'),
            Node(executable='z_controller_vision.py', package='position_control'),
            Node(executable='yaw_controller.py', package='position_control'),
        ]
    )
    launch_description.add_action(group)
    return launch_description
