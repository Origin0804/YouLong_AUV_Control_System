from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_debug_mode = DeclareLaunchArgument(
        'debug_mode', default_value='false',
        description='Enable debug mode: suppress auto-start, expose /task/exec service'
    )
    debug_mode = LaunchConfiguration('debug_mode')

    return LaunchDescription([
        declare_debug_mode,
        Node(
            package='uv_task',
            executable='task_runner',
            name='task_runner',
            output='screen',
            parameters=[{'debug_mode': debug_mode}],
        ),
    ])
