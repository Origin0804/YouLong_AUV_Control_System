"""Real hardware bringup launch file.

Launches micro-ROS agent + hw_manager + all control/perception/nav/task nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_enable_ai = DeclareLaunchArgument(
        'enable_ai', default_value='true',
        description='Enable AI perception nodes'
    )
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav', default_value='true',
        description='Enable navigation node'
    )
    declare_enable_task = DeclareLaunchArgument(
        'enable_task', default_value='false',
        description='Enable task runner'
    )
    declare_publish_annotated = DeclareLaunchArgument(
        'publish_annotated', default_value='false',
        description='Publish YOLO annotated images with detection boxes'
    )

    enable_ai = LaunchConfiguration('enable_ai')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_task = LaunchConfiguration('enable_task')
    publish_annotated = LaunchConfiguration('publish_annotated')

    # Hardware manager (replaces sim_bridge on real hardware)
    hw_manager = Node(
        package='uv_hm',
        executable='hw_manager',
        name='hw_manager',
        output='screen',
    )

    basic_motion = Node(
        package='uv_control',
        executable='basic_motion',
        name='basic_motion',
        output='screen',
    )

    vision = Node(
        package='uv_perception',
        executable='vision',
        name='vision',
        output='screen',
        parameters=[{'sim_mode': False, 'publish_annotated': publish_annotated}],
        condition=IfCondition(enable_ai),
    )

    position = Node(
        package='uv_perception',
        executable='position',
        name='position',
        output='screen',
        condition=IfCondition(enable_ai),
    )

    navigator = Node(
        package='uv_nav',
        executable='navigator',
        name='navigator',
        output='screen',
        condition=IfCondition(enable_nav),
    )

    task_runner = Node(
        package='uv_task',
        executable='task_runner',
        name='task_runner',
        output='screen',
        condition=IfCondition(enable_task),
    )

    return LaunchDescription([
        declare_enable_ai,
        declare_enable_nav,
        declare_enable_task,
        declare_publish_annotated,
        hw_manager,
        basic_motion,
        vision,
        position,
        navigator,
        task_runner,
    ])
