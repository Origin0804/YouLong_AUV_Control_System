from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_publish_annotated = DeclareLaunchArgument(
        'publish_annotated', default_value='false',
        description='Publish YOLO annotated images with detection boxes'
    )
    publish_annotated = LaunchConfiguration('publish_annotated')

    return LaunchDescription([
        declare_publish_annotated,
        Node(
            package='uv_perception',
            executable='vision',
            name='vision',
            output='screen',
            parameters=[{'publish_annotated': publish_annotated}],
        ),
        Node(
            package='uv_perception',
            executable='position',
            name='position',
            output='screen',
        ),
    ])
