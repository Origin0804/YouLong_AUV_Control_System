"""HIL (Hardware-in-the-Loop) bringup launch file.

Launches Stonefish simulator + sim_bridge in HIL mode (camera passthrough only)
+ micro-ROS agent + all control/perception/nav/task nodes.

The MCU (STM32) replaces sim_bridge's PID/thruster-mixing/ZIT6-state logic,
communicating via micro-ROS over serial with the MicroXRCEAgent.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Arguments ──────────────────────────────────────────────────
    declare_enable_ai = DeclareLaunchArgument(
        'enable_ai', default_value='false',
        description='Enable AI perception nodes'
    )
    declare_enable_nav = DeclareLaunchArgument(
        'enable_nav', default_value='false',
        description='Enable navigation node'
    )
    declare_enable_task = DeclareLaunchArgument(
        'enable_task', default_value='false',
        description='Enable task runner'
    )
    declare_enable_motion = DeclareLaunchArgument(
        'enable_motion', default_value='false',
        description='Enable motion control nodes'
    )
    declare_scenario = DeclareLaunchArgument(
        'scenario', default_value='underwater_xunyun.scn',
        description='Stonefish scenario file name (in Data/ directory)'
    )
    declare_serial_dev = DeclareLaunchArgument(
        'serial_dev', default_value='/dev/ttyUSB0',
        description='Serial device for micro-ROS agent (MCU connection)'
    )
    declare_serial_baud = DeclareLaunchArgument(
        'serial_baud', default_value='921600',
        description='Baud rate for micro-ROS serial link'
    )
    declare_publish_annotated = DeclareLaunchArgument(
        'publish_annotated', default_value='false',
        description='Publish YOLO annotated images with detection boxes'
    )

    enable_ai = LaunchConfiguration('enable_ai')
    enable_nav = LaunchConfiguration('enable_nav')
    enable_task = LaunchConfiguration('enable_task')
    enable_motion = LaunchConfiguration('enable_motion')
    serial_dev = LaunchConfiguration('serial_dev')
    serial_baud = LaunchConfiguration('serial_baud')
    publish_annotated = LaunchConfiguration('publish_annotated')

    # ── Stonefish simulator ────────────────────────────────────────
    from ament_index_python.packages import get_package_share_directory
    stonefish_share = get_package_share_directory('stonefish_ros2')
    workspace_root = os.path.abspath(os.path.join(stonefish_share, '..', '..', '..', '..'))
    stonefish_source_dir = os.path.join(workspace_root, 'src', 'stonefish_ros2')
    simulation_data_dir = os.path.join(stonefish_source_dir, 'Data')

    stonefish_sim = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        namespace='stonefish_ros2',
        name='stonefish_simulator',
        arguments=[
            simulation_data_dir,
            os.path.join(simulation_data_dir, 'underwater_xunyun.scn'),
            '100.0',
            '1280',
            '720',
            'high',
        ],
        output='screen',
    )

    # ── sim_bridge in HIL mode (camera passthrough only) ───────────
    sim_bridge = Node(
        package='uv_sim',
        executable='sim_bridge',
        name='sim_bridge',
        output='screen',
        parameters=[{'hil_mode': True}],
    )

    # ── micro-ROS Agent (bridges serial ↔ DDS) ─────────────────────
    # Runs: micro_ros_agent serial -D <dev> -b <baud> -v 4
    import pathlib
    _agent_install = pathlib.Path.home() / 'micro_ros_agent_ws' / 'install'
    _agent_bin = (_agent_install / 'micro_ros_agent' / 'lib' / 'micro_ros_agent' / 'micro_ros_agent').as_posix()
    _agent_lib = ':'.join([
        (_agent_install / 'micro_ros_agent' / 'lib').as_posix(),
        (_agent_install / 'micro_ros_msgs' / 'lib').as_posix(),
    ])
    _existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    _agent_env = {'LD_LIBRARY_PATH': f'{_agent_lib}:{_existing_ld}'} if _existing_ld else {'LD_LIBRARY_PATH': _agent_lib}
    micro_ros_agent = ExecuteProcess(
        cmd=[_agent_bin, 'serial', '-D', serial_dev, '-b', serial_baud, '-v', '4'],
        additional_env=_agent_env,
        output='screen',
        name='micro_ros_agent',
    )

    # ── Control  (optional) ────────────────────────────────────────────────────
    basic_motion = Node(
        package='uv_control',
        executable='basic_motion',
        name='basic_motion',
        output='screen',
        condition=IfCondition(enable_motion),
    )

    # ── Perception (optional) ──────────────────────────────────────
    vision = Node(
        package='uv_perception',
        executable='vision',
        name='vision',
        output='screen',
        parameters=[{'publish_annotated': publish_annotated}],
        condition=IfCondition(enable_ai),
    )

    position = Node(
        package='uv_perception',
        executable='position',
        name='position',
        output='screen',
        condition=IfCondition(enable_ai),
    )

    # ── Navigation (optional) ──────────────────────────────────────
    navigator = Node(
        package='uv_nav',
        executable='navigator',
        name='navigator',
        output='screen',
        condition=IfCondition(enable_nav),
    )

    # ── Task runner (optional) ─────────────────────────────────────
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
        declare_enable_motion,
        declare_scenario,
        declare_serial_dev,
        declare_serial_baud,
        declare_publish_annotated,
        LogInfo(msg=['HIL simulation data dir: ', simulation_data_dir]),
        LogInfo(msg=['micro-ROS serial device: ', serial_dev, ' @ ', serial_baud]),
        stonefish_sim,
        sim_bridge,
        micro_ros_agent,
        basic_motion,
        vision,
        position,
        navigator,
        task_runner,
    ])
