import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('for_urdf_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_world = os.path.join(pkg_share, 'world', 'slam_arena.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    default_map = os.path.expanduser('~/slam_arena_map.yaml')
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'nav2_default_view.rviz')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to Gazebo world file',
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to Nav2 parameters file',
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='dallbae',
        description='Spawned robot entity name in Gazebo',
    )

    robot_description_xml = xacro.process_file(xacro_file).toxml()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
        }],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
        }.items(),
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-entity', LaunchConfiguration('robot_name'),
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',
                ],
            )
        ],
    )

    # ros2_control 컨트롤러 스폰 및 /cmd_vel 리매핑
    spawn_jsb = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        remappings=[('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel')],
        output='screen',
    )

    delayed_controllers = TimerAction(
        period=4.5,
        actions=[spawn_jsb, spawn_diff_drive],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'],
    )
    

    rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': True}],
    )

    delayed_nav2_and_rviz = TimerAction(
        period=6.5,
        actions=[nav2_bringup, rviz_node],
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        cmd_vel_relay,
        params_file_arg,
        robot_name_arg,
        robot_state_pub,
        gazebo_launch,
        spawn_robot,
        delayed_controllers,
        delayed_nav2_and_rviz,
    ])