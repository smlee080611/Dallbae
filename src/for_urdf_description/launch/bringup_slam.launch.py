import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('for_urdf_description')  # 본인 패키지명으로 수정
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')  # 본인 Xacro 경로로 수정
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_online_async.yaml')
    default_world = os.path.join(pkg_share, 'world', 'slam_arena.world')
    rviz_config_file = os.path.join(pkg_share, 'config', 'slam_view.rviz')
    
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='new world'
    )


    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='dallbae',
        description='my robot'
    )
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
        }],
    )

    # 기존 Gazebo + 컨트롤러 스폰 launch가 있다면 함께 포함
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
        }.items()

    )

    spawn_robot = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-entity', LaunchConfiguration('robot_name'),
                    '-topic', 'robot_description',
                    '-x','0.0',
                    '-y','0.0',
                    '-z','0.1',
                ],
            )
        ]
    )
    spawn_jsb = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )
    spawn_diff_drive = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--ros-args',
            '-r', '/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel',
        ],
        output='screen',
    )
    delayed_controllers = TimerAction(
        period=4.5,
        actions=[spawn_jsb, spawn_diff_drive],
    )

    # slam_toolbox: YAML 로드 후 핵심 파라미터를 명시적으로 덮어써 충돌 방지
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': True,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
            },
        ],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )


    # /clock 및 /tf가 올라온 뒤 slam_toolbox가 붙도록 살짝 지연 실행
    delayed_slam = TimerAction(
        period=2.0,
        actions=[slam_toolbox_node,rviz_node,],
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        robot_state_pub,
        gazebo_launch,
        spawn_robot,
        delayed_controllers,
        delayed_slam,
    ])