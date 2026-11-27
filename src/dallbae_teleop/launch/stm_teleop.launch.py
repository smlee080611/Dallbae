import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup_nodes(context):
  urdf_pkg = LaunchConfiguration("urdf_pkg").perform(context)
  urdf_rel_path = LaunchConfiguration("urdf_file").perform(context)
  ctrl_rel_path = LaunchConfiguration("controllers_file").perform(context)

  urdf_path = os.path.join(get_package_share_directory(urdf_pkg), urdf_rel_path)
  controllers_path = os.path.join(get_package_share_directory(urdf_pkg), ctrl_rel_path)

  robot_description = {
      "robot_description": Command([
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          urdf_path,
      ])
  }

  robot_state_pub = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      output="screen",
      parameters=[robot_description],
  )

  control_node = Node(
      package="controller_manager",
      executable="ros2_control_node",
      output="screen",
      parameters=[robot_description, controllers_path],
  )

  joint_state_broadcaster_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
      output="screen",
  )

  diff_drive_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
      output="screen",
  )

  delay_diff_drive_after_jsb = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=joint_state_broadcaster_spawner,
          on_exit=[diff_drive_spawner],
      )
  )

  return [
      robot_state_pub,
      control_node,
      joint_state_broadcaster_spawner,
      delay_diff_drive_after_jsb,
  ]


def generate_launch_description():
  return LaunchDescription([
      DeclareLaunchArgument(
          "urdf_pkg",
          default_value="for_urdf_description",
          description="URDF/Xacro and controller YAML package name",
      ),
      DeclareLaunchArgument(
          "urdf_file",
          default_value="urdf/robot_copy.xacro",
          description="Relative path to Xacro/URDF inside urdf_pkg",
      ),
      DeclareLaunchArgument(
          "controllers_file",
          default_value="config/controller.yaml",
          description="Relative path to controller YAML inside urdf_pkg",
      ),
      OpaqueFunction(function=_setup_nodes),
  ])