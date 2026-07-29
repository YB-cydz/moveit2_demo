"""
真机 MoveIt 2 启动文件

前提：机器人已运行并提供：
  - 完整的 /joint_states（所有关节）
  - 完整的 /tf（已通过 robot_state_publisher 加载 URDF）
  - 关节控制话题（如 /arm/ctrl 等）

本 launch 仅启动：
  - move_group（规划 + IK + 碰撞检测）
  - RViz（可视化 + MotionPlanning 面板）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 加载 MoveIt 配置（URDF、SRDF、kinematics、joint_limits 等）
    moveit_config = (
        MoveItConfigsBuilder("tianyi2_complete", package_name="demo")
        .to_moveit_configs()
    )

    # ---- RViz 参数 ------------------------------------------------------------
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=str(
            moveit_config.package_path / "config" / "moveit.rviz"
        ),
        description="RViz 配置文件路径",
    )

    # ---- move_group 节点 -------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # ---- RViz 节点 ------------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        rviz_config_arg,
        move_group_node,
        rviz_node,
    ])
