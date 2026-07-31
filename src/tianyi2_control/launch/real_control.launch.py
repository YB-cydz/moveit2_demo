#!/usr/bin/env python3
"""
天轶2.0 真实硬件控制 Launch

与 demo.launch.py 的区别:
  - /joint_states 由 joint_state_publisher 从真实硬件状态发布（替代 FakeSystem）
  - 不启动 joint_state_broadcaster
  - 附带 bridge_node（MoveIt 轨迹 → /arm/cmd_pos）

启动:
  ros2 launch tianyi2_control real_control.launch.py
  ros2 launch tianyi2_control real_control.launch.py use_rviz:=false
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "tianyi2_complete", package_name="tianyi2_moveit"
    ).to_moveit_configs()

    moveit_launch = os.path.join(
        get_package_share_directory("tianyi2_moveit"), "launch"
    )

    ld = LaunchDescription()

    # ── 公共参数 ──
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))

    # =========================================================================
    # ① static_virtual_joint_tfs（如有虚拟关节）
    # =========================================================================
    virtual_joints_launch = os.path.join(
        moveit_launch, "static_virtual_joint_tfs.launch.py"
    )
    if os.path.exists(virtual_joints_launch):
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(virtual_joints_launch),
            )
        )

    # =========================================================================
    # ② robot_state_publisher（读 /joint_states → 广播 TF）
    # =========================================================================
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_launch, "rsp.launch.py")
            ),
        )
    )

    # =========================================================================
    # ③ move_group（MoveIt 核心：运动学、规划、执行）
    # =========================================================================
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_launch, "move_group.launch.py")
            ),
        )
    )

    # =========================================================================
    # ④ RViz（可选）
    # =========================================================================
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_launch, "moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # =========================================================================
    # ⑤ ros2_control_node（FakeSystem — 提供控制器所需的 state/command 接口）
    #    注意: 不启动 joint_state_broadcaster，/joint_states 由 joint_state_publisher 提供
    # =========================================================================
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config" / "ros2_controllers.yaml"),
            ],
            output="screen",
        )
    )

    # =========================================================================
    # ⑥ 控制器 spawner（仅手臂，不含 joint_state_broadcaster）
    # =========================================================================
    for controller in ["L_Arm_Group_controller", "R_Arm_Group_controller"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # =========================================================================
    # ⑦ joint_state_publisher（真实硬件 → /joint_states）
    # =========================================================================
    ld.add_action(
        Node(
            package="tianyi2_control",
            executable="joint_state_publisher",
            output="screen",
        )
    )

    # =========================================================================
    # ⑧ bridge_node — 单独启动:
    #     ros2 run tianyi2_control bridge_node
    # =========================================================================

    return ld
