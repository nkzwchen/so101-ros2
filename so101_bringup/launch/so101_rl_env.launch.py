# RL 训练用 launch（F1 方案）
#
# 与 so101_teleoperate.launch.py 的关键区别：
#   ★ **不启动 LeaderTeleopComponent**
#
# 为什么必须去掉它：
#   LeaderTeleopComponent 订阅 /leader/joint_states 并发布
#   /follower/arm_controller/joint_trajectory。而 SO101Env 也要发布同一个话题。
#   两者同时存在会形成「两个发布者抢同一话题」。
#   去掉它之后，**SO101Env 成为 follower 的唯一控制源**，模式切换完全在
#   env 内部用 Python 处理，不涉及任何 ROS2 话题仲裁。
#
# 本 launch 启动的东西：
#   ┌─ follower 侧 ────────────────────────────────────────┐
#   │ robot_state_publisher（URDF → TF）                    │
#   │ follower_ros2_node（串口桥，发布 joint_states_raw）    │
#   │ controller_manager（ros2_control）                     │
#   │ joint_state_broadcaster / arm_controller /            │
#   │ gripper_controller                                     │
#   └───────────────────────────────────────────────────────┘
#   ┌─ leader 侧 ──────────────────────────────────────────┐
#   │ leader_ros2_node（L1 改造版：可被 /leader/arm_command │
#   │                   驱动；有 /leader/set_torque）        │
#   └───────────────────────────────────────────────────────┘
#   ┌─ 相机（可选）────────────────────────────────────────┐
#   └───────────────────────────────────────────────────────┘
#
# 用法：
#   ros2 launch so101_bringup so101_rl_env.launch.py
#   ros2 launch so101_bringup so101_rl_env.launch.py use_cameras:=false

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    args = []
    actions = []

    bringup_pkg = get_package_share_directory('so101_bringup')
    description_pkg = get_package_share_directory('so101_description')

    # ---- 参数 ----
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_pkg, 'urdf', 'so101_new_calib.urdf.xacro'),
    )
    args.append(model_arg)

    use_cameras_arg = DeclareLaunchArgument(
        'use_cameras', default_value='true',
        description='是否启动 usb_cam 相机节点（没接相机时设 false）',
    )
    args.append(use_cameras_arg)

    use_leader_arg = DeclareLaunchArgument(
        'use_leader', default_value='true',
        description='是否启动 leader 臂桥（人类接管 + 手感反馈需要）',
    )
    args.append(use_leader_arg)

    display_arg = DeclareLaunchArgument(
        'display', default_value='false', description='是否启动 RViz',
    )
    args.append(display_arg)

    display_config_arg = DeclareLaunchArgument(
        'display_config',
        default_value=os.path.join(
            get_package_share_directory('so101_teleop'),
            'rviz', 'teleop_with_camera.rviz',
        ),
    )
    args.append(display_config_arg)

    model = LaunchConfiguration('model')
    use_cameras = LaunchConfiguration('use_cameras')
    use_leader = LaunchConfiguration('use_leader')
    display = LaunchConfiguration('display')
    display_config = LaunchConfiguration('display_config')

    actions.append(LogInfo(
        msg='[RL ENV LAUNCH] 启动 SO101 训练环境（不含 LeaderTeleopComponent）'
    ))

    # ---- follower（含 ros2_control 与控制器）----
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'follower.launch.py')
        ),
        launch_arguments={'model': model, 'use_sim_time': 'false'}.items(),
    )
    # 延时启动，等 URDF/TF 就绪
    actions.append(TimerAction(period=2.0, actions=[follower_launch]))

    # ---- leader（L1 改造版桥，可被驱动 + set_torque）----
    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'leader.launch.py')
        ),
        launch_arguments={'model': model, 'use_sim_time': 'false'}.items(),
        condition=IfCondition(EqualsSubstitution(use_leader, 'true')),
    )
    actions.append(TimerAction(period=2.0, actions=[leader_launch]))

    # ---- 相机（可选）----
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'include', 'camera.launch.py')
        ),
        condition=IfCondition(EqualsSubstitution(use_cameras, 'true')),
    )
    actions.append(TimerAction(period=4.0, actions=[cameras_launch]))

    actions.append(LogInfo(
        msg='[RL ENV LAUNCH] ⚠️ 未启动 LeaderTeleopComponent —— SO101Env 是 follower 的唯一控制源',
        condition=IfCondition(EqualsSubstitution(use_leader, 'true')),
    ))

    # ---- RViz（可选）----
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={'joint_states_gui': 'false', 'display_config': display_config}.items(),
        condition=IfCondition(EqualsSubstitution(display, 'true')),
    )
    actions.append(TimerAction(period=6.0, actions=[display_launch]))

    return LaunchDescription(args + actions)
