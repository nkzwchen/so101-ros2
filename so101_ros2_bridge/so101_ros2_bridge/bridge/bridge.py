#!/usr/bin/env python3

# Copyright 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import math
import threading
import time
from abc import ABC, abstractmethod
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

# Ensure the conda site-packages directory is in the system path
from so101_ros2_bridge.utils.core import ensure_conda_site_packages_from_env

ensure_conda_site_packages_from_env()

from lerobot.robots.so_follower import SO101Follower
from lerobot.teleoperators.so_leader import SO101Leader

from so101_ros2_bridge import CALIBRATION_BASE_DIR  # defined in __init__.py
from so101_ros2_bridge.bridge.registry import ROBOT_FACTORY_REGISTRY


class SO101ROS2Bridge(Node, ABC):
    JOINT_NAMES = [
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll',
        'gripper',
    ]

    def __init__(self, node_name='so101_ros2_bridge'):
        super().__init__(node_name)
        params = self.read_parameters()
        self.use_degrees = params['use_degrees']

        # Initialize watchdog at the background
        self._is_alive = True
        self._watchdog_interval = 0.01  # 100 hz whatchdog
        self._timeout = 5.0
        self._alive_thread = threading.Thread(target=self._alive, daemon=True)

        # Pre-allocate the message and its internal lists
        self._joint_state_msg = JointState()
        self._joint_state_msg.name = self.JOINT_NAMES
        self._positions = [0.0] * len(self.JOINT_NAMES)
        self._velocities = [0.0] * len(self.JOINT_NAMES)

        rate = params.get('publish_rate', 30.0)

        self.joint_pub = self.create_publisher(JointState, 'joint_states_raw', 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_joint_states)

        # Register the robot type in the factory registry
        if not ROBOT_FACTORY_REGISTRY:
            raise RuntimeError(
                'ROBOT_FACTORY_REGISTRY is empty. Ensure robot types are registered.'
            )

        self.declare_parameter('type', 'follower')
        robot_type = self.get_parameter('type').get_parameter_value().string_value
        self.get_logger().info(f'Initializing SO101 ROS2 Bridge for robot type: {robot_type}')
        factory_fn = ROBOT_FACTORY_REGISTRY.get(robot_type)
        if factory_fn is None:
            raise ValueError(f"Robot type '{robot_type}' not registered.")

        # Initialize robot
        self.robot = factory_fn(params)

        try:
            self.robot.connect(calibrate=False)
        except Exception as e:
            raise RuntimeError(f'Failed to connect to SO101 robot: {e}') from e

        self.last_positions = None
        self.last_time = self.get_clock().now()

        # Start alive
        self._alive_thread.start()

    @abstractmethod
    def read_parameters(self) -> dict:
        """Reads parameters from the ROS2 parameter server."""

    @abstractmethod
    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """

    # ---------------------------------------------------------------- 力矩开关
    # ⚠️ 放在基类里：FollowerBridge 和 LeaderBridge 是**兄弟类**，不是父子关系，
    #   放子类里另一边就继承不到（2026-09-29 改的时候踩了一次）。
    def _read_torque_off_motors(self):
        """返回当前**确实无力矩**的电机名列表；读不到返回 None。

        `Torque_Enable` 是可读寄存器（地址 40，长度 1），所以这里能拿到权威状态 ——
        比在脑子里记一个 `self._torque_on` 布尔量可靠（舵机状态可能被别的进程
        或 `disable_torque_on_disconnect` 改掉）。
        """
        try:
            te = self.robot.bus.sync_read('Torque_Enable')
            if not te:
                return None
            return [m for m in self.JOINT_NAMES if int(te.get(m, 1)) == 0]
        except Exception as e:                              # noqa: BLE001
            self.get_logger().warn(
                f'读 Torque_Enable 失败：{type(e).__name__}: {e}')
            return None

    def _sync_goal_to_present(self, who: str, motors=None):
        """把**指定电机**（默认全部）的 `Goal_Position` 写成当前实测位置。

        ★★★ 只能在电机**当前无力矩**时调用 ★★★
            对已经在保持力矩的电机写 `Goal_Position = 当前实测位置`，会**抹掉它
            用来顶住重力的稳态偏置**：本舵机是比例位置环且 `I_Coefficient = 0`，
            正是靠"指令值比实际位置高出一个静差量"来顶重力的。把目标抹平 = 撤掉
            顶重力的力 → 机械臂立刻下垂。
            2026-09-29 实测：主臂因此**下垂 4.0°**（用户看到的现象）。

        为什么要做（同日的另一起事故，主臂突然上跳）：
            Feetech 舵机 `Torque_Enable = 1` 时会**立刻冲向 `Goal_Position` 寄存器里
            的旧值**，而这个寄存器在力矩关闭期间不会被更新。于是"开力矩"这个动作
            本身就会让机械臂猛冲向上一次运行留下的目标。
            现场表现：主臂**突然跳起来** → 脚本第一条指令是"开力矩之前"测到的
            下垂位置 → 又把它拽回去 → 第一帧就落后插值参考 34.8° → 跟踪中止。
            同期日志里**没有任何**写失败或过热报错 —— 因为舵机根本没出错。

            影响面不止归位脚本：`env.py` 每次"人类接管 ↔ 回到 policy"切换都会
            开/关主臂力矩，同一个坑会在每次接管结束时重演。

        顺带解决第三个坑：限幅基准 `_last_cmd` 也是跨次运行残留的。
        它若停在"上一次运行的旧目标"上，下一条命令会被它限幅到错误的位置。
        所以被同步的那些电机的 `_last_cmd` 也要一并重置为真实位置。

        读写都用 sync_read/sync_write 的**同一个归一化约定**，不手写单位换算，
        避免"度 vs 归一化"这类静默错误。
        """
        present = self.robot.bus.sync_read('Present_Position')
        if not present:
            raise RuntimeError('sync_read(Present_Position) 返回空')
        if motors is None:
            motors = list(self.JOINT_NAMES)
        goal = {m: present[m] for m in motors if m in present}
        if not goal:
            return present
        self.robot.bus.sync_write('Goal_Position', goal)
        if getattr(self, '_last_cmd', None) is None:
            self._last_cmd = dict(goal)
        else:
            self._last_cmd.update(goal)
        self.get_logger().info(
            f'{who} Goal_Position 已同步为当前实测位置：{sorted(goal)}')
        return present

    def _prepare_torque_enable(self, who: str):
        """开力矩前的准备：**只对当前松着的电机**同步 Goal_Position。

        读不到 `Torque_Enable` 时保守地**照旧同步全部** —— 两害相权：
          · 该同步却没同步 → 松着的臂会猛冲旧目标（十几度，危险）
          · 不该同步却同步了 → 有力矩的臂被抹掉重力偏置，下垂几度（难受，但不危险）
        """
        off = self._read_torque_off_motors()
        if off is None:
            self.get_logger().warn(
                f'{who} 取不到力矩状态，为安全起见仍同步全部 Goal_Position')
            self._sync_goal_to_present(who)
        elif off:
            self._sync_goal_to_present(who, off)
        else:
            self.get_logger().info(
                f'{who} 已全部带力矩，跳过 Goal_Position 同步（避免抹掉重力偏置）')

    # In your publish_joint_states method:
    def publish_joint_states(self):
        try:
            current_time = self.get_clock().now()
            obs = self.get_joints_states()

            # Update the pre-allocated lists instead of creating new ones
            for i, joint in enumerate(self.JOINT_NAMES):
                if joint == 'gripper':
                    pos = ((obs.get(f'{joint}.pos', 0.0)) / 100.0) * math.pi
                else:
                    if self.use_degrees:
                        pos = math.radians(obs.get(f'{joint}.pos', 0.0))
                    else:
                        # unormalized range [-100, 100] to radians
                        pos = (obs.get(f'{joint}.pos', 0.0) / 100.0) * math.pi
                self._positions[i] = pos

            if self.last_positions is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt > 1e-6:  # More robust check against zero
                    for i in range(len(self.JOINT_NAMES)):
                        self._velocities[i] = (self._positions[i] - self.last_positions[i]) / dt

            # Update and publish the pre-allocated message
            self._joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self._joint_state_msg.position = self._positions
            self._joint_state_msg.velocity = self._velocities
            self.joint_pub.publish(self._joint_state_msg)

            # Store a copy for the next velocity calculation
            self.last_positions = list(self._positions)  # or self._positions[:]
            self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f'Failed to publish joint states: {e}')

    def _alive(self):
        """
        Watchdog thread running at a fixed rate. Monitors rclpy.ok() and ROS time.
        Triggers shutdown if ROS crashes or time stops progressing.
        """
        interval = self._watchdog_interval
        next_tick = time.monotonic()

        while self._is_alive:
            # Check ROS system state
            if not rclpy.ok():
                self.get_logger().error(
                    'ROS shutdown detected (rclpy.ok() == False). Triggering shutdown.'
                )
                self.shutdown_hook()
                rclpy.try_shutdown()
                break

            # Check time progress
            now = self.get_clock().now()
            elapsed = (now - self.last_time).nanoseconds / 1e9
            if elapsed > self._timeout:
                self.get_logger().error(
                    f'ROS time not updating for {elapsed:.1f}s — triggering shutdown.'
                )
                self.shutdown_hook()
                rclpy.try_shutdown()
                break

            # Sleep until next tick
            next_tick += interval
            sleep_time = max(0.0, next_tick - time.monotonic())
            time.sleep(sleep_time)

    def shutdown_hook(self):
        if not self._is_alive:
            return  # Prevent double cleanup

        self._is_alive = False
        try:
            self.get_logger().info('Shutting down, disconnecting robot...')
            if hasattr(self, 'robot'):
                (
                    self.robot.disconnect()
                    if self.robot.is_connected
                    else self.get_logger().warn('Robot is already disconnected')
                )
        except Exception as e:
            self.get_logger().warn(f'Exception during shutdown: {e}')

    def radians_to_normalized(self, joint_name: str, rad: float) -> float:
        """
        converts a command in radians from MoveIt to the format expected by the SO101 API.
        """
        if joint_name != 'gripper' and self.use_degrees:
            return math.degrees(rad)
        else:
            # Convert radians to normalized range [-100, 100] for other joints and gripper to normalized [0,100]
            normalized = (rad / math.pi) * 100.0
            return normalized


class FollowerBridge(SO101ROS2Bridge):
    def __init__(self):
        super().__init__('so101_follower_ros2_bridge')

        # Type hint of the robot
        if not isinstance(self.robot, SO101Follower):
            raise TypeError(f'Expected robot type SO101Follower, got {type(self.robot).__name__}')

        # QoS for joint command subscriber
        qos_joint_cmds = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # only latest command matters
            reliability=QoSReliabilityPolicy.RELIABLE,  # ensure delivery
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscribe to commands from the ros2_control hardware interface bridge
        self.create_subscription(
            Float64MultiArray,
            'joint_commands',  # This topic should match the publisher in the C++ bridge
            self.command_callback,
            qos_joint_cmds,
        )

        # 力矩开关 service：/follower/set_torque (std_srvs/SetBool)
        # 用途：
        #   True  → follower 保持位置（正常被 policy / 控制器驱动）
        #   False → follower 变松，可用手自由推动
        #           （调试时常用；也用于需要手动摆位的场景）
        # 与 LeaderBridge 的 set_torque 对称，便于 env 统一管理。
        self._torque_srv = self.create_service(
            SetBool, 'set_torque', self._set_torque_callback
        )

        # ★★ 限幅（2026-09 恢复，改为「不读电机」的实现）★★
        #   单位同 radians_to_normalized 的输出：非 gripper 是度，gripper 是 [-100,100]。
        #   约定与 LeaderBridge 完全一致：以「上一条命令」为基准，**不读电机当前值**。
        #
        #   踩坑经过：原先靠 lerobot 的 ensure_safe_goal_position 限幅，而它会在
        #   **每个控制周期读一次全部舵机位置**（num_retry=5）。相机占满 USB 总线时
        #   这次读会拖慢指令流；当时为做诊断把 max_relative_target 关成 0.0，
        #   于是从臂失去了对「命令流不规整」的免疫：
        #     归位脚本一旦因节拍落后而**爆发式补发**，从臂就原样吃下并发抖，
        #     而 leader 桥有 20°/步限幅把这些爆发抹平了 —— 表现为
        #     **「主臂动得很好、从臂动得很差」**。
        #   现在改为不读电机的实现：既保住安全网，又不付串口往返的代价。
        _mrt = self.get_parameter('max_relative_target').get_parameter_value().double_value
        self.max_relative_target = _mrt if _mrt > 0.0 else None

        self.get_logger().info(
            'SO101 Follower ROS2 Bridge initialized. | service: set_torque '
            f'| max_relative_target={self.max_relative_target}'
        )

    def _set_torque_callback(self, request, response):
        """ROS2 service 回调：/follower/set_torque。

        True  → enable_torque：follower 保持位置
        False → disable_torque：follower 变松，可用手推动（调试用）

        注意：disable 之后若控制器仍在发命令，舵机不会响应（无力矩）。
        恢复 True 时会瞬间重新跟随 `Goal_Position` —— 所以**开力矩前**会先把
        `Goal_Position` 同步成当前实测位置，见 `_sync_goal_to_present()`。
        """
        ok = self.set_follower_torque(bool(request.data))
        response.success = ok
        response.message = f'follower torque -> {"ON" if request.data else "OFF"}'
        return response

    def set_follower_torque(self, enabled: bool) -> bool:
        """开/关 follower 全部电机的力矩。返回是否成功。"""
        try:
            if enabled:
                # ★ 只对当前松着的电机同步 Goal_Position，然后才开力矩
                self._prepare_torque_enable('follower')
                self.robot.bus.enable_torque()
            else:
                self.robot.bus.disable_torque()
            self.get_logger().info(f'Follower torque -> {"ON" if enabled else "OFF"}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to set follower torque: {type(e).__name__}: {e}')
            return False

    def read_parameters(self) -> dict:
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('id', 'Tzili')
        self.declare_parameter('calibration_dir', str(CALIBRATION_BASE_DIR))
        self.declare_parameter('use_degrees', True)
        # 注意：必须声明为 double（默认 0.0），不能用 int 默认值 0。
        # 原因：lerobot 0.5.1 的 ensure_safe_goal_position 只接受 float/dict，
        # 传 int 会抛 TypeError。若声明成 int，ROS2 又会拒绝 double 类型的 yaml 值。
        # 约定：0.0 或 <0 表示不限制；>0 表示单步最大变化量（度）。
        self.declare_parameter('max_relative_target', 0.0)
        self.declare_parameter('disable_torque_on_disconnect', True)
        self.declare_parameter('publish_rate', 30.0)

        max_relative_target = (
            self.get_parameter('max_relative_target').get_parameter_value().double_value
        )
        max_relative_target = max_relative_target if max_relative_target > 0.0 else None

        return {
            'port': self.get_parameter('port').get_parameter_value().string_value,
            'id': self.get_parameter('id').get_parameter_value().string_value,
            'calibration_dir': Path(
                self.get_parameter('calibration_dir').get_parameter_value().string_value
            ),
            'use_degrees': self.get_parameter('use_degrees').get_parameter_value().bool_value,
            'max_relative_target': max_relative_target,
            'disable_torque_on_disconnect': (
                self.get_parameter('disable_torque_on_disconnect').get_parameter_value().bool_value
            ),
            'publish_rate': self.get_parameter('publish_rate').get_parameter_value().double_value,
        }

    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """
        return self.robot.get_observation()

    def command_callback(self, msg: Float64MultiArray):
        """
        Receives joint command goals in radians and sends them to the robot.
        """
        # ★★ NaN 防线（2026-09 修，务必保留）★★
        #   C++ 硬件桥在「硬件激活 → 收到第一帧 joint_states_raw」之间会发出 NaN：
        #   so101_hardware_bridge.cpp 的 write() 兜底是「命令是 NaN 就退回用位置」，
        #   但 hw_positions_ 此时**也是 NaN**（on_init 里三个缓冲区都初始化为 NaN）。
        #
        #   若让这个 NaN 进入下面的 _last_cmd 限幅，会**永久毒化**限幅基准：
        #       prev = NaN → diff = NaN − NaN = NaN
        #       → 两个比较都是 False → new_v = NaN + NaN = NaN
        #       → _last_cmd 又被写成 NaN → 之后每一条命令都是 NaN
        #   后果：send_action 每帧抛 ValueError，手臂**一步都不动且永不自愈**
        #   （实测错误计数一路涨到 28350，从臂完全静止）。
        #
        #   对策：非有限值的命令**直接忽略** —— 不发送，也绝不更新 _last_cmd。
        #   这同时也消掉了原先"启动时报一次 cannot convert float NaN to integer"的噪音。
        if len(msg.data) != len(self.JOINT_NAMES) or not all(
            math.isfinite(v) for v in msg.data
        ):
            self._bad_cmd_count = getattr(self, '_bad_cmd_count', 0) + 1
            if self._bad_cmd_count <= 3 or self._bad_cmd_count % 500 == 0:
                self.get_logger().warn(
                    f'忽略含 NaN/Inf 或长度不符的命令 (#{self._bad_cmd_count})'
                )
            return

        # 先转成 lerobot 期望的单位（follower 侧 send_action 需要 ".pos" 后缀键），
        # 同时保留一份「纯 motor 名」的副本用于限幅。
        target_positions = {}
        raw_values = {}
        for i, joint in enumerate(self.JOINT_NAMES):
            # Convert the incoming radian command to the format the robot expects (degrees or normalized)
            v = self.radians_to_normalized(joint, msg.data[i])
            raw_values[joint] = v
            target_positions[f'{joint}.pos'] = v

        # ---- 安全限幅：以「上一条命令」为基准（不读电机当前值）----
        # 实现与 LeaderBridge.command_callback 逐字一致，保持两条命令路径的鲁棒性对称。
        # 作用：命令流即使出现爆发式补发（脚本节拍落后、某帧迟到），
        #       单步变化也不会超过 max_relative_target，从臂不会被"甩"。
        if self.max_relative_target is not None:
            if getattr(self, '_last_cmd', None) is None:
                # 首条命令不做限幅，否则会从 0 开始把臂拉到 0 附近
                self._last_cmd = dict(raw_values)
            else:
                cap = self.max_relative_target
                for joint in self.JOINT_NAMES:
                    prev = self._last_cmd.get(joint, raw_values[joint])
                    # 双保险：万一 _last_cmd 被污染成非有限值，自愈为当前值
                    if not math.isfinite(prev):
                        self._last_cmd[joint] = raw_values[joint]
                        continue
                    diff = raw_values[joint] - prev
                    if diff > cap:
                        diff = cap
                    elif diff < -cap:
                        diff = -cap
                    new_v = prev + diff
                    target_positions[f'{joint}.pos'] = new_v
                    raw_values[joint] = new_v
                self._last_cmd = dict(raw_values)

        try:
            self.robot.send_action(target_positions)
        except Exception as e:
            # 原代码只打 str(e)，丢了类型和 traceback，导致 "Failed to send commands to robot: 10" 无法定位。
            # 这里补全信息，并做限流（每 50 次打一次完整 traceback）以免刷屏。
            self._cmd_err_count = getattr(self, '_cmd_err_count', 0) + 1
            if self._cmd_err_count <= 3 or self._cmd_err_count % 50 == 0:
                import traceback as _tb
                detail = _tb.format_exc()
                self.get_logger().error(
                    f'Failed to send commands to robot '
                    f'[#{self._cmd_err_count}] {type(e).__name__}: {e}\n'
                    f'  target_positions = {target_positions}\n'
                    f'  traceback:\n{detail}'
                )


class LeaderBridge(SO101ROS2Bridge):
    """Leader 臂桥。

    与 FollowerBridge 的关键区别：
      · FollowerBridge 是「执行器」：订阅命令 → send_action 驱动 follower 电机
      · LeaderBridge  默认是「传感器」：只发布 joint_states_raw（人在动它）

    ★ L1 新增能力：让 leader 也能被电机驱动（跟随 follower），用于 RL 训练的
      「模式 A（policy 控制，leader 跟随 follower）」。
      实现方式：订阅独立话题 arm_command，直接写 leader 的 Feetech 总线。

    为什么用独立话题（/leader/arm_command）而不是复用 /leader/joint_commands：
      C++ 的 So101HardwareBridge 已经在发布 /leader/joint_commands（它把 ros2_control
      的写命令转发出来，当前无人订阅）。若我们也订阅同一话题，会形成
      「我们发的命令被 C++ 桥转发回来」的反馈环。
      因此必须用独立话题，避免撞名。

    为什么直接操作 self.robot.bus 而不调 send_feedback()：
      lerobot 的 SOLeader.send_feedback() 目前是 `raise NotImplementedError`
      （其 feedback_features 返回空 dict，即"本设备不支持 feedback"）。
      我们直接写总线（等价于 FollowerBridge.send_action 的做法），
      避免修改 lerobot 第三方源码（升级时会冲突）。
    """

    def __init__(self):
        super().__init__('so101_leader_ros2_bridge')

        # Type hint of the robot
        if not isinstance(self.robot, SO101Leader):
            raise TypeError(f'Expected robot type SO101Leader, got {type(self.robot).__name__}')

        # 基类的 read_parameters() 只保存了 use_degrees / publish_rate，
        # 这里把 L1 需要的两个参数显式取出并保存（参数本身已在 read_parameters 里声明）。
        #
        # command_topic 用全限定名（强制以 / 开头），避免受命名空间影响。
        _ct = self.get_parameter('command_topic').get_parameter_value().string_value
        self.command_topic = _ct if _ct.startswith('/') else f'/leader/{_ct}'
        _mrt = self.get_parameter('max_relative_target').get_parameter_value().double_value
        self.max_relative_target = _mrt if _mrt > 0.0 else None

        # QoS：只要最新命令（与 FollowerBridge 一致）
        qos_joint_cmds = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # only latest command matters
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # 订阅「驱动 leader」的命令（由 SO101Env 发布）
        self.create_subscription(
            Float64MultiArray,
            self.command_topic,
            self.command_callback,
            qos_joint_cmds,
        )

        # 力矩开关 service：模式 A/B 切换时由 SO101Env 调用
        #   True  → leader 跟随 follower（力矩 ON）
        #   False → 人类自由推动（力矩 OFF，接管用）
        self._torque_srv = self.create_service(
            SetBool, 'set_torque', self._set_torque_callback
        )

        self.get_logger().info(
            f'SO101 Leader ROS2 Bridge initialized. '
            f'Command topic: {self.command_topic} '
            f'(max_relative_target={self.max_relative_target}) '
            f'| service: set_torque'
        )

    def read_parameters(self) -> dict:
        """⚠️ 本方法在 super().__init__() **内部**被调用（见基类第 66 行），
        因此所有参数必须在这里声明 —— 放在子类 __init__ 里会太晚。"""
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('id', 'Gili')
        self.declare_parameter('calibration_dir', str(CALIBRATION_BASE_DIR))
        self.declare_parameter('use_degrees', True)
        self.declare_parameter('publish_rate', 30.0)
        # L1 新增：leader 驱动能力
        # max_relative_target 单位=度，0 表示不限；防止 follower 位置突变时 leader 猛冲
        self.declare_parameter('max_relative_target', 20.0)
        # 独立命令话题（实际为 /leader/arm_command）
        self.declare_parameter('command_topic', 'arm_command')

        return {
            'port': self.get_parameter('port').get_parameter_value().string_value,
            'id': self.get_parameter('id').get_parameter_value().string_value,
            'calibration_dir': Path(
                self.get_parameter('calibration_dir').get_parameter_value().string_value
            ),
            'use_degrees': self.get_parameter('use_degrees').get_parameter_value().bool_value,
            'publish_rate': self.get_parameter('publish_rate').get_parameter_value().double_value,
            'max_relative_target': (
                self.get_parameter('max_relative_target').get_parameter_value().double_value
            ),
            'command_topic': self.get_parameter('command_topic').get_parameter_value().string_value,
        }

    def get_joints_states(self) -> dict:
        """
        Returns the current joint states as a dictionary.
        This method should be implemented by subclasses to return the robot's joint states.
        """
        return self.robot.get_action()

    # ------------------------------------------------------------------ L1 新增
    def command_callback(self, msg: Float64MultiArray):
        """接收弧度制的关节目标，驱动 leader 电机（让 leader 跟随 follower）。

        与 FollowerBridge.command_callback 的逻辑保持一致：
          · 入参单位是弧度（与 /follower/joint_states 同单位）
          · 用 radians_to_normalized 转成 lerobot 期望的度/归一化值
          · 直接写总线（lerobot 的 send_feedback 是 stub）
        """
        if len(msg.data) != len(self.JOINT_NAMES) or not all(
            math.isfinite(v) for v in msg.data
        ):
            # ★ NaN 防线，与 FollowerBridge.command_callback 同一理由：
            #   非有限值一旦进入下面的 _last_cmd 就会**永久毒化**限幅基准，
            #   之后每条命令都是 NaN、电机动不了且永不自愈。
            self._bad_cmd_count = getattr(self, '_bad_cmd_count', 0) + 1
            if self._bad_cmd_count <= 3 or self._bad_cmd_count % 500 == 0:
                self.get_logger().warn(
                    f'忽略含 NaN/Inf 或长度不符的命令 (#{self._bad_cmd_count})'
                )
            return

        target_positions = {}
        raw_values = {}   # 用于限幅：保存归一化后的原始目标（key = 纯 motor 名）
        for i, joint in enumerate(self.JOINT_NAMES):
            v = self.radians_to_normalized(joint, msg.data[i])
            # ⚠️ 键名必须是「纯 motor 名」（不带 .pos 后缀）！
            # FeetechMotorsBus.sync_write 内部用键去查 bus.motors（键为 shoulder_pan 等），
            # 传 "shoulder_pan.pos" 会 KeyError。这与 SO101Follower.send_action 的做法一致：
            # 那边 get_action 得到的也是 ".pos" 键，但它先 removesuffix(".pos") 再 sync_write。
            raw_values[joint] = v
            target_positions[joint] = v

        # ---- 安全限幅（max_relative_target，单位同 radians_to_normalized 的输出）----
        # 作用：leader 跟随 follower 时，若 follower 位置发生突变（例如刚接管完、
        # 或某关节被手动挪过），单步命令不会超过 max_relative_target，避免舵机猛冲。
        # 语义与 lerobot 的 ensure_safe_goal_position 类似，但以「上一条命令」为基准
        # （不读电机当前值，省一次总线往返，10Hz 下更稳）。
        if self.max_relative_target is not None:
            if getattr(self, '_last_cmd', None) is None:
                # 首条命令不做限幅，否则会从 0 开始把 leader 拉到 0 附近
                self._last_cmd = dict(raw_values)
            else:
                cap = self.max_relative_target
                for joint in self.JOINT_NAMES:
                    prev = self._last_cmd.get(joint, raw_values[joint])
                    diff = raw_values[joint] - prev
                    if diff > cap:
                        diff = cap
                    elif diff < -cap:
                        diff = -cap
                    new_v = prev + diff
                    target_positions[joint] = new_v
                    raw_values[joint] = new_v
                self._last_cmd = dict(raw_values)

        try:
            # 直接写 Goal_Position（normalize=True 是默认值，与 SO101Follower.send_action
            # 内部 bus.sync_write(..., normalize=True) 一致）。
            self.robot.bus.sync_write('Goal_Position', target_positions)
        except Exception as e:
            # 限流，避免刷屏（leader 以 10~30Hz 收命令，出错会很快刷满日志）
            self._cmd_err_count = getattr(self, '_cmd_err_count', 0) + 1
            if self._cmd_err_count <= 3 or self._cmd_err_count % 100 == 0:
                self.get_logger().error(
                    f'Failed to drive leader arm [#{self._cmd_err_count}] '
                    f'{type(e).__name__}: {e}'
                )

    # 力矩开关：模式 A/B 切换时使用
    def _set_torque_callback(self, request, response):
        """ROS2 service 回调：/leader/set_torque (std_srvs/SetBool)。

        模式 A（policy 控制）→ data=True ：leader 力矩 ON，跟随 follower
        模式 B（人类接管）  → data=False：leader 力矩 OFF，人类自由推动

        注意：模式 A 下若力矩开启，人手推 leader 会与电机对抗（抖振/发热）。
        所以接管前务必先设为 False。
        """
        ok = self.set_leader_torque(bool(request.data))
        response.success = ok
        response.message = f'leader torque -> {"ON" if request.data else "OFF"}'
        return response

    def set_leader_torque(self, enabled: bool) -> bool:
        """开/关 leader 全部电机的力矩。返回是否成功。

        ★ 开力矩前**只对当前松着的电机**把 `Goal_Position` 同步成实测位置：
          · 松着的电机不同步 → 开力矩瞬间会猛冲"上一次运行留下的目标"
            （2026-09-29 就是这么把一次归位跑废的：主臂突然跳起来又回去）
          · 已经带力矩的电机若也同步 → 会抹掉它顶重力的静差偏置 → 下垂几度
          详见 `_prepare_torque_enable()` / `_sync_goal_to_present()` 的注释。
        """
        try:
            if enabled:
                self._prepare_torque_enable('leader')
                self.robot.bus.enable_torque()
            else:
                self.robot.bus.disable_torque()
            self.get_logger().info(f'Leader torque -> {"ON" if enabled else "OFF"}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to set leader torque: {type(e).__name__}: {e}')
            return False
