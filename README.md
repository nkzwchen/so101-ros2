# so101-ros2 — SO101 的 ROS2 集成（HIL-SERL 改造版）

> 本仓库基于 [nimiCurtis/so101_ros2](https://github.com/nimiCurtis/so101_ros2)
> （SO101 机械臂的 ROS2 集成）改造，用于 **HIL-SERL 真机强化学习**：
> 提供「一套 launch 起全部节点、给 RL 环境喂关节+双相机、并接受逐帧关节指令」的接口。
>
> 上游原始 README 保留在 [`README_upstream.md`](README_upstream.md)。

---

## 1. 这套 ROS2 侧提供什么

```
                       ┌──────────────── so101_bringup (launch) ────────────────┐
   USB 串口            │                                                        │
 ┌──────────┐  /dev/ttyACM1  ┌──────────────────┐   /follower/joint_states      │
 │ leader   │───────────────▶│ follower_ros2_node│──────────────┐                │
 │ (主臂)   │                │  (ros2_control)   │              │                │
 └──────────┘                └──────────────────┘              ▼                │
        │                            ▲                 ┌──────────────────┐      │
        │ /leader/joint_states       │ /follower/       │  RL 环境 (env.py)│      │
        └───────────────────────────▶│  arm_controller/ │  · 读关节+相机   │      │
                                     │  joint_trajectory│  · 出关节增量指令│      │
 两个 USB 相机                       └──────────────────┘  · 人类接管状态机 │      │
 ┌──────────┐  /follower/cam_main/image_raw                    └──────────────────┘
 │ usb_cam  │──────────────────────────────────────────────────────────▶│
 │ ×2       │  /follower/cam_wrist/image_raw                            │
 └──────────┘                                                             │
```

- **`so101_ros2_bridge`** —— 串口 ↔ ROS2 桥。含 HIL-SERL 需要的**力矩开关**、
  **Goal_Position 同步**（避免使能瞬间跳变）、策略插件（含远程策略）。
- **`so101_bringup`** —— launch 与参数：控制器、相机、遥操作，以及本项目新增的
  **`so101_rl_env.launch.py`（一条命令起整套 RL 环境）**。
- **`so101_description`** —— URDF / mesh / USD。

---

## 2. 快速开始

```bash
# 1) 依赖 ROS2 Humble。本项目用 RoboStack 的 conda 环境（见 docs/so101/ROS2环境搭建-RoboStack方案.md）
conda activate hil-serl
source /opt/ros/humble/setup.bash

# 2) 编译
cd ~/pi_project/ros2_ws
colcon build --symlink-install
source install/local_setup.bash

# 3) 起整套 RL 环境（从臂 + 主臂 + 两个相机）
ros2 launch so101_bringup so101_rl_env.launch.py \
    use_leader:=true use_cameras:=true
```

常用开关：`use_leader`（是否起主臂桥）、`use_cameras`（是否起相机）、
`follower_port` / `leader_port`（串口，默认 `/dev/ttyACM1` / `/dev/ttyACM0`）。

**换串口/相机后必须确认设备号**：`/dev/videoN` 会随插拔顺序变化，
参数里用 `/dev/v4l/by-id/...` 的稳定符号链接（本仓库的相机配置已经这么做了）。

---

## 3. 话题与服务（RL 环境用到的部分）

| 名字 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/follower/joint_states` | `sensor_msgs/JointState` | 出 | 从臂 6 关节角 + 速度 |
| `/leader/joint_states` | `sensor_msgs/JointState` | 出 | 主臂 6 关节角 |
| `/follower/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 入 | 逐点下发关节目标（**多点 min-jerk 轨迹**，见下）|
| `/follower/cam_main/image_raw` | `sensor_msgs/Image` | 出 | 全局相机（640×480，第三视角）|
| `/follower/cam_wrist/image_raw` | `sensor_msgs/Image` | 出 | 腕部相机（**640×360**，见 §5）|
| `/follower/set_torque` | `std_srvs/SetBool` | 入 | 从臂力矩开关 |
| `/leader/set_torque` | `std_srvs/SetBool` | 入 | 主臂力矩开关（人接管时断开）|

---

## 4. 关键改动（相对上游）

### 4.1 新增 `so101_rl_env.launch.py`

一条命令起齐：从臂控制器 + 主臂桥 + 两个 `usb_cam`（按 by-id 解析成字面
`/dev/videoN` 再传给 `usb_cam`，并**错开启动** `CAMERA_START_STAGGER_SEC` 秒）。
不这么做的话，两台同型号相机同时枚举会互相抢设备，表现为偶发打不开。

### 4.2 桥的加固（`bridge.py`，+410 行）

| 改动 | 为什么 |
|---|---|
| 使能力矩前先 `_sync_goal_to_present()`，且**只同步力矩为 0 的电机** | `enable_torque()` 不写 `Goal_Position`，主臂会**突然跳到上次的目标角再弹回**（真机事故）。只同步力矩关闭的电机，才能保住重力补偿偏置 |
| 有限性校验 + 自愈 | 一旦 NaN 写进 `_last_cmd`，之后所有指令都是 NaN，从臂彻底不动。现在检测到非有限值就拒绝并复位 |
| `_read_torque_off_motors()` / `_prepare_torque_enable()` | 上面两条的支撑实现 |

### 4.3 相机配置

- `cam_main` = **全局视角** → `/dev/video0`（…2603031），**640×480**
- `cam_wrist` = **腕部视角** → `/dev/video2`（…2602030），**640×360**

> ⚠️ **这两个设备号曾经是反的！** 上游与 `lerobot/useful_script/record.sh`
> 互相矛盾（`record.sh` 说 index2、`eval_act.sh` 说 index0）。
> 判断方法很简单：**抓一帧看画面内容** —— 能看到整条臂+桌面的是全局，随臂晃动的是腕部。

---

## 5. 🔴 必读：腕部相机为什么必须是 640×360

`usb_cam` **0.8.1** 在用 `pixel_format:=mjpeg2rgb` 发布时，内部解码行距会被
**对齐补齐**，但 `sensor_msgs/Image.step` 仍然按 `width*3` 声明：

```
424 × 3 = 1272  →  实际按 1280 填充   （补齐 8 字节 = 2.67 像素/行）
Image.step = 1272                      ← 撒谎了
```

任何按 `step` reshape 的消费方都会拿到**整体斜切**的图像（斜条纹 + 彩虹伪彩）。
实测：

| 来源 | 拉普拉斯方差 | 逐行错位 |
|---|---|---|
| 同一台设备**直接** V4L2 抓 424×240 | 919 | 0.0 px |
| `usb_cam` 发出来的 424×240 | **4293** | **−2.68 px** |
| `usb_cam` 发出来的 640×360 | 正常 | 0.0 px |

**为什么 cam_main 没事**：`640×3 = 1920 = 15×128`，本来就是对齐的。
**结论**：这台设备上凡是 `width*3` 不是 128 倍数的分辨率（424×240、352×288、
800×480、800×600…）都会被弄歪；**640×360 / 640×480 / 1024×576 安全**。

640×360 与 424×240 **同为 16:9、视野完全一致**（已抓图逐像素对比），
所以选它：既安全，又比 640×480 省 25% 带宽。

> 自检：`python so101_tools/check_stride.py --topic /follower/cam_wrist/image_raw`
> 正常应看到 `逐行位移 +0.0 px`、方差 ~500–900。出现 4000+ 就是又歪了。

---

## 6. 相关文档

`docs/so101/`：

- `ROS2环境搭建-RoboStack方案.md` —— 从零搭这个环境（含踩坑）
- `so101_ros2_ros2_basics.md` —— ROS2 基础与机器人描述
- `so101_ros2_communication_code.md` —— 串口协议与桥的实现细节
- `so101_ros2_learning.md` —— 模仿学习/遥操作部分

训练侧（环境、录制、标定、HIL-SERL 流水线）见另一个仓库 **hil-serl-so101**。

---

## 7. 出处与许可

上游：[nimiCurtis/so101_ros2](https://github.com/nimiCurtis/so101_ros2)（MIT）。
本仓库保留上游全部历史与许可。
