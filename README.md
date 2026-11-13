# SO101 ROS2：相机与机器人描述

本仓库负责 ROS2 相机启动与机器人描述，SO101 人工奖励训练见 [HIL-SERL 项目分支](https://github.com/nkzwchen/hil-serl-so101/tree/docs/so101-project-evidence-20261113)，六维抓取采集与 π0.5 性能实验见 [OpenPI 项目分支](https://github.com/nkzwchen/openpi/tree/docs/so101-project-evidence-20261113)。两路项目的动作/图像契约不同，不能直接混用数据。

配套训练仓库：`git@github.com:nkzwchen/hil-serl-so101.git`。两仓库使用开发分支 `feat/so101-human-reward-3d`；部署记录各自 commit。

当前录制/actor 使用 LeRobot 原生串口控制，仅相机依赖 ROS2。不要在录制时同时启动机器人串口桥或 ros2_control，否则争用两臂串口。旧桥仍保留供单独调试。

## 本机相机启动

激活已有 RoboStack 环境并加载工作空间后：

```bash
ros2 launch so101_bringup so101_record_cameras.launch.py
```

相机参数在 so101_bringup/config，设备通过 by-id 路径识别。相机丢帧和解码问题由采集日志记录；训练端保留质量标记并由人工复核。

本机使用 camera_driver_ws 中的 usb_cam 补丁版本；换机器时必须同步驱动版本和构建配置，单独克隆本仓库不会自动带入该工作空间。云端离线训练不需要安装 ROS2 或相机驱动。

URDF 在 so101_description/urdf。两仓库放在同一父目录；其他布局设置 SO101_URDF 为实际 URDF 绝对路径。

## 录制和训练

所有录制、人工成功帧标注、GPU 训练和 actor 命令以配套 hil-serl 仓库 README.md 为准。动作前三个关节相对控制，其他三个关节在录制阶段冻结；奖励来自每段结束后的人工标注，不使用视觉奖励模型。

历史 ROS 桥使用方式见 README_upstream.md 和 docs。设备未连接时只进行离线验证。
