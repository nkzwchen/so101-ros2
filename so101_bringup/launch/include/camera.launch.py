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

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

PKG_BRINGUP = 'so101_bringup'
SHARE_DIR = get_package_share_directory(PKG_BRINGUP)

# ★ 相机节点之间必须错开启动的秒数（2026-09 实测确定）。
#   见 generate_launch_description() 里的说明。
CAMERA_START_STAGGER_SEC = 3.0

# Map "type" from the cameras YAML to (package, executable)
TYPE_REGISTRY: Dict[str, Tuple[str, str]] = {
    'usb_camera': ('usb_cam', 'usb_cam_node_exe'),
    'realsense2_camera': ('realsense2_camera', 'realsense2_camera_node'),
}


def load_yaml_from_pkg(package_name: str, rel_path: str) -> Any:
    """Load YAML at <share(package)>/<rel_path>."""
    package_path = get_package_share_directory(package_name)
    abs_path = os.path.join(package_path, rel_path)
    with open(abs_path, 'r') as f:
        return yaml.safe_load(f)


@dataclass
class CameraSpec:
    name: str
    camera_type: str
    param_path: Path
    namespace: Optional[str] = None
    remappings: Optional[List[Tuple[str, str]]] = None  # optional per-camera overrides


def _resolve_param_path(relative_param_path: str) -> Path:
    # Your param files live under PKG_BRINGUP/config/<file>.yaml
    p = Path(SHARE_DIR) / 'config' / relative_param_path
    if not p.exists():
        raise FileNotFoundError(f'Parameter file not found: {p}')
    return p


def _load_camera_param_yaml(param_path: Path) -> Dict[str, Any]:
    """读单个相机的参数 yaml。顶层键是节点名通配符（如 '**'、'/follower/cam_main'）。"""
    with open(param_path, 'r') as f:
        return yaml.safe_load(f) or {}


def _extract_video_device(param_cfg: Dict[str, Any], fqn: str) -> Optional[str]:
    """从参数 yaml 中取出该相机节点的 video_device。

    优先精确匹配全限定节点名（如 /follower/cam_main），其次回退到 '/**' 通配段。
    """
    for key in (fqn, '/**', '**'):
        section = param_cfg.get(key)
        if not isinstance(section, dict):
            continue
        params = section.get('ros__parameters')
        if isinstance(params, dict) and params.get('video_device'):
            return str(params['video_device'])
    return None


def _resolve_video_device_for_usb_cam(device: str, cam_name: str) -> str:
    """把 by-id 软链解析成 usb_cam 真正认得的字面设备路径。

    ★ 为什么必须做这一步（2026-09 实测确认）：
      usb_cam 0.8.1 **只接受字面量 `/dev/videoN`**。若传入软链，它会对软链做
      **单层 readlink()（不递归解析）**，然后**无条件再拼一次 `/dev/`**，
      得到根本不存在的路径，节点随即报错并退出：

          Device specified is not available or is not a vaild V4L2 device: `...`

      实测证据（关键：软链的**写法**决定拼出什么错误路径）：
          传 "/dev/video0"                  → ✅ 正常打开
          传 "0"                            → ❌ `/dev/0`（不支持裸编号）
          传 by-id 软链（绝对 --> /dev/video0）→ ❌ `/dev//dev/video0`
          传自建软链（绝对 --> /dev/video0）  → ❌ `/dev//dev/video0`
          传 by-id 软链（relative --> ../../video0）→ ❌ `/dev/../../video0`

      另外注意：**设备编号本身极不稳定**。实测中同一台相机在几分钟内
      从 video0 漂到 video2、又漂到 video4（UVC 设备经 USB/IP 重新枚举时
      会重新分配 minor 号）。所以既不能硬编码 videoN，也不能只在启动时
      读一次就写死。

      结论：在这里、于 launch 阶段把 by-id 解析成当前字面路径 ——
      既让 usb_cam 能打开，又保留 yaml 里 by-id 的抗漂移能力。
    """
    resolved = os.path.realpath(device)
    if not os.path.exists(resolved):
        raise RuntimeError(
            f'相机 {cam_name} 的设备不存在：配置写的是 {device!r}，'
            f'解析后为 {resolved!r}。请确认相机已 attach，且 by-id 链接存在：'
            f'ls -la /dev/v4l/by-id/'
        )
    return resolved


def parse_cameras_config(
    config_file_rel: str = 'config/so101_cameras.yaml',
) -> List[CameraSpec]:
    cfg = load_yaml_from_pkg(PKG_BRINGUP, config_file_rel)
    if not cfg or 'cameras' not in cfg:
        raise RuntimeError(f"No valid 'cameras' list found in {config_file_rel}")

    cams: List[CameraSpec] = []
    for cam in cfg['cameras']:
        name = cam.get('name')
        cam_type = cam.get('camera_type')
        rel_param = cam.get('param_path')
        namespace = cam.get('namespace')
        remappings = cam.get('remappings')  # optional

        if not name or not cam_type or not rel_param:
            raise ValueError(
                f"Camera entry is missing one of 'name'/'camera_type'/'param_path': {cam}"
            )
        if cam_type not in TYPE_REGISTRY:
            raise ValueError(
                f"Unknown camera type '{cam_type}'. Supported: {list(TYPE_REGISTRY.keys())}"
            )

        cams.append(
            CameraSpec(
                name=name,
                camera_type=cam_type,
                param_path=_resolve_param_path(rel_param),
                namespace=namespace,
                remappings=remappings,  # may be None
            )
        )
    return cams


def build_node_for_camera(spec: CameraSpec) -> Node:
    pkg, exe = TYPE_REGISTRY[spec.camera_type]

    # Common override: ensure camera_name matches the camera "name" from the list
    # Both usb_cam and realsense2_camera honor a 'camera_name' parameter.
    overrides: Dict[str, Any] = {
        'camera_name': spec.name,
    }

    # Optional: sensible defaults that often help (comment/uncomment as needed)
    # For usb_cam you might also want to use a frame_id matching the camera name:
    if spec.camera_type == 'usb_camera':
        overrides.setdefault('frame_id', spec.name)

        # ★ usb_cam 不认软链，必须在 launch 阶段解析成字面 /dev/videoN 再注入。
        #   参数合并规则：parameters=[yaml, overrides] 后者优先，所以这里能覆盖 yaml。
        fqn = f'/{spec.namespace}/{spec.name}' if spec.namespace else f'/{spec.name}'
        raw_device = _extract_video_device(_load_camera_param_yaml(spec.param_path), fqn)
        if raw_device:
            resolved = _resolve_video_device_for_usb_cam(raw_device, spec.name)
            if resolved != raw_device:
                print(f'[camera.launch] {spec.name}: {raw_device} -> {resolved}')
            overrides['video_device'] = resolved
        else:
            print(f'[camera.launch] ⚠️ {spec.name}: 参数文件里没找到 video_device，'
                  f'将直接交给 usb_cam（软链会失败）')

    # Build parameters list: first the YAML file path, then the overrides dict.
    # ROS 2 merges later entries over earlier ones, so our overrides win.
    parameters = [str(spec.param_path), overrides]

    # Remappings:
    # - If you didn’t define custom remappings in the YAML, you can keep defaults.
    # - Example for usb_cam (single stream):
    default_usb_cam_remaps = [
        ('image_raw', f'{spec.name}/image_raw'),
        ('image_raw/compressed', f'{spec.name}/image_compressed'),
        ('image_raw/compressedDepth', f'{spec.name}/compressedDepth'),
        ('image_raw/theora', f'{spec.name}/image_raw/theora'),
        ('camera_info', f'{spec.name}/camera_info'),
    ]

    # For RealSense, topic structure is richer; if you want uniform names,
    # define them per-camera in so101_cameras.yaml under 'remappings'.
    remappings = spec.remappings
    if remappings is None and spec.camera_type == 'usb_camera':
        remappings = default_usb_cam_remaps

    return Node(
        package=pkg,
        executable=exe,
        name=spec.name,  # node name (distinct per camera)
        namespace=spec.namespace,  # logical grouping per your config
        output='screen',
        parameters=parameters,
        remappings=remappings or [],
    )


def generate_launch_description():
    camera_specs = parse_cameras_config()

    # ★★ 相机节点必须**错开启动**，不能同时起（2026-09 实测）★★
    #
    #   现象：两个 usb_cam 节点同时启动时，**必有一个**在控制项初始化阶段
    #         直接崩溃退出：
    #             terminate called after throwing an instance of 'char*'
    #         谁死是随机的 —— 谁抢输谁死（实测两种顺序都出现过）。
    #
    #   根因：两个进程同时通过 USB/IP(vhci_hcd) 对各自的 UVC 设备做
    #         控制项设置 + stream-on。本机 USB/IP 链路本就脆弱
    #         （dmesg 持续刷 `vhci_hcd: urb->status -104` = ECONNRESET），
    #         抢占失败的一方设备被重置，后续 ioctl 失败 →
    #         usb_cam 内部 `throw` 了一个 char* 字面量 → 未捕获 → SIGABRT。
    #
    #   实测对照（同一状态、每次重新解析路径，各观察 11 秒）：
    #         stagger = 0s  →  cam_main 已死 / cam_wrist 存活   ❌
    #         stagger = 3s  →  cam_main 存活 / cam_wrist 存活   ✅
    #
    #   注：这与帧率无关（30fps 与 10fps 同时启动都必死一个），
    #       纯粹是"同时抢占"的时序问题。
    actions = []
    for idx, spec in enumerate(camera_specs):
        delay = idx * CAMERA_START_STAGGER_SEC
        node = build_node_for_camera(spec)
        if delay > 0:
            print(f'[camera.launch] {spec.name}: 延后 {delay:.0f}s 启动（错开 USB/IP 抢占）')
            actions.append(TimerAction(period=delay, actions=[node]))
        else:
            actions.append(node)

    return LaunchDescription(actions)
