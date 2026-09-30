from __future__ import annotations

import math
from typing import Any, List, Mapping, Optional

import numpy as np
from openpi_client.websocket_client_policy import WebsocketClientPolicy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from ..utils.buffer import ActionBuffer
from ..utils.conversion import ros_image_to_hwc_uint8, ros_jointstate_to_vec
from ..utils.filtering import LowPassFilter
from .base import BasePolicy, PolicyConfig
from .registry import register_policy


@register_policy("openpi_remote")
class OpenPIRemote(BasePolicy):
    def __init__(self, cfg: PolicyConfig, node: Node) -> None:
        super().__init__(cfg, node)
        self.node = node

        extra = cfg.extra or {}
        server_cfg = extra.get("server", {})
        obs_cfg = extra.get("observation", {})
        lpf_cfg = extra.get("lpf_filtering", {})

        host = server_cfg.get("host", "127.0.0.1")
        port = int(server_cfg.get("port", 8080))

        self._base_image_key = obs_cfg.get("base_image_key", "observation.images.cam_main")
        self._wrist_image_key = obs_cfg.get("wrist_image_key", "observation.images.cam_wrist")
        self._state_key = obs_cfg.get("state_key", "observation.state")
        self._task = cfg.task

        robot_props = cfg.robot_properties
        self._state_joint_names: List[str] = robot_props.get(
            "state_joint_names",
            robot_props.get("joint_names"),
        )

        self._client = WebsocketClientPolicy(host=host, port=port)
        self.node.get_logger().info(
            f"[OpenPIRemote] Connected to OpenPI server {host}:{port}, "
            f"metadata={self._client.get_server_metadata()}"
        )

        lpf = None
        if lpf_cfg.get("enable", False):
            lpf = LowPassFilter(alpha=float(lpf_cfg.get("alpha", 0.1)))
        self._action_buffer = ActionBuffer(lpf=lpf)

    def make_observation(self, ros_obs: Mapping[str, Any]) -> dict:
        base_img = ros_image_to_hwc_uint8(ros_obs[self._base_image_key])
        wrist_img = ros_image_to_hwc_uint8(ros_obs[self._wrist_image_key])
        state, _ = ros_jointstate_to_vec(
            ros_obs[self._state_key],
            joint_order=self._state_joint_names,
            use_lerobot_ranges_norms=False,
        )

        return {
            "observation/image": base_img,
            "observation/wrist_image": wrist_img,
            "observation/state": state.astype(np.float32),
            "prompt": self._task,
        }

    def predict_action_chunk(
        self,
        observation: dict,
        time_per_action: float,
        inference_delay: float,
    ) -> None:
        result = self._client.infer(observation)
        actions = np.asarray(result["actions"], dtype=np.float32)

        if actions.ndim != 2:
            self.node.get_logger().error(
                f"[OpenPIRemote] Expected actions shape [T, D], got {actions.shape}"
            )
            return

        actions_list = actions.tolist()
        if not actions_list:
            self.node.get_logger().warn("[OpenPIRemote] Empty action chunk returned.")
            return

        if inference_delay > 0.0 and time_per_action > 0.0:
            start_index = math.ceil(inference_delay / time_per_action)
        else:
            start_index = 0

        self._action_buffer.set(actions_list, start_index=start_index)
        self.node.get_logger().log(
            f"[OpenPIRemote] Received action chunk shape={actions.shape}, "
            f"start_index={start_index}",
            severity=LoggingSeverity.INFO,
            throttle_duration_sec=5.0,
        )

    def get_action(self) -> Optional[List[float]]:
        return self._action_buffer.get()

    def reset(self, context: Optional[Mapping[str, Any]] = None) -> None:
        self._action_buffer.reset()
