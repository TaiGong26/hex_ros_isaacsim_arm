#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-08-31
################################################################
"""Define the transport abstraction for the Isaac Sim Archer bridge."""

from abc import ABC, abstractmethod
from collections import deque
from typing import Any, Optional

import numpy as np


class ArmInterfaceBase(ABC):
    """Define ROS-independent transport services for the Archer node.

    The interface stores incoming manipulator commands and forwards complete
    position/effort command frames to the Isaac Sim Bridge.
    """

    def __init__(self, name: str = "unknown") -> None:
        """Initialize transport state.

        Args:
            name: Logical name used by the concrete transport implementation.

        """
        self._name = name
        self._rate_param = {}
        self._robot_param = {}
        # Keep a bounded queue so disconnected control loops cannot grow it.
        self._manip_ctrl_deque = deque(maxlen=100)
        # Preserve the Bridge-provided order for outgoing JointState fields.
        self._joint_names = []

    def get_rate_param(self) -> dict:
        """Return the configured control-rate parameters.

        Returns:
            A dictionary containing the configured ROS rates.

        """
        return self._rate_param

    def get_robot_param(self) -> dict:
        """Return the configured Archer robot parameters.

        Returns:
            A dictionary containing gripper and URDF configuration.

        """
        return self._robot_param

    def set_joint_names(self, names: list) -> None:
        """Store the first non-empty Bridge joint-name frame.

        Args:
            names: Joint names in the order supplied by Isaac Sim.

        """
        if not self._joint_names and names:
            self._joint_names = list(names)

    def get_joint_names(self) -> list:
        """Return the cached Bridge joint names.

        Returns:
            A copy of the Bridge joint-name sequence.

        """
        return list(self._joint_names)

    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        """Retrieve one item using the requested queue policy.

        Args:
            dq: Queue from which to retrieve an item.
            latest: If true, discard older items and return the newest item.

        Returns:
            The selected item, or ``None`` when the queue is empty.

        """
        if not dq:
            return None
        if latest:
            # Apply the newest target and discard stale manipulator commands.
            value = dq[-1]
            dq.clear()
            return value
        return dq.popleft()

    def get_manip_ctrl(self, latest: bool = False):
        """Return one queued manipulator command.

        Args:
            latest: Whether to discard stale commands and return the newest.

        Returns:
            A manipulator command, or ``None`` when the queue is empty.

        """
        return self.deque_helper(self._manip_ctrl_deque, latest)

    @abstractmethod
    def ok(self) -> bool:
        """Return whether ROS is active."""
        raise NotImplementedError

    @abstractmethod
    def shutdown(self) -> None:
        """Release ROS resources."""
        raise NotImplementedError

    @abstractmethod
    def sleep(self) -> None:
        """Sleep for one control period."""
        raise NotImplementedError

    @abstractmethod
    def logi(self, msg, *args, **kwargs) -> None:
        """Write an informational log."""
        raise NotImplementedError

    @abstractmethod
    def logd(self, msg, *args, **kwargs) -> None:
        """Write a debug log."""
        raise NotImplementedError

    @abstractmethod
    def logw(self, msg, *args, **kwargs) -> None:
        """Write a warning log."""
        raise NotImplementedError

    @abstractmethod
    def loge(self, msg, *args, **kwargs) -> None:
        """Write an error log."""
        raise NotImplementedError

    @abstractmethod
    def pub_joint_command(self, names: list, position: np.ndarray,
                          effort: np.ndarray) -> None:
        """Publish one complete Bridge joint command.

        Args:
            names: Joint names defining the output order.
            position: Joint position targets in the same order as ``names``.
            effort: Joint effort feed-forward values in the same order.

        """
        raise NotImplementedError
