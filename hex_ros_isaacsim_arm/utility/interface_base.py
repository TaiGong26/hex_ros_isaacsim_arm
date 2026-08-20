#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################
# bridge-forwarding adaptation: joint names are taken dynamically from the
# bridge joint_states `name` field; a generated fallback is used before the
# first bridge state arrives.

from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod

import numpy as np

from hex_util_msg.dataclass.dataclass_base import HexDcBaseTime
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboManipCtrlStamped,
    HexDcRoboManipStateStamped,
)


class ArmInterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        self._name = name
        self._arm_dof = 6
        self._grip_dof = 1

        ### ros parameters
        self._rate_param = {}
        self._robot_param = {}
        self._bridge_param = {}

        ### rx msg queues
        self._manip_ctrl_deque = deque(maxlen=100)

        ### bridge state cache (latest joint_states from the bridge)
        self._bridge_stamp = HexDcBaseTime()
        self._bridge_names = []
        self._bridge_state = {}  # name -> (pos, vel, eff)

        ### joint names (dynamic from bridge; generated fallback before arrival)
        self._arm_joint_names = []
        self._grip_joint_names = []
        self._reset_generated_joint_names()

    ####################
    ### ros infrastructure
    ####################
    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("ArmInterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("ArmInterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("ArmInterfaceBase.sleep")

    @abstractmethod
    def now_ns(self) -> int:
        raise NotImplementedError("ArmInterfaceBase.now_ns")

    @abstractmethod
    def now_stamp(self) -> HexDcBaseTime:
        raise NotImplementedError("ArmInterfaceBase.now_stamp")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("ArmInterfaceBase.logf")

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_robot_param(self) -> dict:
        return self._robot_param

    def get_bridge_param(self) -> dict:
        return self._bridge_param

    def set_dofs(self, arm_dof: int, grip_dof: int):
        """arm/grip DOF from the machine type; refresh generated fallback names."""
        self._arm_dof = arm_dof
        self._grip_dof = grip_dof
        self._reset_generated_joint_names()

    ####################
    ### joint names & bridge state cache
    ####################
    def _reset_generated_joint_names(self):
        """Fallback names used before the first bridge joint_states arrives."""
        arm_prefix = self._bridge_param.get("arm_prefix", "arm_joint_")
        grip_prefix = self._bridge_param.get("grip_prefix", "grip_joint_")
        self._arm_joint_names = [
            f"{arm_prefix}{i}" for i in range(1, self._arm_dof + 1)
        ]
        self._grip_joint_names = [
            f"{grip_prefix}{i}" for i in range(1, self._grip_dof + 1)
        ]

    def set_bridge_state(self, names, position, velocity, effort, stamp):
        """Cache the latest bridge joint_states (called from the ROS callback)."""
        self._bridge_stamp = stamp
        self._bridge_names = list(names)
        self._bridge_state = {
            n: (float(position[i]), float(velocity[i]), float(effort[i]))
            for i, n in enumerate(self._bridge_names)
        }
        self._update_joint_names_from_bridge()

    def _update_joint_names_from_bridge(self):
        """Split bridge names into arm/grip groups by prefix (bridge order)."""
        arm_prefix = self._bridge_param.get("arm_prefix", "arm_joint_")
        grip_prefix = self._bridge_param.get("grip_prefix", "grip_joint_")
        arm_names = [n for n in self._bridge_names if n.startswith(arm_prefix)]
        grip_names = [n for n in self._bridge_names if n.startswith(grip_prefix)]
        if len(arm_names) >= self._arm_dof:
            self._arm_joint_names = arm_names[:self._arm_dof]
        if len(grip_names) >= self._grip_dof:
            self._grip_joint_names = grip_names[:self._grip_dof]

    def get_arm_joint_names(self) -> list:
        return list(self._arm_joint_names)

    def get_grip_joint_names(self) -> list:
        return list(self._grip_joint_names)

    def get_bridge_stamp(self) -> HexDcBaseTime:
        return self._bridge_stamp

    def get_forward_stamp(self) -> HexDcBaseTime:
        """Header stamp for forwarded msgs: bridge sim time or ROS now."""
        if self._bridge_param.get("time_source", "sim") == "ros":
            return self.now_stamp()
        return self.get_bridge_stamp()

    def get_arm_state(self) -> np.ndarray:
        """(pos, vel, eff) of arm joints in arm_joint_names order."""
        pos = np.zeros(self._arm_dof)
        vel = np.zeros(self._arm_dof)
        eff = np.zeros(self._arm_dof)
        for i, n in enumerate(self._arm_joint_names):
            v = self._bridge_state.get(n)
            if v is not None:
                pos[i], vel[i], eff[i] = v
        return pos, vel, eff

    def get_grip_state(self) -> np.ndarray:
        """(pos, vel, eff) of grip joints in grip_joint_names order."""
        pos = np.zeros(self._grip_dof)
        vel = np.zeros(self._grip_dof)
        eff = np.zeros(self._grip_dof)
        for i, n in enumerate(self._grip_joint_names):
            v = self._bridge_state.get(n)
            if v is not None:
                pos[i], vel[i], eff[i] = v
        return pos, vel, eff

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_manip_state(self, out: HexDcRoboManipStateStamped):
        raise NotImplementedError("ArmInterfaceBase.pub_manip_state")

    @abstractmethod
    def pub_bridge_cmd(self, names: list, effort: np.ndarray):
        raise NotImplementedError("ArmInterfaceBase.pub_bridge_cmd")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            if dq:
                ret = dq[-1]
                dq.clear()
                return ret
            else:
                return None

    # manip ctrl
    def get_manip_ctrl(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboManipCtrlStamped]:
        return self.deque_helper(self._manip_ctrl_deque, latest)
