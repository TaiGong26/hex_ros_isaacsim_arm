#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboManipCtrl,
    HexDcRoboManipStateStamped,
)


class TestInterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        ### ros parameters
        self._rate_param = {}

        ### rx msg queues
        self._manip_state_deque = deque(maxlen=100)

        ### name
        self._name = name

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass

    ####################
    ### ros infrastructure
    ####################
    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("TestInterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("TestInterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("TestInterfaceBase.sleep")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("TestInterfaceBase.logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("TestInterfaceBase.logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("TestInterfaceBase.logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("TestInterfaceBase.loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("TestInterfaceBase.logf")

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_manip_ctrl(self, out: HexDcRoboManipCtrl):
        raise NotImplementedError("TestInterfaceBase.pub_manip_ctrl")

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

    # manip state
    def get_manip_state(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboManipStateStamped]:
        return self.deque_helper(self._manip_state_deque, latest)
