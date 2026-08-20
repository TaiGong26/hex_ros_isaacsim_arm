#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-20
################################################################
# Mock Isaac Sim ros_bridge for local testing.
#
# Subscribes to the deliverable's torque command topic (JointState.effort),
# integrates a simple per-joint model  qdd = (tau - b*qd) / m  and publishes
# joint_states (name/position/velocity/effort) on the bridge state topic.
# One parameter set tests arm / maver / trigger_a3 alike.

import threading

import numpy as np

import rclpy
import rclpy.node

from sensor_msgs.msg import JointState


class MockBridge:

    def __init__(self):
        rclpy.init()
        self.__node = rclpy.node.Node("mock_bridge")
        self.__logger = self.__node.get_logger()
        self.__shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        ### parameters
        self.__node.declare_parameter("cmd_topic", "/hex_arm_cmd")
        self.__node.declare_parameter("state_topic", "/joint_states")
        self.__node.declare_parameter("joint_names", "")
        self.__node.declare_parameter("mass", 1.0)
        self.__node.declare_parameter("damping", 1.0)
        self.__node.declare_parameter("pub_rate", 500.0)
        # fixed bridge stamp (sec); "0" = use the real clock. Declared as string
        # so launch string overrides don't trip the double/string type check.
        self.__node.declare_parameter("fixed_stamp_sec", "0.0")

        raw_names = self.__node.get_parameter("joint_names").value
        if isinstance(raw_names, str):
            self.__joint_names = [
                n.strip() for n in raw_names.split(",") if n.strip()
            ]
        else:
            self.__joint_names = [str(n) for n in raw_names]
        self.__pub_rate = float(
            self.__node.get_parameter("pub_rate").value)
        self.__fixed_stamp_sec = float(
            self.__node.get_parameter("fixed_stamp_sec").value)
        if not self.__joint_names:
            self.__logger.error("joint_names is empty, mock_bridge disabled")

        n = len(self.__joint_names)
        self.__mass = self.__broadcast_param("mass", n, 1.0)
        self.__damping = self.__broadcast_param("damping", n, 1.0)

        self.__q = np.zeros(n)
        self.__qd = np.zeros(n)
        self.__tau = np.zeros(n)
        self.__lock = threading.Lock()

        ### sub — bridge torque command
        self.__cmd_sub = self.__node.create_subscription(
            JointState,
            self.__node.get_parameter("cmd_topic").value,
            self.__cmd_callback,
            10,
        )
        ### pub — bridge joint_states
        self.__state_pub = self.__node.create_publisher(
            JointState,
            self.__node.get_parameter("state_topic").value,
            10,
        )

    def __broadcast_param(self, key: str, n: int, default: float):
        raw = self.__node.get_parameter(key).value
        if isinstance(raw, (list, tuple)):
            arr = np.asarray(raw, dtype=np.float64)
            if arr.size != n:
                self.__logger.warning(
                    f"{key} size {arr.size} != {n}, broadcasting first value")
                arr = np.full(n, arr[0] if arr.size else default)
            return arr
        return np.full(n, float(raw))

    def __cmd_callback(self, msg: JointState):
        with self.__lock:
            # adopt joint names from the first cmd message when not configured
            if not self.__joint_names and msg.name:
                self.__joint_names = list(msg.name)
                n = len(self.__joint_names)
                self.__mass = np.full(n, self.__mass[0] if self.__mass.size else 1.0)
                self.__damping = np.full(n, self.__damping[0] if self.__damping.size else 1.0)
                self.__q = np.zeros(n)
                self.__qd = np.zeros(n)
                self.__tau = np.zeros(n)
            # map effort by name (defensive against ordering differences)
            eff_map = dict(zip(msg.name, msg.effort))
            self.__tau = np.asarray(
                [eff_map.get(n, 0.0) for n in self.__joint_names],
                dtype=np.float64,
            )

    def run(self):
        rate = self.__node.create_rate(self.__pub_rate)
        dt = 1.0 / self.__pub_rate
        while rclpy.ok():
            with self.__lock:
                names = list(self.__joint_names)
                # wait until names are known (configured or adopted from cmd)
                if not names:
                    continue
                # qdd = (tau - b*qd) / m ; semi-implicit Euler
                self.__qd += (self.__tau - self.__damping * self.__qd) \
                    / self.__mass * dt
                self.__q += self.__qd * dt
                pos, vel, tau = self.__q.copy(), self.__qd.copy(), self.__tau.copy()

            msg = JointState()
            if self.__fixed_stamp_sec != 0.0:
                msg.header.stamp.sec = int(self.__fixed_stamp_sec)
                msg.header.stamp.nanosec = 0
            else:
                msg.header.stamp = self.__node.get_clock().now().to_msg()
            msg.name = names
            msg.position = pos.tolist()
            msg.velocity = vel.tolist()
            msg.effort = tau.tolist()
            self.__state_pub.publish(msg)
            rate.sleep()

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def shutdown(self):
        if self.__shutting_down:
            return
        self.__shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            self.__spin_thread.join()
        except Exception:
            pass


def main():
    mock = MockBridge()
    try:
        mock.run()
    except KeyboardInterrupt:
        pass
    finally:
        mock.shutdown()


if __name__ == '__main__':
    main()
