#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-08-31
################################################################
"""Provide the ROS 2 interface for the Isaac Sim Archer bridge."""

import threading

import numpy as np
import rclpy
import rclpy.node
from sensor_msgs.msg import JointState
from hex_ros_msgs.msg import HexRosJnt, HexRosRoboManipCtrlStamped

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader, HexDcBaseJntFull, HexDcBasePose, HexDcBaseQuaternion,
    HexDcBaseVector3,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl, HexDcRoboArmCtrlMode, HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode, HexDcRoboManipCtrl, HexDcRoboManipCtrlStamped,
)
from .interface_base import ArmInterfaceBase


class DataInterface(ArmInterfaceBase):
    """Provide ROS 2 transport services for the Archer Bridge node.

    ROS callbacks convert messages and enqueue them; control-mode validation,
    interpolation, and torque limiting remain in the Archer node.
    """

    def __init__(self, name: str = "unknown") -> None:
        """Initialize ROS 2 publishers, subscribers, and the spin thread.

        Args:
            name: ROS 2 node name.

        """
        rclpy.init()
        self.__node = rclpy.node.Node(
            name, automatically_declare_parameters_from_overrides=True)
        self._logger = self.__node.get_logger()
        # Prevent repeated shutdown calls from destroying ROS resources twice.
        self._shutting_down = False
        super().__init__(name)

        self._rate_param["ros"] = float(
            self.__node.get_parameter("ctrl_rate").value)
        self._robot_param = {
            "grip_type": str(
                self.__node.get_parameter("robot_grip_type").value),
            "urdf_path": str(self.__node.get_parameter("urdf_path").value),
        }
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        self.__command_pub = self.__node.create_publisher(
            JointState,
            self.__node.get_parameter("joint_command_topic").value,
            10)
        self.__ctrl_sub = self.__node.create_subscription(
            HexRosRoboManipCtrlStamped,
            "manip_ctrl",
            self.__manip_ctrl_callback,
            10)
        self.__joint_state_sub = self.__node.create_subscription(
            JointState,
            self.__node.get_parameter("joint_state_topic").value,
            self.__joint_state_callback,
            10)

        self.__spin_thread = threading.Thread(
            target=self.__spin, daemon=True)
        self.__spin_thread.start()

    def sleep(self) -> None:
        """Sleep for one configured control period."""
        self.__rate.sleep()

    def ok(self) -> bool:
        """Return whether the ROS 2 context is active.

        Returns:
            ``True`` while ROS 2 is running.

        """
        return rclpy.ok()

    def shutdown(self) -> None:
        """Destroy ROS resources and stop the spin thread."""
        if self._shutting_down:
            return
        self._shutting_down = True
        self.__node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.__spin_thread.join(timeout=1.0)

    def __spin(self) -> None:
        """Process ROS callbacks until the context is shut down."""
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass
        except Exception:
            if not self._shutting_down:
                raise

    def logi(self, msg, *args, **kwargs) -> None:
        """Write an informational log."""
        self._logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs) -> None:
        """Write a warning log."""
        self._logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs) -> None:
        """Write an error log."""
        self._logger.error(msg, *args, **kwargs)

    def logd(self, msg, *args, **kwargs) -> None:
        """Write a debug log."""
        self._logger.debug(msg, *args, **kwargs)

    def pub_joint_command(self, names, position, effort) -> None:
        """Publish position targets and feed-forward effort.

        Args:
            names: Bridge joint names defining the output order.
            position: Position targets corresponding to ``names``.
            effort: Effort feed-forward values corresponding to ``names``.

        """
        msg = JointState()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = np.asarray(position, dtype=np.float64).tolist()
        msg.effort = np.asarray(effort, dtype=np.float64).tolist()
        self.__command_pub.publish(msg)

    @staticmethod
    def __jnt_to_dc(jnt: HexRosJnt) -> HexDcBaseJntFull:
        """Convert a ROS joint command into the shared dataclass."""
        return HexDcBaseJntFull(
            pos=np.asarray(jnt.pos, dtype=np.float64),
            vel=np.asarray(jnt.vel, dtype=np.float64),
            eff=np.asarray(jnt.eff, dtype=np.float64),
            kp=np.asarray(jnt.kp, dtype=np.float64),
            kd=np.asarray(jnt.kd, dtype=np.float64),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64))

    @staticmethod
    def __manip_ctrl_to_dc(msg: HexRosRoboManipCtrlStamped):
        """Convert a ROS manipulator command without applying control.

        The node applies mode validation and target generation after this
        transport-level conversion.
        """
        arm = msg.manip_ctrl.arm_ctrl
        grip = msg.manip_ctrl.grip_ctrl
        return HexDcRoboManipCtrlStamped(
            header=HexDcBaseHeader(frame_id=msg.header.frame_id),
            manip_ctrl=HexDcRoboManipCtrl(
                arm_ctrl=HexDcRoboArmCtrl(
                    ctrl_mode=HexDcRoboArmCtrlMode(int(arm.ctrl_mode)),
                    grav=HexDcBaseVector3(
                        x=arm.grav.x, y=arm.grav.y, z=arm.grav.z),
                    jnt=DataInterface.__jnt_to_dc(arm.jnt),
                    pose=HexDcBasePose(
                        position=HexDcBaseVector3(
                            x=arm.pose.position.x,
                            y=arm.pose.position.y,
                            z=arm.pose.position.z),
                        orientation=HexDcBaseQuaternion(
                            x=arm.pose.orientation.x,
                            y=arm.pose.orientation.y,
                            z=arm.pose.orientation.z,
                            w=arm.pose.orientation.w))),
                grip_ctrl=HexDcRoboGripCtrl(
                    ctrl_mode=HexDcRoboGripCtrlMode(int(grip.ctrl_mode)),
                    jnt=DataInterface.__jnt_to_dc(grip.jnt))))

    def __manip_ctrl_callback(self, msg: HexRosRoboManipCtrlStamped) -> None:
        """Convert and queue an incoming manipulator command.

        Args:
            msg: ROS 2 manipulator control message.

        """
        self.logd(f"received manip control: {msg.manip_ctrl.arm_ctrl.ctrl_mode}")
        self._manip_ctrl_deque.append(self.__manip_ctrl_to_dc(msg))

    def __joint_state_callback(self, msg: JointState) -> None:
        """Cache the first non-empty Bridge joint-name frame.

        Args:
            msg: Joint state message published by Isaac Sim.

        """
        self.set_joint_names(msg.name)
