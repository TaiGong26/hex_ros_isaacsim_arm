#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################
# bridge-forwarding DataInterface for the Isaac Sim archer:
#   - subscribes to bridge joint_states (all bridge topics are params)
#   - publishes a single torque command to the bridge
#   - forwards manip_state at rate_state with configurable time source

import numpy as np
import threading

from hex_util_runtime import ns_now

import rclpy
import rclpy.node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboManipStateStamped,
    HexRosRoboManipCtrlStamped,
)

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode,
    HexDcRoboManipCtrl,
    HexDcRoboManipCtrlStamped,
    HexDcRoboManipStateStamped,
)

from .interface_base import ArmInterfaceBase


class DataInterface(ArmInterfaceBase):

    def __init__(self, name: str = "unknown"):
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self._logger = self.__node.get_logger()
        self._shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        super().__init__(name)

        ### rate parameters
        self.__node.declare_parameter('ctrl_rate', 1000.0)
        self.__node.declare_parameter('rate_state', 500.0)
        self._rate_param["ros"] = self.__node.get_parameter('ctrl_rate').value
        self._rate_param["state"] = self.__node.get_parameter('rate_state').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### robot parameters
        self.__node.declare_parameter('robot_frame_id', "base_link")
        self.__node.declare_parameter('robot_grip_type', "empty")
        self.__node.declare_parameter('urdf_path', "")
        self._robot_param = {
            "frame_id": self.__node.get_parameter('robot_frame_id').value,
            "grip_type": self.__node.get_parameter('robot_grip_type').value,
            "urdf_path": self.__node.get_parameter('urdf_path').value,
        }

        ### bridge parameters (every bridge topic we subscribe to is a param)
        self.__node.declare_parameter('bridge_state_topic', "/joint_states")
        self.__node.declare_parameter('bridge_cmd_topic', "/hex_arm_cmd")
        self.__node.declare_parameter('time_source', "sim")
        self.__node.declare_parameter('arm_joint_prefix', "arm_joint_")
        self.__node.declare_parameter('grip_joint_prefix', "grip_joint_")
        self._bridge_param = {
            "state_topic": self.__node.get_parameter('bridge_state_topic').value,
            "cmd_topic": self.__node.get_parameter('bridge_cmd_topic').value,
            "time_source": self.__node.get_parameter('time_source').value,
            "arm_prefix": self.__node.get_parameter('arm_joint_prefix').value,
            "grip_prefix": self.__node.get_parameter('grip_joint_prefix').value,
        }

        ### publisher — manip_state
        self.__manip_state_pub = self.__node.create_publisher(
            HexRosRoboManipStateStamped,
            'manip_state',
            10,
        )
        ### publisher — bridge torque command
        self.__bridge_cmd_pub = self.__node.create_publisher(
            JointState,
            self._bridge_param["cmd_topic"],
            10,
        )

        ### subscriber — manip_ctrl
        self.__manip_ctrl_sub = self.__node.create_subscription(
            HexRosRoboManipCtrlStamped,
            'manip_ctrl',
            self.__manip_ctrl_callback,
            10,
        )
        self.__manip_ctrl_sub
        ### subscriber — bridge joint_states
        self.__bridge_state_sub = self.__node.create_subscription(
            JointState,
            self._bridge_param["state_topic"],
            self.__bridge_state_callback,
            10,
        )
        self.__bridge_state_sub

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### ros infrastructure
    ####################
    def ok(self) -> bool:
        return rclpy.ok()

    def shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        self._logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self._logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self._logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self._logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self._logger.fatal(msg, *args, **kwargs)

    ####################
    ### time source
    ####################
    def now_ns(self) -> int:
        return ns_now()

    def now_stamp(self) -> HexDcBaseTime:
        now = self.__node.get_clock().now()
        secs, nsecs = now.seconds_nanoseconds()
        return HexDcBaseTime(secs=secs, nsecs=nsecs)

    ####################
    ### publishers
    ####################
    def pub_manip_state(self, out: HexDcRoboManipStateStamped):
        msg = HexRosRoboManipStateStamped()
        # forward header time source: "sim" (bridge stamp) or "ros" (ROS now)
        forward_stamp = self.get_forward_stamp()
        hardware_stamp = Time(
            sec=int(forward_stamp.secs),
            nanosec=int(forward_stamp.nsecs),
        )
        msg.header.stamp = Time(
            sec=int(forward_stamp.secs),
            nanosec=int(forward_stamp.nsecs),
        )
        msg.header.frame_id = out.header.frame_id

        arm = out.manip_state.arm_state
        msg.manip_state.arm_state.jnt.header.stamp = hardware_stamp
        msg.manip_state.arm_state.jnt.header.frame_id = out.header.frame_id
        msg.manip_state.arm_state.jnt.name = self.get_arm_joint_names()
        msg.manip_state.arm_state.jnt.position = \
            np.asarray(arm.jnt.position, dtype=np.float64).tolist()
        msg.manip_state.arm_state.jnt.velocity = \
            np.asarray(arm.jnt.velocity, dtype=np.float64).tolist()
        msg.manip_state.arm_state.jnt.effort = \
            np.asarray(arm.jnt.effort, dtype=np.float64).tolist()
        msg.manip_state.arm_state.pose.position.x = arm.pose.position.x
        msg.manip_state.arm_state.pose.position.y = arm.pose.position.y
        msg.manip_state.arm_state.pose.position.z = arm.pose.position.z
        msg.manip_state.arm_state.pose.orientation.x = arm.pose.orientation.x
        msg.manip_state.arm_state.pose.orientation.y = arm.pose.orientation.y
        msg.manip_state.arm_state.pose.orientation.z = arm.pose.orientation.z
        msg.manip_state.arm_state.pose.orientation.w = arm.pose.orientation.w

        grip = out.manip_state.grip_state
        msg.manip_state.grip_state.jnt.header.stamp = hardware_stamp
        msg.manip_state.grip_state.jnt.header.frame_id = out.header.frame_id
        msg.manip_state.grip_state.jnt.name = self.get_grip_joint_names()
        msg.manip_state.grip_state.jnt.position = \
            np.asarray(grip.jnt.position, dtype=np.float64).tolist()
        msg.manip_state.grip_state.jnt.velocity = \
            np.asarray(grip.jnt.velocity, dtype=np.float64).tolist()
        msg.manip_state.grip_state.jnt.effort = \
            np.asarray(grip.jnt.effort, dtype=np.float64).tolist()

        self.__manip_state_pub.publish(msg)

    def pub_bridge_cmd(self, names: list, effort: np.ndarray):
        """Publish the single control torque to the bridge (JointState.effort)."""
        msg = JointState()
        forward_stamp = self.get_forward_stamp()
        msg.header.stamp = Time(
            sec=int(forward_stamp.secs),
            nanosec=int(forward_stamp.nsecs),
        )
        msg.name = list(names)
        msg.effort = np.asarray(effort, dtype=np.float64).tolist()
        self.__bridge_cmd_pub.publish(msg)

    ####################
    ### subscribers
    ####################
    def __manip_ctrl_callback(self, msg: HexRosRoboManipCtrlStamped):
        self._manip_ctrl_deque.append(self.__manip_ctrl_msg_to_dc(msg))

    def __bridge_state_callback(self, msg: JointState):
        names = list(msg.name)
        position = np.asarray(msg.position, dtype=np.float64)
        velocity = np.asarray(msg.velocity, dtype=np.float64)
        effort = np.asarray(msg.effort, dtype=np.float64)
        stamp = HexDcBaseTime(
            secs=int(msg.header.stamp.sec),
            nsecs=int(msg.header.stamp.nanosec),
        )
        self.set_bridge_state(names, position, velocity, effort, stamp)

    @staticmethod
    def __manip_ctrl_msg_to_dc(
            msg: HexRosRoboManipCtrlStamped) -> HexDcRoboManipCtrlStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.sec),
                nsecs=int(msg.header.stamp.nanosec),
            ),
            frame_id=msg.header.frame_id,
        )

        arm_msg = msg.manip_ctrl.arm_ctrl
        arm_ctrl = HexDcRoboArmCtrl(
            ctrl_mode=HexDcRoboArmCtrlMode(int(arm_msg.ctrl_mode)),
            grav=HexDcBaseVector3(
                x=arm_msg.grav.x,
                y=arm_msg.grav.y,
                z=arm_msg.grav.z,
            ),
            jnt=DataInterface.__jnt_to_dc(arm_msg.jnt),
            pose=DataInterface.__pose_to_dc(arm_msg.pose),
        )

        grip_msg = msg.manip_ctrl.grip_ctrl
        grip_ctrl = HexDcRoboGripCtrl(
            ctrl_mode=HexDcRoboGripCtrlMode(int(grip_msg.ctrl_mode)),
            jnt=DataInterface.__jnt_to_dc(grip_msg.jnt),
        )

        return HexDcRoboManipCtrlStamped(
            header=header,
            manip_ctrl=HexDcRoboManipCtrl(
                arm_ctrl=arm_ctrl,
                grip_ctrl=grip_ctrl,
            ),
        )

    @staticmethod
    def __jnt_to_dc(jnt: HexRosJnt) -> HexDcBaseJntFull:
        return HexDcBaseJntFull(
            pos=np.asarray(jnt.pos, dtype=np.float64),
            vel=np.asarray(jnt.vel, dtype=np.float64),
            eff=np.asarray(jnt.eff, dtype=np.float64),
            kp=np.asarray(jnt.kp, dtype=np.float64),
            kd=np.asarray(jnt.kd, dtype=np.float64),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64),
        )

    @staticmethod
    def __pose_to_dc(pose: Pose) -> HexDcBasePose:
        return HexDcBasePose(
            position=HexDcBaseVector3(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
            ),
            orientation=HexDcBaseQuaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
        )
