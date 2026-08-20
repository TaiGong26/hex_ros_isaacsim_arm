#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################
# Isaac Sim archer bridge-forwarding node.
#
# Unlike the real-robot node this is a pure forwarder: it subscribes to the
# bridge joint_states, computes the control torque locally (mirroring the
# mujoco sim package torque laws, including gravity/friction compensation)
# and publishes a single torque command back to the bridge. No hardware.

import os
import sys
from typing import Optional

import numpy as np

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntState,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrlMode,
    HexDcRoboArmState,
    HexDcRoboGripCtrlMode,
    HexDcRoboGripState,
    HexDcRoboManipCtrlStamped,
    HexDcRoboManipState,
    HexDcRoboManipStateStamped,
)
from hex_util_ros import (
    HexDynUtilY6,
    HexFricUtil,
    arm_pos_limit,
    grip_pos_limit,
    interp_joint,
)

scrpit_path = os.path.abspath(os.path.dirname(__file__))
if scrpit_path not in sys.path:
    sys.path.append(scrpit_path)
from utility import DataInterface

# Default gains used when only a position target is given (JNT mode),
# mirrored from hex_ros_sim_archer_y6/mujoco_sim.py.
ARM_KP_DEFAULT = np.array([400.0, 400.0, 500.0, 200.0, 100.0, 100.0])
ARM_KD_DEFAULT = np.array([5.0, 5.0, 5.0, 5.0, 2.0, 2.0])
GRIP_KP_DEFAULT = np.array([10.0])
GRIP_KD_DEFAULT = np.array([0.5])

# archer friction model, mirrored from mujoco_sim.py (4 base + 2 distal).
ARM_FRIC_FC = np.array([0.28] * 4 + [0.05] * 2)
ARM_FRIC_FV = np.array([0.04] * 4 + [0.0015] * 2)

# grip joint range from the mujoco archer model (gr100).
GRIP_LIMIT_LOWER = np.array([0.0])
GRIP_LIMIT_UPPER = np.array([0.6])


class IsaacsimArcherY6:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("hex_ros_isaacsim_archer_y6")

        ### parameters
        rate_param = self.__data_interface.get_rate_param()
        robot_param = self.__data_interface.get_robot_param()
        bridge_param = self.__data_interface.get_bridge_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")
        self.__data_interface.logi(f"rate_state: {rate_param['state']} hz")
        self.__data_interface.logi(f"robot_frame_id: {robot_param['frame_id']}")
        self.__data_interface.logi(f"robot_grip_type: {robot_param['grip_type']}")
        self.__data_interface.logi(f"bridge_state_topic: {bridge_param['state_topic']}")
        self.__data_interface.logi(f"bridge_cmd_topic: {bridge_param['cmd_topic']}")
        self.__data_interface.logi(f"time_source: {bridge_param['time_source']}")

        ### derived
        grip_dof = 1 if robot_param["grip_type"] != "empty" else 0
        self.__data_interface.set_dofs(arm_dof=6, grip_dof=grip_dof)
        self.__grip_dof = grip_dof
        self.__state_decim = max(
            1,
            int(round(rate_param["ros"] / rate_param["state"])),
        )
        self.__robot_frame_id = robot_param["frame_id"]

        ### dynamics (compensation / FK / IK), mirrors mujoco_sim
        urdf_path = robot_param["urdf_path"]
        if not urdf_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                urdf_path = os.path.join(
                    get_package_share_directory("hex_ros_urdf_archer_y6"),
                    "urdf", "empty.urdf")
            except Exception:
                self.__data_interface.loge(
                    "urdf_path is empty and hex_ros_urdf_archer_y6 not found")
        self.__dyn_util = HexDynUtilY6(
            model_path=urdf_path,
            last_link="link_6",
            pose_end_in_flange=np.array([0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
        )
        self.__fric_util = HexFricUtil(
            fc=ARM_FRIC_FC,
            fv=ARM_FRIC_FV,
            fo=np.zeros(6),
            k=np.full(6, 100.0),
        )

        # arm joint limits from the URDF (mirror mujoco jnt_range)
        arm_lower, arm_upper = self.__dyn_util.get_limit()
        self.__arm_lower = np.asarray(arm_lower, dtype=np.float64)
        self.__arm_upper = np.asarray(arm_upper, dtype=np.float64)

        ### current control (seeded with a home MIT hold)
        self.__cur_ctrl = None

    def __gravity(self, arm_ctrl) -> np.ndarray:
        return np.array([
            arm_ctrl.grav.x,
            arm_ctrl.grav.y,
            arm_ctrl.grav.z,
        ])

    def __apply_arm_ctrl(self, arm_ctrl, cur_pos, cur_vel) -> np.ndarray:
        """Arm torque in canonical arm order; mirrors mujoco __apply_arm_ctrl."""
        ctrl_jnt = arm_ctrl.jnt
        mode = arm_ctrl.ctrl_mode
        comp = self.__dyn_util.compensation(cur_pos, cur_vel) \
            + self.__fric_util(cur_vel)

        if mode == HexDcRoboArmCtrlMode.MIT:
            tar_pos = arm_pos_limit(ctrl_jnt.pos, self.__arm_lower,
                                    self.__arm_upper)
            tau_cmds = ctrl_jnt.kp * (
                tar_pos - cur_pos) + ctrl_jnt.kd * (
                    ctrl_jnt.vel - cur_vel) + ctrl_jnt.eff + comp

        elif mode == HexDcRoboArmCtrlMode.JNT:
            tar_pos = arm_pos_limit(ctrl_jnt.pos, self.__arm_lower,
                                    self.__arm_upper)
            mid_pos = interp_joint(
                cur_pos,
                tar_pos,
                # not a right formulation, just for test (mirrors mujoco)
                err_limit=self.__lim_vel_or(ctrl_jnt.lim_vel) * 3e-3,
            )
            tau_cmds = ARM_KP_DEFAULT * (
                mid_pos - cur_pos) + ARM_KD_DEFAULT * (
                    ctrl_jnt.vel - cur_vel) + ctrl_jnt.eff + comp

        elif mode == HexDcRoboArmCtrlMode.EE:
            pos = np.array([
                arm_ctrl.pose.position.x,
                arm_ctrl.pose.position.y,
                arm_ctrl.pose.position.z,
            ])
            ori = np.array([
                arm_ctrl.pose.orientation.w,
                arm_ctrl.pose.orientation.x,
                arm_ctrl.pose.orientation.y,
                arm_ctrl.pose.orientation.z,
            ])
            ik_success, tar_pos = self.__dyn_util.inverse_kinematics_analytic(
                (pos, ori), cur_pos)
            if not ik_success:
                self.__data_interface.logw("Inverse kinematics failed")
            tar_pos = arm_pos_limit(tar_pos, self.__arm_lower,
                                    self.__arm_upper)
            mid_pos = interp_joint(
                cur_pos,
                tar_pos,
                err_limit=self.__lim_vel_or(ctrl_jnt.lim_vel) * 3e-3,
            )
            tau_cmds = ARM_KP_DEFAULT * (
                mid_pos - cur_pos) + ARM_KD_DEFAULT * (
                    ctrl_jnt.vel - cur_vel) + ctrl_jnt.eff + comp

        elif mode == HexDcRoboArmCtrlMode.NONE:
            tau_cmds = np.zeros(6)

        else:
            raise ValueError(f"Unsupported arm control mode: {mode}")

        return tau_cmds

    def __apply_grip_ctrl(self, grip_ctrl, cur_pos, cur_vel) -> np.ndarray:
        """Grip torque; mirrors mujoco __apply_grip_ctrl."""
        ctrl_jnt = grip_ctrl.jnt
        mode = grip_ctrl.ctrl_mode
        tau_cmds = np.zeros(1)

        if mode == HexDcRoboGripCtrlMode.MIT:
            tar_pos = grip_pos_limit(ctrl_jnt.pos, GRIP_LIMIT_LOWER,
                                     GRIP_LIMIT_UPPER)
            tau_cmds = ctrl_jnt.kp * (
                tar_pos - cur_pos) + ctrl_jnt.kd * (
                    ctrl_jnt.vel - cur_vel) + ctrl_jnt.eff

        elif mode == HexDcRoboGripCtrlMode.JNT:
            tar_pos = grip_pos_limit(ctrl_jnt.pos, GRIP_LIMIT_LOWER,
                                     GRIP_LIMIT_UPPER)
            tau_abs = np.fabs(ctrl_jnt.eff)
            lim_vel = self.__grip_lim_vel(ctrl_jnt.lim_vel)
            kd = tau_abs / lim_vel
            pos_err = tar_pos - cur_pos
            grip_tau = np.sign(pos_err) * tau_abs - kd * cur_vel
            tau_cmds = np.clip(np.fabs(pos_err * 1e1), 0.0, 1.0) * grip_tau

        elif mode == HexDcRoboGripCtrlMode.TAU:
            lim_vel = self.__grip_lim_vel(ctrl_jnt.lim_vel)
            kd = np.fabs(ctrl_jnt.eff / lim_vel)
            tau_cmds = ctrl_jnt.eff - kd * cur_vel

        elif mode == HexDcRoboGripCtrlMode.NONE:
            tau_cmds = np.zeros(1)

        else:
            raise ValueError(f"Unsupported gripper control mode: {mode}")

        return tau_cmds

    def __lim_vel_or(self, lim_vel) -> np.ndarray:
        if lim_vel is not None and len(lim_vel) == 6:
            return np.asarray(lim_vel, dtype=np.float64)
        return np.full(6, 1.0)

    def __grip_lim_vel(self, lim_vel) -> float:
        if lim_vel is not None and len(lim_vel) > 0:
            v = float(np.asarray(lim_vel, dtype=np.float64)[0])
            return v if v != 0.0 else 0.5
        return 0.5

    def __compute_tau(self, ctrl: HexDcRoboManipCtrlStamped) -> np.ndarray:
        """Single torque array [arm(6) + grip(grip_dof)]."""
        arm_ctrl = ctrl.manip_ctrl.arm_ctrl
        grip_ctrl = ctrl.manip_ctrl.grip_ctrl

        arm_pos, arm_vel, _ = self.__data_interface.get_arm_state()
        tau_arm = self.__apply_arm_ctrl(arm_ctrl, arm_pos, arm_vel)

        if self.__grip_dof > 0:
            grip_pos, grip_vel, _ = self.__data_interface.get_grip_state()
            tau_grip = self.__apply_grip_ctrl(grip_ctrl, grip_pos, grip_vel)
        else:
            tau_grip = np.zeros(0)

        return np.concatenate([tau_arm, tau_grip])

    def __build_manip_state(self) -> Optional[HexDcRoboManipStateStamped]:
        arm_pos, arm_vel, arm_eff = self.__data_interface.get_arm_state()

        # ee pose from the current arm state (mirror mujoco get_manip_state)
        ee_pos, ee_quat = self.__dyn_util.forward_kinematics(arm_pos)[-1]
        arm_pose = HexDcBasePose(
            position=HexDcBaseVector3(
                x=float(ee_pos[0]),
                y=float(ee_pos[1]),
                z=float(ee_pos[2]),
            ),
            orientation=HexDcBaseQuaternion(
                x=float(ee_quat[1]),
                y=float(ee_quat[2]),
                z=float(ee_quat[3]),
                w=float(ee_quat[0]),
            ),
        )

        arm_state = HexDcRoboArmState(
            jnt=HexDcBaseJntState(
                position=arm_pos,
                velocity=arm_vel,
                effort=arm_eff,
            ),
            pose=arm_pose,
        )

        if self.__grip_dof > 0:
            grip_pos, grip_vel, grip_eff = self.__data_interface.get_grip_state()
            grip_state = HexDcRoboGripState(
                jnt=HexDcBaseJntState(
                    position=grip_pos,
                    velocity=grip_vel,
                    effort=grip_eff,
                ),
            )
        else:
            grip_state = HexDcRoboGripState(jnt=HexDcBaseJntState())

        return HexDcRoboManipStateStamped(
            header=HexDcBaseHeader(
                stamp=self.__data_interface.get_forward_stamp(),
                frame_id=self.__robot_frame_id,
            ),
            manip_state=HexDcRoboManipState(
                arm_state=arm_state,
                grip_state=grip_state,
            ),
        )

    def run(self):
        state_count = 0
        while self.__data_interface.ok():
            # 1. drain to the latest control frame
            ctrl = self.__data_interface.get_manip_ctrl(latest=True)
            if ctrl is not None:
                self.__cur_ctrl = ctrl

            # 2. publish a single torque to the bridge at ctrl_rate (1000 Hz)
            if self.__cur_ctrl is not None:
                tau_cmds = self.__compute_tau(self.__cur_ctrl)
                cmd_names = (self.__data_interface.get_arm_joint_names()
                             + self.__data_interface.get_grip_joint_names())
                self.__data_interface.pub_bridge_cmd(cmd_names, tau_cmds)

            # 3. forward manip_state at rate_state (500 Hz)
            state_count += 1
            if state_count >= self.__state_decim:
                state_count = 0
                manip_state = self.__build_manip_state()
                if manip_state is not None:
                    self.__data_interface.pub_manip_state(manip_state)

            self.__data_interface.sleep()

    def shutdown(self):
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass


def main():
    robot_archer_y6 = IsaacsimArcherY6()
    try:
        robot_archer_y6.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_archer_y6.shutdown()


if __name__ == '__main__':
    main()
