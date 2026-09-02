#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-08-31
################################################################
"""Forward Archer controls to the Isaac Sim implicit PD controller."""

import os
import sys

import numpy as np

script_path = os.path.abspath(os.path.dirname(__file__))
if script_path not in sys.path:
    sys.path.append(script_path)

from utility import DataInterface
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrlMode, HexDcRoboGripCtrlMode,
)
from hex_util_ros import HexDynUtilY6


ARM_DOF = 6
GRIP_TORQUE_LIMIT = 40.0
INTERPOLATION_DT = 0.002
GRIP_TYPE_DOF = {"empty": 0, "gr100": 2}


class IsaacsimArcherY6:
    """Convert supported Archer controls into Bridge joint commands.

    The arm accepts JNT and EE targets.  The gripper accepts NONE, JNT, and
    TAU targets; position commands are interpolated and TAU effort is limited.
    """

    def __init__(self) -> None:
        """Initialize the Archer transport, model, and command state."""
        self.__data_interface = DataInterface("hex_ros_isaacsim_archer_y6")

        rate_param = self.__data_interface.get_rate_param()
        robot_param = self.__data_interface.get_robot_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")
        self.__data_interface.logi(
            f"robot_grip_type: {robot_param['grip_type']}")

        # The interpolation step is fixed at 2 ms by the current command
        # protocol rather than derived from the ROS loop rate.
        self.__grip_dof = GRIP_TYPE_DOF.get(robot_param["grip_type"], 0)
        # Total command size is six arm joints plus the configured gripper.
        self.__dof = ARM_DOF + self.__grip_dof
        # Store the latest command position used as the interpolation origin.
        self.__last_position = np.zeros(self.__dof)
        # Retain the newest accepted command between control cycles.
        self.__cur_ctrl = None
        # Prevent repeated warnings when Bridge state is unavailable.
        self.__bridge_warning_sent = False
        # Prevent repeated warnings when no manipulator command is available.
        self.__control_warning_sent = False
        self.__dyn_util = self.__load_dyn(robot_param["urdf_path"])

    def __load_dyn(self, urdf_path: str):
        """Load the Archer model used by EE inverse kinematics.

        Args:
            urdf_path: Path to the Archer URDF model.

        Returns:
            The dynamics utility, or ``None`` when the model cannot be loaded.

        """
        if not urdf_path:
            self.__data_interface.logw(
                "Archer urdf_path is empty; EE control is disabled")
            return None
        try:
            return HexDynUtilY6(
                model_path=urdf_path,
                last_link="link_6",
                pose_end_in_flange=np.array(
                    [0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
        except Exception as exc:
            self.__data_interface.loge(
                f"failed to load Archer URDF: {exc}")
            return None

    @staticmethod
    def __array(values, size: int, default: float = 0.0) -> np.ndarray:
        """Return a joint array, expanding a scalar value when needed.

        Args:
            values: Input joint values.
            size: Required output length.
            default: Fill value for an invalid input length.

        Returns:
            A floating-point array with exactly ``size`` elements.

        """
        array = np.asarray(values, dtype=np.float64)
        if array.size == size:
            return array
        if array.size == 1:
            return np.full(size, array[0], dtype=np.float64)
        return np.full(size, default, dtype=np.float64)

    def __interpolate(self, last, target, lim_vel):
        """Move each joint toward a target by one velocity-limited step.

        Args:
            last: Position command from the previous control cycle.
            target: Desired joint position.
            lim_vel: Per-joint velocity limits in radians per second.

        Returns:
            The interpolated position command for the current cycle.

        """
        target = np.asarray(target, dtype=np.float64)
        limit = np.abs(np.asarray(lim_vel, dtype=np.float64))
        delta = target - last
        # Limit travel to the configured velocity multiplied by 2 ms.
        position = last + np.sign(delta) * np.minimum(
            np.abs(delta), limit * INTERPOLATION_DT)
        return position

    def __arm_command(self, arm_ctrl):
        """Build the arm position and effort arrays.

        Args:
            arm_ctrl: Shared arm control structure.

        Returns:
            A position/effort tuple, or ``None`` for an unsupported or failed
            arm command.

        """
        mode = arm_ctrl.ctrl_mode
        effort = self.__array(arm_ctrl.jnt.eff, ARM_DOF)

        if mode == HexDcRoboArmCtrlMode.JNT:
            # JNT already supplies the desired arm joint positions.
            target = self.__array(arm_ctrl.jnt.pos, ARM_DOF)
        elif mode == HexDcRoboArmCtrlMode.EE:
            # EE targets require analytical IK before position interpolation.
            if self.__dyn_util is None:
                self.__data_interface.logw("Archer urdf not loaded; EE control is disabled")
                return None
            pose = arm_ctrl.pose
            position = np.array([
                pose.position.x, pose.position.y, pose.position.z])
            orientation = np.array([
                pose.orientation.w, pose.orientation.x,
                pose.orientation.y, pose.orientation.z])
            # Use the previous arm command as the IK seed to reduce solution jumps.
            success, target = self.__dyn_util.inverse_kinematics_analytic(
                (position, orientation), self.__last_position[:ARM_DOF])
            if not success:
                self.__data_interface.logw("Archer inverse kinematics failed")
                return None
        else:
            self.__data_interface.logw(
                "Archer arm control mode is unsupported; use JNT or EE")
            return None

        limit = self.__array(arm_ctrl.jnt.lim_vel, ARM_DOF, np.inf)
        # Interpolate both JNT and EE results to avoid an instantaneous arm jump.
        arm_position = self.__interpolate(
            self.__last_position[:ARM_DOF], target, limit)
        return arm_position, effort

    def __grip_command(self, grip_ctrl):
        """Build gripper position and effort arrays.

        Args:
            grip_ctrl: Shared gripper control structure.

        Returns:
            A position/effort tuple, or ``None`` for an unsupported command.

        """
        if self.__grip_dof == 0:
            empty = np.zeros(0)
            return empty, empty

        last = self.__last_position[ARM_DOF:]
        mode = grip_ctrl.ctrl_mode
        effort = self.__array(grip_ctrl.jnt.eff, self.__grip_dof)
        if mode == HexDcRoboGripCtrlMode.NONE:
            # NONE holds the last position without applying gripper torque.
            return last, np.zeros(self.__grip_dof)
        if mode == HexDcRoboGripCtrlMode.MIT:
            self.__data_interface.logw(
                "Archer gripper MIT control is not supported")
            return None
        if mode == HexDcRoboGripCtrlMode.TAU:
            # TAU forwards torque only after applying the 40 N*m safety limit.
            effort = np.clip(effort, -GRIP_TORQUE_LIMIT, GRIP_TORQUE_LIMIT)
            return last, effort
        if mode != HexDcRoboGripCtrlMode.JNT:
            self.__data_interface.logw(
                "Archer gripper control mode is unsupported; "
                "use NONE, JNT, or TAU")
            return None

        target = self.__array(grip_ctrl.jnt.pos, self.__grip_dof)
        limit = self.__array(
            grip_ctrl.jnt.lim_vel, self.__grip_dof, np.inf)
        # Interpolate gripper position to avoid an instantaneous target jump.
        grip_position = self.__interpolate(last, target, limit)
        return grip_position, effort

    def __apply_manip_ctrl(self, ctrl):
        """Convert one manipulator control into a Bridge command.

        Args:
            ctrl: Shared manipulator control structure.

        Returns:
            A complete position/effort command, or ``None`` if either
            subsystem rejects the control mode.

        """
        arm = self.__arm_command(ctrl.manip_ctrl.arm_ctrl)
        grip = self.__grip_command(ctrl.manip_ctrl.grip_ctrl)
        if arm is None or grip is None:
            return None

        # Combine arm and gripper outputs into one Bridge command frame.
        position = np.concatenate((arm[0], grip[0]))
        effort = np.concatenate((arm[1], grip[1]))
        # Use the generated command as the next interpolation origin.
        self.__last_position = position
        return position, effort

    def run(self) -> None:
        """Run the fixed-rate command-forwarding loop.

        Each cycle consumes the newest command, validates the Bridge joint
        interface, generates one position/effort frame, publishes it, and
        sleeps for the configured control period.
        """
        while self.__data_interface.ok():
            # Consume only the newest frame so stale targets are not applied.
            ctrl = self.__data_interface.get_manip_ctrl(latest=True)
            if ctrl is not None:
                # NONE clears the retained command; other modes replace it.
                self.__data_interface.logd(
                    f"received manip control: "
                    f"{ctrl.manip_ctrl.arm_ctrl.ctrl_mode}")
                if ctrl.manip_ctrl.arm_ctrl.ctrl_mode == \
                        HexDcRoboArmCtrlMode.NONE:
                    self.__cur_ctrl = None
                else:
                    self.__cur_ctrl = ctrl

            # Bridge names define the order of every published joint field.
            joint_names = self.__data_interface.get_joint_names()
            if not joint_names:
                if not self.__bridge_warning_sent:
                    self.__data_interface.logw(
                        "ROS bridge joint_states not received; "
                        "command forwarding is disabled")
                    self.__bridge_warning_sent = True
                self.__data_interface.sleep()
                continue

            if len(joint_names) != self.__dof:
                if not self.__bridge_warning_sent:
                    self.__data_interface.logw(
                        f"expected {self.__dof} bridge joints, "
                        f"received {len(joint_names)}")
                    self.__bridge_warning_sent = True
                self.__data_interface.sleep()
                continue

            if self.__cur_ctrl is not None:
                # Convert the retained high-level command for this cycle.
                command = self.__apply_manip_ctrl(self.__cur_ctrl)
                if command is not None:
                    # Publish position and effort as one consistent command.
                    self.__data_interface.pub_joint_command(
                        joint_names, *command)
                    self.__data_interface.logd("published joint command")
                else:
                    self.__data_interface.logw(
                        "Archer control was rejected; no command published")
            elif not self.__control_warning_sent:
                self.__data_interface.logw(
                    "Archer manip_ctrl not received; command forwarding is idle")
                self.__control_warning_sent = True
            self.__data_interface.sleep()

    def shutdown(self) -> None:
        """Stop the ROS interface and release its resources."""
        self.__data_interface.shutdown()

def main() -> None:
    """Create and run the Archer Bridge forwarder."""
    robot_archer_y6 = IsaacsimArcherY6()
    try:
        robot_archer_y6.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_archer_y6.shutdown()


if __name__ == "__main__":
    main()
