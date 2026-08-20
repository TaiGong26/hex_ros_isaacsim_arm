#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Test controller – cycles manipulator through position presets."""

import os
import sys
import time

import numpy as np

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboArmCtrl,
    HexDcRoboArmCtrlMode as ArmCtrlMode,
    HexDcRoboGripCtrl,
    HexDcRoboGripCtrlMode as GripCtrlMode,
    HexDcRoboManipCtrl,
)

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from test_utils import DataInterface

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ARM_DOF = 6
GRIP_DOF = 1
CYCLE_PERIOD = 5.0

# Arm position presets (MIT / JNT modes)
ARM_POS_PRESETS = [
    [0.0, -1.5, 3.0, 0.0, 0.0, 0.0],
    [0.5, -1.0, 1.5, -0.5, 0.5, 0.0],
    [-0.5, 0.5, 1.57, -0.5, 0.5, 0.0],
]

# End-effector pose presets (EE mode): ([x,y,z], [qw,qx,qy,qz])
ARM_EE_PRESETS = [
    ([0.3, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0]),
    ([0.3, 0.2, 0.3], [1.0, 0.0, 0.0, 0.0]),
]

# Grip presets
GRIP_POS_PRESETS = [[0.0], [0.5]]
GRIP_EFF_PRESETS = [[-1.0], [1.0]]

# Gains
ARM_KP   = [200.0, 200.0, 200.0, 200.0, 100.0, 100.0]
ARM_KD   = [5.0, 5.0, 5.0, 5.0, 2.0, 2.0]
ARM_VLIM = [10.0] * ARM_DOF
ARM_ALIM = [10.0] * ARM_DOF

ARM_MIT_KP   = [0.0] * ARM_DOF
ARM_MIT_KD   = [0.0] * ARM_DOF
ARM_MIT_VLIM = [0.0] * ARM_DOF
ARM_MIT_ALIM = [0.0] * ARM_DOF

GRIP_KP   = [10.0]
GRIP_KD   = [0.5]
GRIP_VLIM = [0.5]
GRIP_ALIM = [1.0]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _build_joint_full(pos, vel, eff, kp, kd, vlim, alim):
    """Build HexDcBaseJntFull from plain Python lists."""
    return HexDcBaseJntFull(
        pos=np.asarray(pos, dtype=np.float64),
        vel=np.asarray(vel, dtype=np.float64),
        eff=np.asarray(eff, dtype=np.float64),
        kp=np.asarray(kp, dtype=np.float64),
        kd=np.asarray(kd, dtype=np.float64),
        lim_vel=np.asarray(vlim, dtype=np.float64),
        lim_acc=np.asarray(alim, dtype=np.float64),
    )

def _zero_vector(n):
    """Zero vector of length n."""
    return np.zeros(n, dtype=np.float64)

def _gravity_vector(z=-9.81):
    return HexDcBaseVector3(0.0, 0.0, z)

def _zero_pose():
    return HexDcBasePose(
        position=HexDcBaseVector3(0.0, 0.0, 0.0),
        orientation=HexDcBaseQuaternion(0.0, 0.0, 0.0, 1.0),
    )

# ---------------------------------------------------------------------------
# Control builder
# ---------------------------------------------------------------------------

def build_manip_ctrl(control_mode: str, cycle_index: int, use_gravity: bool = True) -> HexDcRoboManipCtrl:
    """Assemble a manipulator control message.
    
    control_mode -  'mit' | 'jnt' | 'ee'
    cycle_index  - step counter, drives preset cycling (mod 3)
    use_gravity  - include gravity compensation on z axis
    """
    idx = cycle_index % 3
    gravity = _gravity_vector(-9.81 if use_gravity else 0.0)
    z6, z1 = _zero_vector(ARM_DOF), _zero_vector(GRIP_DOF)

    if control_mode == 'mit':
        arm = HexDcRoboArmCtrl(
            ctrl_mode=ArmCtrlMode.MIT,
            grav=gravity,
            jnt=_build_joint_full(
                ARM_POS_PRESETS[0], 
                z6, 
                z6,
                ARM_MIT_KP, 
                ARM_MIT_KD, 
                ARM_MIT_VLIM, 
                ARM_MIT_ALIM
            ),
            pose=_zero_pose(),
        )
        grip = HexDcRoboGripCtrl(
            ctrl_mode=GripCtrlMode.MIT,
            jnt=_build_joint_full(
                GRIP_POS_PRESETS[idx%2], 
                z1, 
                z1,
                GRIP_KP, GRIP_KD, 
                GRIP_VLIM, 
                GRIP_ALIM
            ),
        )

    elif control_mode == 'jnt':
        arm = HexDcRoboArmCtrl(
            ctrl_mode=ArmCtrlMode.JNT,
            grav=gravity,
            jnt=_build_joint_full(
                ARM_POS_PRESETS[idx], 
                z6, 
                z6,
                ARM_KP, 
                ARM_KD, 
                ARM_VLIM, 
                ARM_ALIM
            ),
            pose=_zero_pose(),
        )
        grip = HexDcRoboGripCtrl(
            ctrl_mode=GripCtrlMode.JNT,
            jnt=_build_joint_full(
                z1, 
                z1, 
                GRIP_EFF_PRESETS[idx%2],
                GRIP_KP, 
                GRIP_KD, 
                GRIP_VLIM, 
                GRIP_ALIM
            ),
        )

    elif control_mode == 'ee':
        pos, ori = ARM_EE_PRESETS[idx%2]
        arm = HexDcRoboArmCtrl(
            ctrl_mode=ArmCtrlMode.EE,
            grav=gravity,
            jnt=_build_joint_full(
                z6, 
                z6, 
                z6, 
                ARM_KP, 
                ARM_KD, 
                ARM_VLIM, 
                ARM_ALIM
            ),
            pose=HexDcBasePose(
                position=HexDcBaseVector3(
                    x = pos[0], 
                    y = pos[1], 
                    z = pos[2]
                ),
                orientation=HexDcBaseQuaternion(
                    x = ori[1], 
                    y = ori[2], 
                    z = ori[3], 
                    w = ori[0]
                ),
            ),
        )
        grip = HexDcRoboGripCtrl(
            ctrl_mode=GripCtrlMode.TAU,
            jnt=_build_joint_full(
                z1, 
                z1, 
                GRIP_EFF_PRESETS[idx%2],
                GRIP_KP, 
                GRIP_KD, 
                GRIP_VLIM, 
                GRIP_ALIM
            ),
        )

    else:
        raise ValueError(f"Unknown mode: {control_mode}")

    return HexDcRoboManipCtrl(arm_ctrl=arm, grip_ctrl=grip)

# ---------------------------------------------------------------------------
# Test node
# ---------------------------------------------------------------------------

class ManipulatorTestNode:
    def __init__(self):
        self.data_interface = DataInterface("test_ctrl")
        self.rate_params = self.data_interface.get_rate_param()
        self.ros_frequency = self.rate_params["ros"]

        self.cycle_decimation = max(1, int(round(CYCLE_PERIOD * self.ros_frequency)))
        self.control_decimation = max(1, int(round(self.ros_frequency / self.rate_params["ctrl"])))
        self.control_mode = self.data_interface.get_ctrl_mode()  # 'mit'|'jnt'|'ee'

        self.data_interface.logi(f"[test_ctrl] mod {self.control_mode}")
        
    def run(self):
        
        loop_counter = 0
        control_publish_counter = 0
        state_receive_counter = 0
        last_report_time = time.monotonic()
        
        while self.data_interface.ok():
            control_publish_counter += 1
            if control_publish_counter >= self.control_decimation:
                control_publish_counter = 0
                cycle_index = loop_counter // self.cycle_decimation
                control_command = build_manip_ctrl(self.control_mode, cycle_index, use_gravity=True)
                self.data_interface.pub_manip_ctrl(control_command)
            
            while self.data_interface.get_manip_state() is not None:
                state_receive_counter += 1

            now = time.monotonic()
            elapsed = now - last_report_time
            if elapsed >= 1.0:
                self.data_interface.logi(f"Manipulator state receive frequency: {state_receive_counter / elapsed:.1f} Hz")
                state_receive_counter = 0
                last_report_time = now

            loop_counter += 1
            self.data_interface.sleep()
        
    def shutdown(self):
        try:
            self.data_interface.shutdown()
        except Exception:
            pass

def main():
    node = ManipulatorTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[test node] error: {e}")
    finally:
        node.shutdown()

if __name__ == '__main__':
    main()