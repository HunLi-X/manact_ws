#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 人形机器人手臂控制脚本
=========================
通过 unitree_sdk2py 的 DDS 通道直接控制 G1 机器人手臂关节。
本脚本不依赖 ROS2，通过底层 DDS 通信与机器人交互。

功能演示：手臂抬起 → 保持 → 放下 → 释放 arm_sdk 控制权

运行方式：
    python3 src/arm.py              # 自动检测网络接口
    python3 src/arm.py eth0         # 指定网络接口

依赖：
    pip install unitree_sdk2py numpy
"""

import time
import sys
import threading

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# 从共享模块导入关节常量和限位
from arm_common import G1JointIndex, ARM_JOINTS, JOINT_LIMITS

# ---- 常量 ----
kPi = 3.141592654
kPi_2 = 1.57079632


class Custom:
    """G1 手臂控制演示类。

    实现四阶段手臂运动序列：
    1. 归零阶段 — 将手臂从当前位置平滑插值到零位
    2. 抬臂阶段 — 将手臂平滑抬起至目标姿态
    3. 归位阶段 — 将手臂平滑放回零位
    4. 释放阶段 — 逐步释放 arm_sdk 控制权，交还给机器人固件
    """

    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02
        self.duration_ = 3.0

        self.kp = 60.
        self.kd = 1.5

        self._state_lock = threading.Lock()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None

        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False

        # 手臂目标角度（单位：弧度）
        self.target_pos = [
            0.0,      kPi_2,  0.0,    kPi_2,  0.0,
            0.0,     -kPi_2,  0.0,    kPi_2,  0.0,
            0.0,      0.0,
        ]

    def _clip_joint(self, joint, value):
        """将关节角度限制在安全范围内。"""
        lo, hi = JOINT_LIMITS.get(joint, (-3.14, 3.14))
        return float(np.clip(value, lo, hi))

    def Init(self):
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        _t0 = time.time()
        while self.first_update_low_state == False:
            if time.time() - _t0 > 10.0:
                raise TimeoutError("未收到关节状态消息，请检查 DDS 通信和网络接口")
            time.sleep(0.1)

        if self.first_update_low_state == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        with self._state_lock:
            self.low_state = msg
        if self.first_update_low_state == False:
            self.first_update_low_state = True

    def LowCmdWrite(self):
        with self._state_lock:
            if self.low_state is None:
                return

        self.time_ += self.control_dt_

        if self.time_ < self.duration_:
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
            for i, joint in enumerate(ARM_JOINTS):
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                self.low_cmd.motor_cmd[joint].tau = 0.
                with self._state_lock:
                    cur_q = self.low_state.motor_state[joint].q
                self.low_cmd.motor_cmd[joint].q = self._clip_joint(joint, (1.0 - ratio) * cur_q)
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 3:
            for i, joint in enumerate(ARM_JOINTS):
                ratio = np.clip((self.time_ - self.duration_) / (self.duration_ * 2), 0.0, 1.0)
                self.low_cmd.motor_cmd[joint].tau = 0.
                with self._state_lock:
                    cur_q = self.low_state.motor_state[joint].q
                self.low_cmd.motor_cmd[joint].q = self._clip_joint(joint, ratio * self.target_pos[i] + (1.0 - ratio) * cur_q)
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 6:
            for i, joint in enumerate(ARM_JOINTS):
                ratio = np.clip((self.time_ - self.duration_*3) / (self.duration_ * 3), 0.0, 1.0)
                self.low_cmd.motor_cmd[joint].tau = 0.
                with self._state_lock:
                    cur_q = self.low_state.motor_state[joint].q
                self.low_cmd.motor_cmd[joint].q = self._clip_joint(joint, (1.0 - ratio) * cur_q)
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 7:
            for i, joint in enumerate(ARM_JOINTS):
                ratio = np.clip((self.time_ - self.duration_*6) / (self.duration_), 0.0, 1.0)
                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = (1 - ratio)

        else:
            self.done = True

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)

if __name__ == '__main__':
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:
        time.sleep(1)
        if custom.done:
            print("Done!")
            sys.exit(0)
