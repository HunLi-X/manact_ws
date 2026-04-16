#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 手臂抓取控制 (armup)
========================
控制机器人手臂执行抓取动作：伸出 → 抬起 → 夹紧保持。

姿态序列：
  1. reach_forward — 伸手接近目标
  2. arms_up       — 抬起目标
  3. pray          — 夹紧保持（不释放 arm_sdk）

运行：
    python3 armup.py           # 自动检测网络接口
    python3 armup.py eth0      # 指定网络接口

依赖：
    pip install unitree_sdk2py numpy
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import time  # 控制循环计时与等待
import sys   # 命令行参数读取（网络接口名）

# ==================================================================
# 2. 第三方库导入
# ==================================================================
import numpy as np  # 数值计算，用于关节角度线性插值
# unitree_sdk2py: 宇树机器人 DDS 通信底层 SDK
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize  # DDS 发布者与工厂初始化
from unitree_sdk2py.core.channel import ChannelSubscriber  # DDS 订阅者，接收关节状态
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_  # 低层指令消息默认构造
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_  # 低层指令/状态 IDL 消息类型
from unitree_sdk2py.utils.crc import CRC  # CRC 校验，G1 固件要求每帧指令附带
from unitree_sdk2py.utils.thread import RecurrentThread  # 定时回调线程，用于 50Hz 控制循环

kPi = 3.141592654
kPi_2 = 1.57079632


class G1JointIndex:
    """G1 机器人关节索引常量（0-29）。"""
    LeftHipPitch = 0;  LeftHipRoll = 1;  LeftHipYaw = 2
    LeftKnee = 3;  LeftAnklePitch = 4;  LeftAnkleRoll = 5
    RightHipPitch = 6;  RightHipRoll = 7;  RightHipYaw = 8
    RightKnee = 9;  RightAnklePitch = 10;  RightAnkleRoll = 11
    WaistYaw = 12;  WaistRoll = 13;  WaistPitch = 14
    LeftShoulderPitch = 15;  LeftShoulderRoll = 16;  LeftShoulderYaw = 17
    LeftElbow = 18;  LeftWristRoll = 19;  LeftWristPitch = 20;  LeftWristYaw = 21
    RightShoulderPitch = 22;  RightShoulderRoll = 23;  RightShoulderYaw = 24
    RightElbow = 25;  RightWristRoll = 26;  RightWristPitch = 27;  RightWristYaw = 28
    kNotUsedJoint = 29


ARM_JOINTS = [
    G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
    G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
    G1JointIndex.LeftWristRoll,
    G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.WaistYaw,
    G1JointIndex.WaistRoll,
    G1JointIndex.WaistPitch,
]


# ======================================================================
# 抓取姿态定义
# ======================================================================
def _pose_arms_up():
    """抬起姿态"""
    return [
        -1.0,   0.7,  0.0,  0.6, -0.8,
        -1.0,  -0.7,  0.0,  0.6,  0.8,
         0.0,   0.0,  0.0,
    ]

def _pose_pray():
    """夹紧保持姿态。"""
    return [
        -1.15,  0.5, -0.3,  0.3, -1.8,
        -1.15, -0.5,  0.3,  0.3,  1.8,
         0.0,   0.0,  0.0,
    ]

def _pose_reach_forward():
    """伸手接近目标姿态。"""
    return [
        -0.8,   0.5, -0.4,  0.15, -1.8,
        -0.8,  -0.5,  0.4,  0.15,  1.8,
         0.0,   0.0,  0.0,
    ]


# 抓取序列：伸出 → 抬起 → 夹紧保持
POSE_SEQUENCE = [
    ("reach_forward",  _pose_reach_forward(),  3.0),
    ("arms_up",        _pose_arms_up(),        3.0),
    ("pray",           _pose_pray(),           3.0),
]


class GrabController:
    """G1 抓取控制器 — 伸手 → 抬起 → 夹紧保持。

    与 MultiPoseController 不同，完成后不释放 arm_sdk，
    机器人保持夹紧姿态等待放下指令。
    """

    def __init__(self, poses, transition_time=2.0, kp=60.0, kd=1.5):
        self.poses = poses
        self.transition_time = transition_time
        self.kp = kp
        self.kd = kd
        self.control_dt = 0.02

        self.time = 0.0
        self.done = False
        self.first_low_state = False

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

        self._build_timeline()

    def _build_timeline(self):
        self.timeline = []
        t = 0.0

        # 初始归零
        t_end = t + self.transition_time
        self.timeline.append((t, t_end, "init_zero", None))
        t = t_end

        # 逐个姿态：过渡 + 保持
        for name, angles, hold_time in self.poses:
            t_end = t + self.transition_time
            self.timeline.append((t, t_end, "transition", (name, angles)))
            t = t_end
            t_end = t + hold_time
            self.timeline.append((t, t_end, "hold", (name, angles)))
            t = t_end

        # 不归零、不释放 — 保持最后姿态（夹紧）
        self.hold_start = t
        self.total_time = t

    def Init(self):
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._state_handler, 10)

    def Start(self):
        self.thread = RecurrentThread(
            interval=self.control_dt, target=self._control_loop, name="grab"
        )
        while not self.first_low_state:
            time.sleep(0.1)
        self.thread.Start()

    def _state_handler(self, msg):
        self.low_state = msg
        if not self.first_low_state:
            self.first_low_state = True

    def _get_current_joint_angles(self):
        return [float(self.low_state.motor_state[j].q) for j in ARM_JOINTS]

    def _send_joint_cmd(self, target_angles):
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
        for i, joint in enumerate(ARM_JOINTS):
            self.low_cmd.motor_cmd[joint].q = target_angles[i]
            self.low_cmd.motor_cmd[joint].dq = 0.0
            self.low_cmd.motor_cmd[joint].tau = 0.0
            self.low_cmd.motor_cmd[joint].kp = self.kp
            self.low_cmd.motor_cmd[joint].kd = self.kd

    def _control_loop(self):
        self.time += self.control_dt

        # 序列执行完成后，保持最后的夹紧姿态
        if self.time >= self.total_time:
            # 保持 pray 姿态，arm_sdk 不释放
            last_angles = self.poses[-1][1]  # 最后一个姿态的角度
            self._send_joint_cmd(last_angles)
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.pub.Write(self.low_cmd)
            if not self.done:
                self.done = True
                print("\n  [抓取完成] 保持夹紧姿态，等待放下指令")
            return

        for t_start, t_end, ptype, pdata in self.timeline:
            if t_start <= self.time < t_end:
                ratio = np.clip((self.time - t_start) / (t_end - t_start), 0.0, 1.0)
                current = self._get_current_joint_angles()

                if ptype == "init_zero":
                    target = [(1.0 - ratio) * c for c in current]
                    self._send_joint_cmd(target)

                elif ptype == "transition":
                    name, target_angles = pdata
                    target = [ratio * t + (1.0 - ratio) * c
                              for t, c in zip(target_angles, current)]
                    self._send_joint_cmd(target)
                    if abs(self.time - t_start) < self.control_dt * 1.5:
                        print(f"\n  >>> {name}")

                elif ptype == "hold":
                    _, target_angles = pdata
                    self._send_joint_cmd(target_angles)

                break

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)


if __name__ == '__main__':
    print("WARNING: 确保机器人周围无障碍物!")
    # 作为子进程调用时通过 stdin 管道自动确认
    if sys.stdin.isatty():
        input("按 Enter 开始抓取...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    ctrl = GrabController(POSE_SEQUENCE)
    ctrl.Init()
    ctrl.Start()

    start = time.time()
    while not ctrl.done:
        time.sleep(1)
        elapsed = time.time() - start
        progress = min(ctrl.time / ctrl.total_time * 100, 100) if ctrl.total_time > 0 else 0
        print(f"\r  进度: {progress:5.1f}%  ({elapsed:.0f}s/{ctrl.total_time:.0f}s)",
              end="", flush=True)

    total = time.time() - start
    print(f"\r  进度: 100.0%  ({total:.0f}s/{ctrl.total_time:.0f}s)")
    print("  抓取完成，保持夹紧中...")

    # 保持夹紧状态，等待外部终止
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n  手动终止")
