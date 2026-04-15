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
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize  # DDS 订阅者与工厂初始化
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_  # 低层指令消息默认构造（旧版兼容）
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_  # 低层状态消息默认构造（旧版兼容）
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_  # 低层指令 IDL 消息类型
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_  # 低层状态 IDL 消息类型
from unitree_sdk2py.utils.crc import CRC  # CRC 校验，G1 固件要求每帧指令附带
from unitree_sdk2py.utils.thread import RecurrentThread  # 定时回调线程，用于 50Hz 控制循环
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient  # 运动模式切换客户端

# ---- 常量 ----
kPi = 3.141592654       # 圆周率
kPi_2 = 1.57079632      # π/2 ≈ 90°

class G1JointIndex:
    """G1 机器人关节索引常量，对应 LowCmd/LowState 的 motor_cmd 数组下标。

    关节按顺序编号 0-29，与机器人固件的电机序号一一对应。
    注意：部分关节在不同 DOF 配置下不可用（如 23dof 无手腕 pitch/yaw）。
    """
    # ---- 左腿 (0-5) ----
    LeftHipPitch = 0          # 左髋俯仰
    LeftHipRoll = 1           # 左髋横滚
    LeftHipYaw = 2            # 左髋偏航
    LeftKnee = 3              # 左膝关节
    LeftAnklePitch = 4        # 左踝俯仰（别名 AnkleB）
    LeftAnkleB = 4            # 左踝 B（同 AnklePitch）
    LeftAnkleRoll = 5         # 左踝横滚（别名 AnkleA）
    LeftAnkleA = 5            # 左踝 A（同 AnkleRoll）

    # ---- 右腿 (6-11) ----
    RightHipPitch = 6         # 右髋俯仰
    RightHipRoll = 7          # 右髋横滚
    RightHipYaw = 8           # 右髋偏航
    RightKnee = 9             # 右膝关节
    RightAnklePitch = 10      # 右踝俯仰（别名 AnkleB）
    RightAnkleB = 10          # 右踝 B（同 AnklePitch）
    RightAnkleRoll = 11       # 右踝横滚（别名 AnkleA）
    RightAnkleA = 11          # 右踝 A（同 AnkleRoll）

    # ---- 腰部 (12-14) ----
    WaistYaw = 12             # 腰部偏航
    WaistRoll = 13            # 腰部横滚（注意：29dof 锁腰模式下无效）
    WaistA = 13               # 腰部 A（同 WaistRoll）
    WaistPitch = 14           # 腰部俯仰（注意：29dof 锁腰模式下无效）
    WaistB = 14               # 腰部 B（同 WaistPitch）

    # ---- 左臂 (15-21) ----
    LeftShoulderPitch = 15    # 左肩俯仰
    LeftShoulderRoll = 16     # 左肩横滚
    LeftShoulderYaw = 17      # 左肩偏航
    LeftElbow = 18            # 左肘关节
    LeftWristRoll = 19        # 左腕横滚
    LeftWristPitch = 20       # 左腕俯仰（注意：23dof 下无效）
    LeftWristYaw = 21         # 左腕偏航（注意：23dof 下无效）

    # ---- 右臂 (22-28) ----
    RightShoulderPitch = 22   # 右肩俯仰
    RightShoulderRoll = 23    # 右肩横滚
    RightShoulderYaw = 24     # 右肩偏航
    RightElbow = 25           # 右肘关节
    RightWristRoll = 26       # 右腕横滚
    RightWristPitch = 27      # 右腕俯仰（注意：23dof 下无效）
    RightWristYaw = 28        # 右腕偏航（注意：23dof 下无效）

    # ---- 特殊索引 ----
    kNotUsedJoint = 29        # 该索引非真实关节，用于 arm_sdk 使能/失能控制
                              # q=1 时启用手臂 SDK 控制，q=0 时释放回固件

class Custom:
    """G1 手臂控制演示类。

    实现四阶段手臂运动序列：
    1. 归零阶段 — 将手臂从当前位置平滑插值到零位
    2. 抬臂阶段 — 将手臂平滑抬起至目标姿态
    3. 归位阶段 — 将手臂平滑放回零位
    4. 释放阶段 — 逐步释放 arm_sdk 控制权，交还给机器人固件

    关节控制采用 PD 位置控制：通过设置 q（目标角度）、kp（位置增益）、
    kd（阻尼增益）、dq（前馈速度）、tau（前馈力矩）来控制每个关节。
    """

    def __init__(self):
        # ---- 时间参数 ----
        self.time_ = 0.0           # 当前控制时间累计（秒）
        self.control_dt_ = 0.02    # 控制周期 50Hz（秒），与 G1 固件一致
        self.duration_ = 3.0       # 基础阶段时长（秒），各阶段为其整数倍

        # ---- 状态计数 ----
        self.counter_ = 0          # 保留计数器（当前未使用）

        # ---- 力控参数 ----
        self.weight = 0.           # 重力补偿权重（当前未使用）
        self.weight_rate = 0.2     # 重力补偿切换速率（当前未使用）
        self.kp = 60.              # 位置控制比例增益 (N·m/rad)
        self.kd = 1.5              # 位置控制阻尼增益 (N·m·s/rad)
        self.dq = 0.               # 关节前馈速度 (rad/s)，本脚本设为 0
        self.tau_ff = 0.           # 关节前馈力矩 (N·m)，本脚本设为 0

        # ---- 模式状态机 ----
        self.mode_machine_ = 0     # 当前运动模式（保留字段）

        # ---- DDS 通信对象 ----
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()   # 低层指令消息实例
        self.low_state = None                              # 最新低层状态（由回调填充）

        # ---- 初始化标志 ----
        self.first_update_low_state = False  # 是否已收到第一条关节状态消息
        self.crc = CRC()                      # CRC 校验器
        self.done = False                     # 动作序列是否完成

        # ---- 手臂目标角度（单位：弧度） ----
        # 对应 arm_joints 列表中 13 个关节的目标位置
        # 顺序：左臂5关节 + 右臂5关节 + 腰部3关节
        # 0.0=零位, kPi_2=90°, -kPi_2=-90°
        self.target_pos = [
            # 左臂：肩俯仰=0°, 肩横滚=90°, 肩偏航=0°, 肘=90°, 腕横滚=0°
            0.0,      kPi_2,  0.0,    kPi_2,  0.0,
            # 右臂：肩俯仰=0°, 肩横滚=-90°, 肩偏航=0°, 肘=90°, 腕横滚=0°
            0.0,     -kPi_2,  0.0,    kPi_2,  0.0,
            # 腰部：偏航=0°, 横滚=0°, 俯仰=0°
            0.0,      0.0,    
        ]

        # ---- 本脚本控制的关节索引列表 ----
        # 包含双臂 10 个关节 + 腰部 3 个关节 = 13 个
        # 注意：腿部关节（0-11）不在 arm_sdk 控制范围内，由固件运动控制
        self.arm_joints = [
          G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
          G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
          G1JointIndex.LeftWristRoll,
          G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
          G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
          G1JointIndex.RightWristRoll,
          G1JointIndex.WaistYaw,
          G1JointIndex.WaistRoll,
          G1JointIndex.WaistPitch
        ]

    def Init(self):
        """初始化 DDS 发布者和订阅者。

        发布者：向 "rt/arm_sdk" 通道发送低层关节指令（LowCmd）
        订阅者：从 "rt/lowstate" 通道接收低层关节状态（LowState）
        """
        # 创建手臂指令发布者，通道名 "rt/arm_sdk" 是固件定义的手臂 SDK 接口
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        # 创建低层状态订阅者，获取所有 30 个关节的当前位置、速度、力矩反馈
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        """启动控制循环。

        等待第一条关节状态消息到达后，启动 50Hz 定时控制线程。
        必须收到状态才能获取关节初始位置，用于平滑插值。
        """
        # 创建 50Hz 控制线程，回调函数为 LowCmdWrite
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        # 阻塞等待，直到收到第一条 lowstate 消息
        while self.first_update_low_state == False:
            time.sleep(1)

        if self.first_update_low_state == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        """低层状态回调，由 DDS 订阅者定时调用（~50Hz）。

        Args:
            msg: 包含所有关节当前状态（q=角度, dq=速度, tau=力矩）
        """
        self.low_state = msg

        # 标记已收到状态，解除 Start() 中的阻塞等待
        if self.first_update_low_state == False:
            self.first_update_low_state = True
        
    def LowCmdWrite(self):
        """50Hz 控制回调 — 根据当前阶段计算关节指令并发送。

        四阶段时间线（基于 duration_=3s）：
        - [0s, 3s)      Stage 1: 归零 — 从当前位置插值到零位
        - [3s, 9s)      Stage 2: 抬臂 — 从零位插值到目标姿态
        - [9s, 18s)     Stage 3: 归位 — 从目标姿态插值回零位
        - [18s, 21s)    Stage 4: 释放 — 逐步关闭 arm_sdk 控制权
        - [21s, ...)    完成 — 设置 done 标志

        每帧末尾计算 CRC 并通过 DDS 发送指令。
        """
        self.time_ += self.control_dt_

        if self.time_ < self.duration_ :
          # [Stage 1]: 将手臂从当前位置平滑归零
          # 启用 arm_sdk 控制
          self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:Disable arm_sdk
          for i,joint in enumerate(self.arm_joints):
            # ratio 从 0 线性增长到 1，实现平滑过渡
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            self.low_cmd.motor_cmd[joint].tau = 0.   # 无前馈力矩
            # 目标角度 = (1-ratio) × 当前角度 + ratio × 0（归零）
            self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q 
            self.low_cmd.motor_cmd[joint].dq = 0.    # 目标速度为 0
            self.low_cmd.motor_cmd[joint].kp = self.kp 
            self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 3 :
          # [Stage 2]: 将手臂从零位平滑抬起到目标姿态
          # 持续时间为 duration_ × 2 = 6 秒，动作更缓慢安全
          for i,joint in enumerate(self.arm_joints):
              ratio = np.clip((self.time_ - self.duration_) / (self.duration_ * 2), 0.0, 1.0)
              self.low_cmd.motor_cmd[joint].tau = 0.   
              # 目标角度 = ratio × target_pos[i] + (1-ratio) × 当前角度
              self.low_cmd.motor_cmd[joint].q = ratio * self.target_pos[i] + (1.0 - ratio) * self.low_state.motor_state[joint].q 
              self.low_cmd.motor_cmd[joint].dq = 0.   
              self.low_cmd.motor_cmd[joint].kp = self.kp 
              self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 6 :
          # [Stage 3]: 将手臂从目标姿态平滑放回零位
          # 持续时间为 duration_ × 3 = 9 秒，缓慢放下确保安全
          for i,joint in enumerate(self.arm_joints):
              ratio = np.clip((self.time_ - self.duration_*3) / (self.duration_ * 3), 0.0, 1.0)
              self.low_cmd.motor_cmd[joint].tau = 0.   
              # 目标角度 = (1-ratio) × 当前角度（ratio→1 时趋近于 0）
              self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q
              self.low_cmd.motor_cmd[joint].dq = 0.   
              self.low_cmd.motor_cmd[joint].kp = self.kp 
              self.low_cmd.motor_cmd[joint].kd = self.kd

        elif self.time_ < self.duration_ * 7 :
          # [Stage 4]: 逐步释放 arm_sdk 控制权
          # 将 kNotUsedJoint 的 q 值从 1 平滑过渡到 0
          # q=0 后固件重新接管手臂控制
          for i,joint in enumerate(self.arm_joints):
              ratio = np.clip((self.time_ - self.duration_*6) / (self.duration_), 0.0, 1.0)
              self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q =  (1 - ratio) # 1:Enable arm_sdk, 0:Disable arm_sdk
        
        else:
            # 所有阶段完成，标记退出
            self.done = True
  
        # 计算整帧 CRC 校验值并附带到指令消息
        # G1 固件会校验 CRC，不匹配则丢弃该帧
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        # 通过 DDS 发布指令到机器人
        self.arm_sdk_publisher.Write(self.low_cmd)

if __name__ == '__main__':
    # ---- 安全警告 ----
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    # 等待用户确认，防止误触运行
    input("Press Enter to continue...")

    # ---- 初始化 DDS 通信工厂 ----
    # 参数: (模式, 网络接口名)
    # 模式=0 表示 UDP 单播模式
    # 不传网络接口名则自动检测
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # ---- 创建控制器并启动 ----
    custom = Custom()
    custom.Init()    # 初始化 DDS 通道
    custom.Start()   # 等待状态并启动 50Hz 控制线程

    # ---- 主线程阻塞等待动作完成 ----
    while True:        
        time.sleep(1)
        if custom.done: 
           print("Done!")
           sys.exit(-1)     