# Unitree SDK2 Python 使用手册

> G1 人形机器人 · SDK 1.0.1 · 基于 CycloneDDS + RPC

---

## 目录

- [1. 架构总览](#1-架构总览)
- [2. 快速开始](#2-快速开始)
- [3. DDS 通道通信](#3-dds-通道通信)
- [4. RPC 远程调用](#4-rpc-远程调用)
- [5. G1 控制模式对比](#5-g1-控制模式对比)
- [6. 高层运动控制 (RPC)](#6-高层运动控制-rpc)
- [7. 底层电机控制 (DDS Topic)](#7-底层电机控制-dds-topic)
- [8. Arm SDK 手臂控制](#8-arm-sdk-手臂控制)
- [9. IDL 数据结构](#9-idl-数据结构)
- [10. 工具类库](#10-工具类库)
- [11. 常见问题](#11-常见问题)

---

## 1. 架构总览

### 三层控制架构

```
┌─────────────────────────────────────────────────────────┐
│                    应用层 (你的代码)                      │
├──────────┬──────────────┬───────────────────────────────┤
│ 高层 RPC │   Arm SDK    │      底层 DDS 直接控制          │
│ (运动/手势)│  (手臂+腰部) │     (29关节全控制)              │
├──────────┴──────────────┼───────────────────────────────┤
│         LocoClient / ArmActionClient / ChannelPub/Sub     │
├────────────────────────┴─────────────────────────────────┤
│                  CycloneDDS 通信层                        │
└─────────────────────────────────────────────────────────┘
```

### 模块关系图

```
unitree_sdk2py/
├── core/channel.py          ← 核心通信：Publisher / Subscriber
├── rpc/client.py            ← RPC 客户端基类
├── idl/unitree_hg/          ← G1/H1-2 数据结构（35电机）
│   ├── LowCmd_              → 低层指令
│   ├── LowState_            → 低层状态
│   └── MotorCmd_ / MotorState_
├── g1/
│   ├── loco/g1_loco_client.py       → 运动控制客户端 (sport 服务)
│   ├── arm/g1_arm_action_client.py  → 手臂动作客户端 (arm 服务)
│   └── audio/g1_audio_client.py     → 音频/TTS/LED 客户端
└── comm/motion_switcher_client.py   → 运动模式切换器
```

### 关键区分：`unitree_go` vs `unitree_hg`

| 包名 | 适用机型 | 电机数 | 说明 |
|------|---------|--------|------|
| `unitree_go` | Go2 / B2 / H1 (四足) | 20 | 四足机器人 |
| **`unitree_hg`** | **G1 / H1-2 (人形)** | **35** | **人形机器人，本项目使用** |

---

## 2. 快速开始

### 安装依赖

```bash
# Python >= 3.8, Ubuntu 20.04+
pip install unitree_sdk2py numpy opencv-python
# 或从源码安装
cd unitree_sdk2_python && pip install -e .
```

### 最简示例 — DDS 发布者/订阅者

```python
import time
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

# 0. 初始化 DDS 工厂（必须最先调用）
ChannelFactoryInitialize(0)          # 参数: (mode=0, networkInterface="")
# 多网卡时指定接口:
# ChannelFactoryInitialize(0, "eth0")

# 1. 发布者
pub = ChannelPublisher("my_topic", str)  # 通道名 + 消息类型
pub.Init()
pub.Write("Hello World")

# 2. 订阅者（回调方式）
def callback(msg):
    print(f"收到: {msg}")

sub = ChannelSubscriber("my_topic", str)
sub.Init(callback, queue_len=10)        # 队列长度

time.sleep(1)                            # 等待消息
msg = sub.Read()                         # 阻塞读取（可选）
```

---

## 3. DDS 通道通信

### 3.1 全局初始化

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# 必须在所有操作之前调用一次
ChannelFactoryInitialize(mode=0, networkInterface="")

# mode=0: UDP 单播模式（推荐）
# mode=1: 共享内存模式（单机调试）
# networkInterface: 网卡名，如 "eth0", "enp2s0"，空则自动检测
```

### 3.2 Publisher（发布者）

```python
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

pub = ChannelPublisher("rt/arm_sdk", LowCmd_)  # 通道名 + 类型
pub.Init()

low_cmd = LowCmd_()
low_cmd.motor_cmd[12].q = 0.5          # 设置目标角度
low_cmd.crc = crc.Crc(low_cmd)           # CRC 校验
pub.Write(low_cmd)                       # 发送
```

### 3.3 Subscriber（订阅者）

```python
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

class MyNode:
    def __init__(self):
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateHandler, 10)  # 回调方式，队列长10
        self.low_state = None                     # 缓存最新状态

    def LowStateHandler(self, msg: LowState_):
        """回调函数，由 DDS 线程调用"""
        self.low_state = msg
        # 读取关节角度
        q = msg.motor_state[12].q               # WaistYaw 角度
        dq = msg.motor_state[12].dq             # WaistYaw 速度
```

### 3.4 标准 DDS 通道命名

| 用途 | 发送方向 | 通道名 |
|------|---------|--------|
| **低层命令** | 应用→机器人 | `rt/lowcmd` |
| **低层状态** | 机器人→应用 | `rt/lowstate` |
| **Arm SDK 命令** | 应用→机器人 | `rt/arm_sdk` |
| **Sport Mode State** | 机器人→应用 | `rt/sportmodestate` |
| **RPC Request** | 客户端→服务端 | `rt/api/{service}/request` |
| **RPC Response** | 服务端→客户端 | `rt/api/{service}/response` |

---

## 4. RPC 远程调用

### 4.1 原理

RPC（Remote Procedure Call）是构建在 DDS 之上的请求-响应协议：

```
Client                          Robot Firmware (Server)
  │                                  │
  │  ── Call(apiId, params) ──→     │
  │                                  │  执行对应 API
  │  ←── Response(result) ────      │
  │                                  │
```

- 通过 `rt/api/{serviceName}/request` 发送请求
- 通过 `rt/api/{serviceName}/response` 接收响应
- 同步阻塞等待响应（带超时机制）

### 4.2 ClientBase 方法一览

所有 RPC 客户端继承自 `ClientBase`：

```python
from unitree_sdk2py.rpc.client_base import ClientBase

client.SetTimeout(5.0)                              # 设置超时（秒）
client._RegistApi(api_id, priority=0)                # 注册要调用的 API
client._SetApiVerson(version_str="1.0.0.0")          # 设置 API 版本
result = client._Call(api_id, json_param)             # 同步调用（返回 JSON）
client._CallNoReply(api_id, json_param)               # 无响应调用
result = client._CallBinary(api_id, bytes_data)       # 二进制数据调用
```

### 4.3 Server（服务端）— 自定义 RPC 服务

```python
from unitree_sdk2py.rpc.server import Server

server = Server("my_service")                         # 创建服务

def my_handler(req):                                   # 处理器函数
    return {"status": "ok"}

server._RegistHandler(api_id, my_handler, False)      # 注册处理器
server.Start()                                         # 启动监听
```

### 4.4 错误码

```python
from unitree_sdk2py.rpc.internal import *

RPC_OK = 0                                            # 成功

# 客户端错误 (3001-3107)
RPC_ERR_CLIENT_API_TIMEOUT = 3104                     # 调用超时
RPC_ERR_CLIENT_API_NOT_REG = 3103                     # API 未注册
RPC_ERR_CLIENT_LEASE_INVALID = 3107                   # 租约无效

# 服务端错误 (3201-3207)
RPC_ERR_SERVER_API_NOT_IMPL = 3203                    # API 未实现
RPC_ERR_SERVER_API_PARAMETER = 3204                   # 参数错误
```

### 4.5 Lease 租约机制

用于多客户端互斥访问：

```python
from unitree_sdk2py.rpc.client import Client
from unitree_sdk2py.rpc.lease_client import LeaseClient

# 创建带租约的客户端
client = Client("arm", enableLease=True)
client.LeaseInit(term=1.0)                            # 租约有效期 1s
client.LeaseGet()                                     # 申请租约
# ... 执行操作 ...
client.LeasePut()                                     # 释放租约
```

---

## 5. G1 控制模式对比

| 维度 | 高层 RPC (LocoClient) | 底层 DDS (LowCmd) | Arm SDK (rt/arm_sdk) |
|------|----------------------|-------------------|---------------------|
| **适用场景** | 行走/起立/手势等标准动作 | 全关节精细控制 | 仅手臂+腰部控制 |
| **复杂度** | ⭐ 简单 | ⭐⭐⭐ 复杂 | ⭐⭐ 中等 |
| **安全保护** | ✅ 内置平衡算法 | ❌ 需自行保证安全 | ✅ 腿部仍由固件控制 |
| **需要 ReleaseMode** | ❌ 不需要 | ✅ 必须释放高层控制 | ❌ 不需要 |
| **控制频率** | 按需调用 | ≥500Hz (推荐1000Hz) | 50Hz 即可 |
| **CRC 校验** | 自动处理 | **必须手动计算** | **必须手动计算** |
| **自由度** | 受限于预设 API | 任意轨迹规划 | 臂+腰共13关节 |

### 选择建议

```
需求场景                          →  选择方案
─────────────────────────────────────────────────
让机器人走过去、挥手、握手          →  LocoClient (高层 RPC)
精确控制手臂抓取物体位置           →  Arm SDK (rt/arm_sdk)
自定义全身舞蹈/步态               →  底层 LowCmd (全控制)
视觉伺服追踪（旋转腰部）          →  Arm SDK (仅控制 WaistYaw)
```

---

## 6. 高层运动控制 (RPC)

### 6.1 LocoClient — 运动控制

**服务名**: `"sport"` | **API版本**: `"1.0.0.0"`

#### 初始化与基本使用

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# 1. 初始化 DDS
ChannelFactoryInitialize(0, "eth0")

# 2. 创建客户端
loco = LocoClient()
loco.SetTimeout(5.0)                 # 单次调用超时 5s
loco.Init()                           # 注册所有 API

# 3. 调用运动命令
loco.Move(0.3, 0, 0, True)            # 以 0.3m/s 向前移动
loco.StopMove()                       # 停止移动
loco.BalanceStand(mode=1)             # 平衡站立
```

#### 完整 API 参考

##### FSM 状态机控制

| 方法 | API ID | 功能 | 备注 |
|------|--------|------|------|
| `Damp()` | 1 | **阻尼/下电模式** | 所有电机软制动 |
| `Start()` | 200 | 开始运动 | 从 Damp 切换到正常模式 |
| `ZeroTorque()` | 0 | **零力矩模式** | 关节无扭矩输出 |
| `Sit()` | 3 | **坐下** | |
| `Squat2StandUp()` | 706 | **蹲下→起立** | 需先处于蹲姿 |
| `StandUp2Squat()` | 706 | **起立→蹲下** | 需先站立 |
| `Lie2StandUp()` | 702 | **躺倒→起立** | ⚠️ 需面朝上、地面硬平糙 |

##### 速度控制 (VELOCITY 7105)

| 方法 | API ID | 参数 | 说明 |
|------|--------|------|------|
| `Move(vx, vy, vyaw, continuous)` | 7105 | `(m/s, m/s, rad/s, bool)` | 连续或一次性速度 |
| `StopMove()` | VELOCITY | 无 | 停止移动（发 STOPMOVE 命令） |
| `SetVelocity(vx, vy, omega, duration)` | 7105 | 同上 + 持续时间(s) | 设置持续速度 |

```python
# 示例：向前移动
loco.Move(vx=0.3, vy=0.0, vyaw=0.0, continuous=True)

# 示例：原地旋转
loco.Move(vx=0.0, vy=0.0, vyaw=0.3, continuous=True)

# 示例：停止
loco.StopMove()
```

**注意**: G1 人形机器人可能不支持侧移 (`vy`)，建议只使用 `vx` 和 `vyaw`。

##### 站立高度控制 (STAND_HEIGHT 7104)

| 方法 | 参数值 | 效果 |
|------|--------|------|
| `HighStand()` | UINT32_MAX | **高站姿** |
| `LowStand()` | 0 | **低站姿** |
| `SetStandHeight(height)` | 浮点值 | 自定义高度 |

##### 平衡模式 (BALANCE_MODE 7102)

| 方法 | 参数 | 效果 |
|------|------|------|
| `BalanceStand(mode)` | 0=默认, 1=增强, 2=锁定 | 平衡站立 |

##### 手臂任务 (ARM_TASK 7106)

| 方法 | stage 参数 | 效果 |
|------|-----------|------|
| `WaveHand(turn_flag)` | 0=不转, 1=转身 | **挥手** |
| `ShakeHand(stage)` | 0/1/2 阶段 | **握手** |
| `SetTaskId(task_id)` | 整数 | 设置手臂任务ID |

##### 其他

| 方法 | API ID | 说明 |
|------|--------|------|
| `SetFsmId(id)` | 7101 | 设置 FSM 状态 ID |

#### 完整测试示例代码

```python
"""G1 运动控制完整示例 — 13 种预设动作"""

import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def main():
    ChannelFactoryInitialize(0, "eth0")

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    test_menu = {
        0: ("阻尼 Damp", lambda: loco.Damp()),
        1: ("蹲下→起立 Squat2StandUp", lambda: loco.Squat2StandUp()),
        2: ("起立→蹲下 StandUp2Squat", lambda: loco.StandUp2Squat()),
        3: ("前进 Move forward", lambda: loco.Move(0.3, 0, 0)),
        4: ("横向移动 lateral", lambda: loco.Move(0, 0.3, 0)),
        5: ("旋转 rotate", lambda: loco.Move(0, 0, 0.3)),
        6: ("低站姿 LowStand", lambda: loco.LowStand()),
        7: ("高站姿 HighStand", lambda: loco.HighStand()),
        8: ("零力矩 ZeroTorque", lambda: loco.ZeroTorque()),
        9: ("挥手(不转身) WaveHand", lambda: loco.WaveHand(0)),
        10: ("挥手(转身) WaveHand+Turn", lambda: loco.WaveHand(1)),
        11: ("握手 ShakeHand", lambda: loco.ShakeHand(0)),
        12: ("躺倒→起立 Lie2StandUp", lambda: loco.Lie2StandUp()),
    }

    choice = int(input("输入测试项编号 (0-12): "))
    if choice in test_menu:
        name, func = test_menu[choice]
        print(f"执行: {name}")
        func()
        time.sleep(3)


if __name__ == "__main__":
    main()
```

### 6.2 G1ArmActionClient — 手臂预设动作

**服务名**: `"arm"` | **API版本**: `"1.0.0.14"`

#### 16 种预设动作

```python
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient

ChannelFactoryInitialize(0, "eth0")
arm = G1ArmActionClient()
arm.SetTimeout(5.0)
arm.Init()

# 执行预设动作
arm.ExecuteAction(action_id=17)     # 17 = 鼓掌 (clap)

# 获取可用动作列表
action_list = arm.GetActionList()
print(f"可用动作: {action_list}")
```

#### 动作 ID 映射表

| action_id | 名称 | 描述 |
|-----------|------|------|
| 99 | release arm | 释放手臂（交还固件控制） |
| 11 | two-hand kiss | 双手吻 |
| 12 | left kiss | 左吻 |
| 13 | right kiss | 右吻 |
| 15 | hands up | 举手 |
| 17 | clap | 鼓掌 |
| 18 | high five | 击掌 |
| 19 | hug | 拥抱 |
| 20 | heart | 比心 |
| 21 | right heart | 右侧比心 |
| 22 | reject | 拒绝 |
| 23 | right hand up | 右手举起 |
| 24 | x-ray | X光 |
| 25 | face wave | 挥手 |
| 26 | high wave | 高举挥手 |
| 27 | shake hand | 握手 |

### 6.3 AudioClient — 音频/TTS/LED

**服务名**: `"voice"` | **API版本**: `"1.0.0.0"` (部分 API `"1.0.0.14"`)

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

ChannelFactoryInitialize(0, "eth0")

audio = AudioClient()
audio.SetTimeout(5.0)
audio.Init()

audio.TtsMaker("你好，我是宇树G1机器人", speaker_id=0)   # 文字转语音
volume = audio.GetVolume()                                 # 获取音量
audio.SetVolume(80)                                        # 设置音量 0-100
audio.LedControl(R=255, G=128, B=0)                       # LED 设为橙色
```

### 6.4 MotionSwitcherClient — 运动模式切换

**服务名**: `"motion_switcher"` | **API版本**: `"1.0.0.1"`

在使用底层 DDS 控制之前，**必须先释放高层控制权**：

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
import time

ChannelFactoryInitialize(0, "eth0")

msc = MotionSwitcherClient()
msc.SetTimeout(5.0)
msc.Init()

# 检查当前模式
code, info = msc.CheckMode()
print(f"当前模式: {info['name']}")

# 循环尝试释放高层控制，直到成功
while msc.CheckMode()[1]['name']:
    msc.ReleaseMode()
    time.sleep(1)

print("已释放高层控制，可以接管底层控制了！")
```

**可用模式别名**:

| 别名 | 名称 | 说明 |
|------|------|------|
| `"normal"` | 正常模式 | 默认，支持高层运动 API |
| `"advanced"` | 高级模式 | 支持更多底层功能 |
| `"ai"` | AI 模式 | AI 相关功能 |
| `"ai-w"` | AI 写入模式 | AI 写入功能 |

---

## 7. 底层电机控制 (DDS Topic)

### 7.1 完整控制流程

```
初始化 DDS → 释放高层控制 → 订阅 lowstate → 创建 lowcmd publisher
                                                    ↓
                                          定时循环 (≥500Hz):
                                              读取状态 → 计算指令 → CRC校验 → Write
```

### 7.2 完整代码模板

```python
"""
G1 底层电机控制完整模板 — 29DOF 全关节 PD 位置控制
"""
import time
import sys
import threading
import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient


# ================================================================
# G1 关节索引常量
# ================================================================
class Joints:
    """G1 29 DOF 关节索引"""
    # 左腿 (0-5)
    LeftHipPitch, LeftHipRoll, LeftHipYaw = 0, 1, 2
    LeftKnee, LeftAnklePitch, LeftAnkleRoll = 3, 4, 5
    # 右腿 (6-11)
    RightHipPitch, RightHipRoll, RightHipYaw = 6, 7, 8
    RightKnee, RightAnklePitch, RightAnkleRoll = 9, 10, 11
    # 腰部 (12-14), 注意 29dof 锁腰时 Roll/Pitch 无效
    WaistYaw, WaistRoll, WaistPitch = 12, 13, 14
    # 左臂 (15-21), 注意 23dof 时 WristPitch/WristYaw 无效
    LeftShoulderPitch, LeftShoulderRoll, LeftShoulderYaw = 15, 16, 17
    LeftElbow, LeftWristRoll, LeftWristPitch, LeftWristYaw = 18, 19, 20, 21
    # 右臂 (22-28), 注意 23dof 时 WristPitch/WristYaw 无效
    RightShoulderPitch, RightShoulderRoll, RightShoulderYaw = 22, 23, 24
    RightElbow, RightWristRoll, RightWristPitch, RightWristYaw = 25, 26, 27, 28
    # 特殊
    kNotUsedJoint = 29    # arm_sdk 使能位 (q=1 使能, q=0 失能)


class LowLevelController:
    """底层电机控制器 — 500Hz PD 位置控制"""

    def __init__(self, control_joints=None):
        """
        Args:
            control_joints: 要控制的关节索引列表。None 则控制全部 29 个。
                           如只需控制腰部: [Joints.WaistYaw]
        """
        # ---- 控制参数 ----
        self.control_dt = 0.002          # 控制周期 500Hz（官方推荐）
        self.kp_default = 60.0           # 默认位置增益
        self.kd_default = 1.5            # 默认阻尼增益

        # ---- 要控制的关节列表 ----
        if control_joints is not None:
            self.joint_ids = control_joints
        else:
            # 默认全部 29 个关节（排除 kNotUsedJoint）
            self.joint_ids = list(range(29))

        # ---- 目标位置（弧度） ----
        self.target_pos = {i: 0.0 for i in self.joint_ids}

        # ---- DDS 对象 ----
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()
        self.running = True

        # ---- 状态标志 ----
        self.first_state_received = False

    def init_dds(self, net_iface=""):
        """初始化 DDS 和通道。"""
        # 1. 全局初始化
        ChannelFactoryInitialize(0, net_iface)

        # 2. 释放高层控制（必须！）
        self._release_high_level_control()

        # 3. 发布低层指令
        self.cmd_pub = ChannelPublisher("rt/lowcmd", unitree_hg_msg_dds__LowCmd_)
        self.cmd_pub.Init()

        # 4. 订阅低层状态
        self.state_sub = ChannelSubscriber(
            "rt/lowstate",
            unitree_hg_msg_dds__LowState_,
        )
        self.state_sub.Init(self._state_callback, 10)

        # 5. 启动定时控制线程
        self.ctrl_thread = RecurrentThread(
            interval=self.control_dt,
            target=self._control_loop,
            name="low_ctrl",
        )

        print("等待关节状态...")
        timeout_start = time.time()
        while not self.first_state_received and time.time() - timeout_start < 10:
            time.sleep(0.05)

        if self.first_state_received:
            print(f"状态就绪，启动 {int(1/self.control_dt)}Hz 控制循环")
            self.ctrl_thread.Start()
        else:
            print("ERROR: 等待状态超时！")

    def _release_high_level_control(self):
        """通过 MotionSwitcher 释放高层控制权。"""
        try:
            msc = MotionSwitcherClient()
            msc.SetTimeout(5.0)
            msc.Init()

            for _ in range(30):  # 最多等 30s
                code, info = msc.CheckMode()
                if not info.get('name'):
                    print("高层控制已释放")
                    break
                print(f"当前模式: {info['name']}，正在释放...")
                msc.ReleaseMode()
                time.sleep(1.0)
            else:
                print("WARNING: 未能释放高层控制！")
        except Exception as e:
            print(f"WARNING: MotionSwitcher 异常: {e}")

    def _state_callback(self, msg):
        """低层状态回调。"""
        self.low_state = msg
        if not self.first_state_received:
            self.first_state_received = True

    def _control_loop(self):
        """500Hz 控制循环 — 核心逻辑在此实现。"""
        # ===== 在这里编写你的控制逻辑 =====

        for joint_id in self.joint_ids:
            cmd = self.low_cmd.motor_cmd[joint_id]
            cmd.mode = 1                    # 1=Enable
            cmd.q = self.target_pos[joint_id]  # 目标位置 (rad)
            cmd.dq = 0.0                    # 目标速度 (rad/s)
            cmd.tau = 0.0                   # 前馈力矩 (N·m)
            cmd.kp = self.kp_default        # 位置增益
            cmd.kd = self.kd_default        # 阻尼增益

        # CRC 校验 — 必须每帧都算！
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)

        # 发送指令到机器人
        self.cmd_pub.Write(self.low_cmd)

    def set_target(self, joint_id, angle_rad):
        """设置单个关节的目标角度。"""
        self.target_pos[joint_id] = angle_rad

    def set_targets(self, targets_dict):
        """批量设置目标角度。targets_dict: {joint_id: angle_rad}"""
        self.target_pos.update(targets_dict)

    def stop(self):
        """停止控制并清理。"""
        self.running = False
        if hasattr(self, 'ctrl_thread') and self.ctrl_thread.is_alive():
            self.ctrl_thread.join(timeout=2.0)

    def get_current_positions(self):
        """获取所有关节当前位置。"""
        if self.low_state is None:
            return {}
        return {
            i: self.low_state.motor_state[i].q
            for i in self.joint_ids
        }


# ================================================================
# 使用示例
# ================================================================
if __name__ == "__main__":
    controller = LowLevelController(control_joints=[
        Joints.WaistYaw,  # 只控制腰部偏航
    ])
    controller.init_dds(net_iface="eth0" if len(sys.argv) > 1 else "")

    # 让腰部缓慢转到 30° (≈0.52 rad)
    duration = 3.0
    target_angle = 0.52

    start_time = time.time()
    while time.time() - start_time < duration:
        ratio = min((time.time() - start_time) / duration, 1.0)
        current_angle = ratio * target_angle
        controller.set_target(Joints.WaistYaw, current_angle)
        time.sleep(controller.control_dt * 2)

    # 保持 2 秒
    time.sleep(2.0)

    # 回零
    start_time = time.time()
    while time.time() - start_time < 2.0:
        ratio = max(1.0 - (time.time() - start_time) / 2.0, 0.0)
        controller.set_target(Joints.WaistYaw, ratio * target_angle)
        time.sleep(controller.control_dt * 2)

    print("完成!")
    controller.stop()
```

### 7.3 控制频率说明

| 频率 | 周期 | 适用场景 |
|------|------|---------|
| **1000 Hz** | 1 ms | 官方推荐，腿部控制 |
| **500 Hz** | 2 ms | 手臂控制最低要求 |
| **50 Hz** | 20 ms | Arm SDK 模式（可接受） |
| **< 30 Hz** | >33 ms | ⚠️ 可能导致抖动或不稳定 |

---

## 8. Arm SDK 手臂控制

### 8.1 与底层控制的区别

| 特性 | rt/lowcmd (底层) | rt/arm_sdk (Arm SDK) |
|------|------------------|---------------------|
| 控制范围 | 全部 29 个关节 | 仅手臂+腰部 (13个) |
| 腿部控制 | ✅ 需要 | ❌ 固件继续控制腿部 |
| 需要 ReleaseMode | ✅ 必须 | ❌ 不需要 |
| 使能方式 | 无 | motor_cmd[29].q = 1/0 |
| 控制频率 | ≥500Hz | 50Hz 即可 |
| 安全风险 | 高（全身） | 中（仅上半身） |

### 8.2 Arm SDK 控制模板

```python
"""
Arm SDK 控制 — 仅控制手臂和腰部，不需要 ReleaseMode。
适合：视觉伺服追踪、手臂定位、手势跟随等。
"""
import time
import sys
import threading
import numpy as np
from math import pi as kPi, pi as kPi_2

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


class Joints:
    """G1 关节索引"""
    WaistYaw = 12; WaistRoll = 13; WaistPitch = 14
    # ... 同上
    kNotUsedJoint = 29  # ★ Arm SDK 特殊使能位


class ArmSdkController:
    """Arm SDK 控制器 — 50Hz 即可稳定工作"""

    def __init__(self):
        self.control_dt = 0.02          # 50Hz（比底层控制慢很多）
        self.kp = 60.0
        self.kd = 1.5
        self.running = True

        # 要控制的关节：双臂 10 + 腰部 3 = 13
        self.arm_joints = [
            15, 16, 17, 18, 19,        # 左臂
            22, 23, 24, 25, 26,        # 右臂
            12, 13, 14,                # 腰部
        ]

        self.target_pos = {i: 0.0 for i in self.arm_joints}

        # DDS 对象
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()
        self.state_ready = False

    def init_dds(self, net_iface=""):
        """初始化 — 不需要 ReleaseMode！"""
        ChannelFactoryInitialize(0, net_iface)

        # ★ Arm SDK 专用发布通道
        self.cmd_pub = ChannelPublisher("rt/arm_sdk", unitree_hg_msg_dds__LowCmd_)
        self.cmd_pub.Init()

        self.state_sub = ChannelSubscriber("rt/lowstate", unitree_hg_msg_dds__LowState_)
        self.state_sub.Init(self._on_state, 10)

        # 启动控制线程
        self.thread = RecurrentThread(self.control_dt, self._loop, name="arm_sdk")
        
        print("等待关节状态...")
        t0 = time.time()
        while not self.state_ready and time.time() - t0 < 10:
            time.sleep(0.05)

        if self.state_ready:
            print("Arm SDK 就绪，启动控制")
            self.thread.Start()
        else:
            print("ERROR: 超时")

    def _on_state(self, msg):
        self.low_state = msg
        if not self.state_ready:
            self.state_ready = True

    def _loop(self):
        """50Hz 控制循环。"""
        # ★ 使能 Arm SDK（kNotUsedJoint.q = 1 表示启用 arm_sdk 模式）
        self.low_cmd.motor_cmd[Joints.kNotUsedJoint].q = 1.0

        for jid in self.arm_joints:
            cmd = self.low_cmd.motor_cmd[jid]
            cmd.mode = 1
            cmd.q = self.target_pos.get(jid, 0.0)
            cmd.dq = 0.0
            cmd.tau = 0.0
            cmd.kp = self.kp
            cmd.kd = self.kd

        # CRC
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.cmd_pub.Write(self.low_cmd)

    def set_targets(self, targets):
        """设置目标位置。targets: dict{joint_index: angle_rad}"""
        self.target_pos.update(targets)

    def release_arm(self, duration=2.0):
        """平滑释放 arm_sdk 控制权（逐步将 q 从 1 降到 0）。"""
        t0 = time.time()
        while time.time() - t0 < duration:
            ratio = 1.0 - (time.time() - t0) / duration
            self.low_cmd.motor_cmd[Joints.kNotUsedJoint].q = max(ratio, 0.0)
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.cmd_pub.Write(self.low_cmd)
            time.sleep(self.control_dt)

    def stop(self):
        """停止控制线程。"""
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=3.0)


if __name__ == "__main__":
    ctrl = ArmSdkController()
    ctrl.init_dds(sys.argv[1] if len(sys.argv) > 1 else "")

    # 示例：举起双手
    ctrl.set_targets({
        Joints.LeftShoulderPitch: -kPi_2,   # 左肩抬起 90°
        Joints.RightShoulderPitch: -kPi_2,  # 右肩抬起 90°
    })
    
    # 保持 5 秒
    time.sleep(5.0)

    # 回零并释放
    ctrl.set_targets({j: 0.0 for j in ctrl.arm_joints})
    time.sleep(2.0)
    ctrl.release_arm(duration=2.0)
    ctrl.stop()
    print("完成!")
```

---

## 9. IDL 数据结构

### 9.1 LowCmd_ — 低层指令消息 (unitree_hg, 35电机)

```python
class LowCmd_:
    mode_pr: uint8           # 控制模式: 0=PR(串联), 1=AB(并联)
    mode_machine: uint8      # 运动机器模式
    motor_cmd[35]: MotorCmd_ # 35个电机命令（实际用前 29+1 个）
    reserve[4]: uint32       # 预留
    crc: uint32              # ★ CRC32 校验码（必须正确）
```

### 9.2 LowState_ — 低层状态消息 (unitree_hg, 35电机)

```python
class LowState_:
    version[2]: uint32
    mode_pr: uint8
    mode_machine: uint8
    tick: uint32
    imu_state: IMUState_              # IMU 数据
    motor_state[35]: MotorState_      # 35个电机状态
    wireless_remote[40]: uint8         # 遥控器原始数据
    reserve[4]: uint32
    crc: uint32
```

### 9.3 MotorCmd_ — 单个电机命令

```python
class MotorCmd_:
    mode: uint8       # 0=Disable(失能), 1=Enable(使能)
    q: float32        # ★ 目标位置 (rad)
    dq: float32       # 目标速度 (rad/s)，通常填 0
    tau: float32      # 前馈力矩 (N·m)，通常填 0
    kp: float32       # ★ 位置比例增益 (N·m/rad)
    kd: float32       # ★ 速度阻尼增益 (N·m·s/rad)
    reserve: uint32
```

### 9.4 MotorState_ — 单个电机状态

```python
class MotorState_:
    mode: uint8           # 当前模式
    q: float32            # ★ 当前位置 (rad)
    dq: float32           # 当前速度 (rad/s)
    ddq: float32          # 当前加速度 (rad/s²)
    tau_est: float32       # 估算力矩 (N·m)
    temperature[2]: int16  # 温度 (×0.01°C)
    vol: float32           # 电压 (V)
    sensor[2]: uint32
    motorstate: uint32     # 电机状态位
    reserve[4]: uint32
```

### 9.5 IMUState_

```python
class IMUState_:
    quaternion[4]: float32   # ★ [w, x, y, z] 四元数（顺序注意！）
    gyro[3]: float32         # 陀螺仪 (rad/s) [x, y, z]
    accel[3]: float32        # 加速度计 (m/s²) [x, y, z]
    rpy[3]: float32          # 欧拉角 (rad) [roll, pitch, yaw]（ZYX 顺序）
    temperature: int16       # 温度
```

**注意**: `quaternion[0]=w`, `quaternion[1]=x`, `quaternion[2]=y`, `quaternion[3]=z`
这与 ROS 的 `[x, y, z, w]` 顺序不同！

### 9.6 G1 关节索引速查表 (29 DOF)

```
索引  关节名称              范围(rad)     说明
─────────────────────────────────────────────────────
 0    left_hip_pitch        [-2.53, 2.88] 左髋俯仰
 1    left_hip_roll         [-0.39, 0.39] 左髋横滚
 2    left_hip_yaw          [-0.57, 0.57] 左髋偏航
 3    left_knee             [-0.18, 2.62] 左膝关节
 4    left_ankle_pitch      [-1.04, 1.04] 左踝俯仰
 5    left_ankle_roll       [-0.39, 0.39] 左踝横滚
 6    right_hip_pitch       [-2.53, 2.88] 右髋俯仰
 7    right_hip_roll        [-0.39, 0.39] 右髋横滚
 8    right_hip_yaw         [-0.57, 0.57] 右髋偏航
 9    right_knee            [-0.18, 2.62] 右膝关节
10    right_ankle_pitch     [-1.04, 1.04] 右踝俯仰
11    right_ankle_roll      [-0.39, 0.39] 右踝横滚
12    waist_yaw             ±0.79        ★ 腰部偏航 (±45°)
13    waist_roll            锁定无效      腰部横滚 (29dof locked)
14    waist_pitch           锁定无效      腰部俯仰 (29dof locked)
15    left_shoulder_pitch   [-2.09, 2.09] 左肩俯仰
16    left_shoulder_roll    [-1.31, 1.31] 左肩横滚
17    left_shoulder_yaw     [-1.49, 1.49] 左肩偏航
18    left_elbow            [-2.44, 2.44] 左肘
19    left_wrist_roll       [-2.09, 2.09] 左腕横滚
20    left_wrist_pitch      23dof无效    左腕俯仰
21    left_wrist_yaw        23dof无效    左腕偏航
22    right_shoulder_pitch  [-2.09, 2.09] 右肩俯仰
23    right_shoulder_roll   [-1.31, 1.31] 右肩横滚
24    right_shoulder_yaw    [-1.49, 1.49] 右肩偏航
25    right_elbow           [-2.44, 2.44] 右肘
26    right_wrist_roll      [-2.09, 2.09] 右腕横滚
27    right_wrist_pitch     23dof无效    右腕俯仰
28    right_wrist_yaw       23dof无效    右腕偏航
29    ★ kNotUsedJoint       0 or 1       Arm SDK 使能标志位
```

---

## 10. 工具类库

| 类/函数 | 文件 | 用途 | 关键方法 |
|---------|------|------|---------|
| `RecurrentThread` | utils/thread.py | 定时循环线程 | `Start()`, `interval` |
| `CRC` | utils/crc.py | CRC32 校验 | `Crc(msg)`, 支持 go/hg |
| `BQueue` | utils/bqueue.py | 有界阻塞队列 | `Get(timeout)`, `Put()`, `Clear()` |
| `Future` | utils/future.py | 异步结果 | `DEFER->READY/FAILED`, `GetResult()` |
| `Singleton` | utils/singleton.py | 单例基类 | `_instance`, `_get_instance()` |
| `LeaseClient` | rpc/lease_client.py | RPC 租约客户端 | `LeaseInit()`, `LeaseGet()`, `LeasePut()` |
| `MotionSwitcherClient` | comm/.../... | 运动模式切换 | `CheckMode()`, `ReleaseMode()`, `SelectMode()` |

### RecurrentThread 使用详解

```python
from unitree_sdk2py.utils.thread import RecurrentThread

thread = RecurrentThread(
    interval=0.002,      # 控制间隔 (秒)
    target=my_func,      # 回调函数
    name="control_loop", # 线程名称
)
# thread.Start()         # 启动循环
# thread.Stop()          # 停止循环

def my_func():
    """被定时调用的函数"""
    pass
```

**注意**: `RecurrentThread` 使用 Linux `timerfd` 驱动，在 Windows 上可能不可用。
Windows 替代方案：使用标准 `threading.Thread` + `time.sleep()`。

---

## 11. 常见问题

### Q1: 为什么机器人没有反应？

**检查清单**：
1. 是否调用了 `ChannelFactoryInitialize()`？
2. 网络接口是否正确？（`eth0` / `wlan0` / `enp2s0`）
3. 底层控制是否先调用了 `MotionSwitcher.ReleaseMode()`？
4. CRC 是否每帧都计算了？
5. `motor_cmd[i].mode` 是否设为 1？
6. Arm SDK 模式是否设了 `motor_cmd[29].q = 1`？

### Q2: CRC 校验失败怎么办？

```python
from unitree_sdk2py.utils.crc import CRC
crc = CRC()

# 每帧发送前必须计算
low_cmd.crc = crc.Crc(low_cmd)
```

**常见错误**: 忘记计算 CRC，或修改了 low_cmd 后忘记重新计算。

### Q3: 如何知道机器人当前的运动模式？

```python
msc = MotionSwitcherClient()
msc.Init()
code, info = msc.CheckMode()
print(info['name'])  # "normal" / ""(空表示已释放)
```

### Q4: 机器人突然倒地？

可能原因：
- 控制频率太低（< 100Hz 对于底层控制）
- kp/kd 参数过大导致振荡
- 未 ReleaseMode 就直接发送底层指令
- 关节角度超出限位

**安全建议**：开发时始终有人在旁边准备按遥控器的急停键。

### Q5: Windows 下运行报错？

`RecurrentThread` 依赖 Linux timerfd，Windows 下需替换：

```python
# Windows 兼替方案
import threading

class WindowsLoop:
    def __init__(self, dt, target):
        self.dt = dt
        self.target = target
        self.running = False
        self.thread = None

    def Start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def Stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)

    def _loop(self):
        import time
        while self.running:
            self.target()
            time.sleep(self.dt)
```

### Q6: ROS2 节点中使用 SDK 怎么处理 DDS 冲突？

`unitree_sdk2py` 使用 CycloneDDS，而 ROS2 也可能用 DDS。两者不冲突，因为它们在不同的 Domain 上。但如果遇到问题，可以在单独的进程中运行 SDK 控制脚本，ROS2 节点通过话题与之通信。

---

## 附录 A: 项目中的集成方式

本项目中，SDK 主要在以下场景使用：

| 场景 | 文件 | 控制方式 | 通道 |
|------|------|---------|------|
| 腰部视觉伺服 | `waist_tracker.py` | Arm SDK | `rt/arm_sdk` |
| 手臂演示 | `src/arm.py` | Arm SDK | `rt/arm_sdk` |
| 机器人驱动 | `driver.py` | ROS2 Twist Bridge → unitree_api | `/api/sport/request` |

---

*最后更新: 2026-03*
