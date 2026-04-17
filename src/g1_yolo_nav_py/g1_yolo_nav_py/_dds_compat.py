"""
DDS 兼容层 — 解决 unitree_sdk2py (CycloneDDS) 与 ROS2 (CycloneDDS) 同进程 domain 冲突。

问题：
    unitree_sdk2py 的 ChannelFactoryInitialize 和 ROS2 的 rclpy.init()
    都基于 CycloneDDS，同一进程中后创建的实例会报 "create domain error"。

方案：
    在 rclpy.init() 之前调用 ChannelFactoryInitialize，
    并通过 CYCLONEDDS_URI 配置让两者使用不同的 DDS DomainId。
    - unitree SDK 使用 DomainId 0（与机器人通信）
    - ROS2 使用 DomainId 1（不与机器人 DDS 冲突即可）

使用：
    from _dds_compat import init_unitree_dds_before_ros2

    def main(args=None):
        init_unitree_dds_before_ros2(iface="eth0")
        rclpy.init(args=args)
        ...

注意：
    - 必须在 rclpy.init() 之前调用
    - 如果 ROS2 也需要与外部 DDS 设备通信（如其他 ROS2 节点），
      则需要确保 ROS2_DOMAIN_ID 环境变量与 CYCLONEDDS_URI 中的 DomainId 一致
"""

import os
import sys
import tempfile
from pathlib import Path

# CycloneDDS XML 配置模板 — 为 ROS2 使用独立的 DomainId
# unitree SDK 在 ChannelFactoryInitialize(0, iface) 时会创建自己的 domain，
# 这里给 ROS2 的 CycloneDDS 配置一个不同的 domain id 避免冲突
_CYCLONEDDS_XML_TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain>
    <Id>{domain_id}</Id>
  </Domain>
</CycloneDDS>
"""

# 标记是否已初始化
_dds_initialized = False


def init_unitree_dds_before_ros2(iface: str = "") -> bool:
    """
    在 rclpy.init() 之前初始化 unitree SDK DDS。

    做两件事：
    1. 设置 CYCLONEDDS_URI 让 ROS2 使用 DomainId 1（避免与 unitree SDK 的 DomainId 0 冲突）
    2. 调用 ChannelFactoryInitialize 初始化 unitree DDS

    Args:
        iface: 网络接口名（空=自动检测）

    Returns:
        True 如果 DDS 初始化成功，False 否则
    """
    global _dds_initialized
    if _dds_initialized:
        return True

    # 1. 写 CycloneDDS 配置文件给 ROS2 用（DomainId 1）
    #    unitree SDK 的 ChannelFactoryInitialize 会在内部创建自己的 CycloneDDS domain
    _xml_content = _CYCLONEDDS_XML_TEMPLATE.format(domain_id=1)
    try:
        _xml_path = os.path.join(tempfile.gettempdir(), "ros2_cyclonedds_domain1.xml")
        with open(_xml_path, "w") as f:
            f.write(_xml_content)
        os.environ["CYCLONEDDS_URI"] = f"file://{_xml_path}"
    except Exception:
        # 写文件失败就用内联 XML
        os.environ["CYCLONEDDS_URI"] = _xml_content.strip()

    # 同时设置 ROS2_DOMAIN_ID 与 CYCLONEDDS_URI 保持一致
    os.environ.setdefault("ROS_DOMAIN_ID", "1")

    # 2. 尝试初始化 unitree SDK
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, iface)
        _dds_initialized = True
        return True
    except Exception as e:
        print(f"[WARNING] ChannelFactoryInitialize 失败: {e}", file=sys.stderr)
        # 即使 unitree SDK 初始化失败，ROS2 的 DomainId 已改，
        # 至少不会因为 domain 冲突而崩溃
        return False
