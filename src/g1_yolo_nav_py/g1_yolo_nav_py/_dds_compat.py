"""
DDS 兼容层 — 解决 unitree_sdk2py (CycloneDDS) 与 ROS2 (CycloneDDS) 同进程 domain 冲突。

问题：
    unitree_sdk2py 的 ChannelFactoryInitialize 和 ROS2 的 rclpy.init()
    都基于 CycloneDDS，同一进程中后创建的实例会报 "create domain error"。

方案：
    1. 先在「干净」环境下调用 ChannelFactoryInitialize（此时 CYCLONEDDS_URI 未设置，
       unitree SDK 使用默认 DomainId 0 与机器人通信）
    2. 然后设置 CYCLONEDDS_URI 让 ROS2 的 rclpy.init() 使用不同的 DomainId，
       避免与已创建的 unitree SDK domain 冲突

使用：
    from g1_yolo_nav_py._dds_compat import init_unitree_dds_before_ros2

    def main(args=None):
        init_unitree_dds_before_ros2(iface="eth0")
        rclpy.init(args=args)
        ...

注意：
    - 必须在 rclpy.init() 之前调用
    - 如果 ROS2 也需要与外部 DDS 设备通信（如其他 ROS2 节点），
      则需要确保 ROS_DOMAIN_ID 环境变量与 CYCLONEDDS_URI 中的 DomainId 一致
"""

import os
import sys
import tempfile

# CycloneDDS XML 配置模板 — 为 ROS2 使用独立的 DomainId
# unitree SDK 在 ChannelFactoryInitialize(0, iface) 时会创建自己的 domain，
# 这里给 ROS2 的 CycloneDDS 配置一个不同的 domain id 避免冲突
_CYCLONEDDS_XML_TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain Id="{domain_id}">
  </Domain>
</CycloneDDS>
"""

# 标记是否已初始化
_dds_initialized = False


def init_unitree_dds_before_ros2(iface: str = "") -> bool:
    """
    在 rclpy.init() 之前初始化 unitree SDK DDS。

    关键：必须先调用 ChannelFactoryInitialize，再设置 CYCLONEDDS_URI。
    如果反过来，unitree SDK 创建 CycloneDDS domain 时会读到错误的配置。

    Args:
        iface: 网络接口名（空=自动检测）

    Returns:
        True 如果 DDS 初始化成功，False 否则
    """
    global _dds_initialized
    if _dds_initialized:
        return True

    # 1. 先初始化 unitree SDK（在 CYCLONEDDS_URI 未设置时，使用默认 DomainId 0）
    dds_ok = False
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, iface)
        _dds_initialized = True
        dds_ok = True
    except Exception as e:
        print(f"[WARNING] ChannelFactoryInitialize 失败: {e}", file=sys.stderr)

    # 2. 只有 unitree SDK 初始化成功时，才设置 CYCLONEDDS_URI 让 ROS2 使用 DomainId 1
    #    如果 ChannelFactoryInitialize 失败，不修改 ROS2 的 domain 配置，
    #    保持 ROS2 在默认 Domain 0 上通信（否则收不到 Domain 0 的相机等数据）
    if dds_ok:
        _xml_content = _CYCLONEDDS_XML_TEMPLATE.format(domain_id=1)
        try:
            _xml_path = os.path.join(tempfile.gettempdir(), "ros2_cyclonedds_domain1.xml")
            with open(_xml_path, "w") as f:
                f.write(_xml_content)
            os.environ["CYCLONEDDS_URI"] = f"file://{_xml_path}"
        except Exception:
            os.environ["CYCLONEDDS_URI"] = _xml_content.strip()

        # 同时设置 ROS_DOMAIN_ID 与 CYCLONEDDS_URI 保持一致
        os.environ.setdefault("ROS_DOMAIN_ID", "1")

    return dds_ok
