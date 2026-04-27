"""可视化工具 — 共享颜色表与绘制函数。

control_panel.py 和 detection_visualizer.py 共用的可视化常量。
"""

from typing import Tuple  # 类型注解

# 预定义颜色表 (BGR) — 与设计系统功能色对应
COLORS = [
    (178, 145, 8),     # Teal-600 BGR
    (246, 130, 59),    # Blue-500 BGR
    (68, 68, 239),     # Red-500 BGR
    (11, 158, 245),    # Amber-500 BGR
    (129, 182, 16),    # Emerald-500 BGR
    (246, 92, 139),    # Violet-500 BGR
]


def get_color(class_id: str) -> Tuple[int, int, int]:
    """根据类别 ID 获取稳定颜色（相同 ID 始终返回相同颜色）。"""
    idx = hash(class_id) % len(COLORS)
    return COLORS[idx]
