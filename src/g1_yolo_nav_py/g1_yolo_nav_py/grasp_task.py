#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 抓取任务主控程序
==================
一键执行：YOLO 检测 → 偏航对齐 → 前进接近 → 抓取 → 交互菜单

状态机：
    SEARCHING   → 旋转搜索目标
    ALIGNING    → 偏航对齐让目标居中
    APPROACHING → 前进到目标附近
    GRABBING    → 执行 armup.py 抓取
    MENU        → 交互菜单

控制方式：
    所有运动控制通过 SportClient 统一封装（/api/sport/request），
    全部使用 Sport API（MOVE/STOPMOVE/SIT 等），不使用 Loco API。
    启动时自动执行 FSM 初始化（DAMP → STANDUP → BALANCESTAND → CONTINUOUSGAIT）。

运行：
    ros2 run g1_yolo_nav_py grasp_task
    python3 -m g1_yolo_nav_py.grasp_task
"""

import rclpy
from rclpy.node import Node

from g1_yolo_nav_py._grasp_state import GraspStateMachineMixin, GraspState


class GraspTaskNode(Node, GraspStateMachineMixin):
    """G1 抓取任务主控节点 — 通过 SportClient (纯 Sport API) 统一控制运动。"""

    def __init__(self) -> None:
        Node.__init__(self, "g1_grasp_task_node")

        self._init_grasp_state(
            self,
            start_state=GraspState.SEARCHING,
        )

        # ---- 定时器（10Hz）----
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("=" * 50)
        self.get_logger().info("G1 抓取任务启动（纯 Sport API 模式）")
        self.get_logger().info(f"目标类别: {self._gs_target_class}")
        self.get_logger().info(f"armup: {self._gs_armup_script}")
        self.get_logger().info(f"armdown: {self._gs_armdown_script}")
        self.get_logger().info("=" * 50)

    # ------------------------------------------------------------------
    #  日志实现
    # ------------------------------------------------------------------
    def _log_info(self, msg: str) -> None:
        self.get_logger().info(msg)

    def _log_error(self, msg: str) -> None:
        self.get_logger().error(msg)

    # ------------------------------------------------------------------
    #  状态变化回调
    # ------------------------------------------------------------------
    def _on_state_changed(self, old_state: GraspState, new_state: GraspState) -> None:
        pass  # 终端模式无需额外 UI 更新

    # ------------------------------------------------------------------
    #  tick 转发
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        self._gs_tick()

    # ------------------------------------------------------------------
    #  清理
    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self._gs_destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
