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

运行：
    ros2 run g1_yolo_nav_py grasp_task
    python3 -m g1_yolo_nav_py.grasp_task
"""

# ==================================================================
# 1. 标准库导入
# ==================================================================
import os          # 环境变量与路径操作
import sys         # 命令行参数与 sys.executable
import time        # 计时与延时
import subprocess  # 子进程执行 armup.py / armdown.py
import math        # 角度弧度转换
import threading   # 后台线程运行菜单与抓取
from pathlib import Path  # arm 脚本路径构造
from enum import Enum, auto  # 状态机枚举定义

# ==================================================================
# 2. 第三方库与 ROS2 导入
# ==================================================================
import rclpy  # ROS2 Python 客户端库
from rclpy.node import Node  # ROS2 节点基类
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # QoS 配置
from geometry_msgs.msg import Twist  # 速度指令消息
from vision_msgs.msg import Detection2DArray  # 2D 检测结果消息

# unitree_sdk2py: 宇树机器人底层 SDK（可选依赖）
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # DDS 通信工厂初始化
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient  # G1 运动控制客户端
    LOCO_AVAILABLE = True
except ImportError:
    LOCO_AVAILABLE = False


class State(Enum):
    SEARCHING = auto()
    ALIGNING = auto()
    APPROACHING = auto()
    GRABBING = auto()
    MENU = auto()
    DONE = auto()


# arm 脚本默认目录（ros2 run 时需要通过参数指定，因为 __file__ 指向 install 目录）
_DEFAULT_ARM_DIR = os.path.expanduser("~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm")


class GraspTaskNode(Node):
    """G1 抓取任务主控节点 — 自动检测、接近、抓取、交互菜单。"""

    def __init__(self) -> None:
        super().__init__("g1_grasp_task_node")

        # ---- 参数 ----
        self.declare_parameter("detection_topic", "/g1/vision/detections")
        self.declare_parameter("target_class_id", "chair")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("camera_fov_deg", 87.0)
        self.declare_parameter("yaw_kp", 2.0)
        self.declare_parameter("max_yaw_speed", 0.6)
        self.declare_parameter("center_tolerance", 0.08)
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("arrive_bbox_ratio", 0.45)
        self.declare_parameter("align_stable_time", 1.0)
        self.declare_parameter("lost_timeout", 2.0)
        self.declare_parameter("search_yaw_speed", 0.3)
        self.declare_parameter("network_interface", "")
        self.declare_parameter("arm_script_dir", _DEFAULT_ARM_DIR)

        p = lambda n: self.get_parameter(n).value
        self._det_topic = p("detection_topic")
        self._target_class = p("target_class_id")
        self._cmd_topic = p("cmd_vel_topic")
        self._fov_rad = math.radians(p("camera_fov_deg"))
        self._kp = float(p("yaw_kp"))
        self._max_yaw = float(p("max_yaw_speed"))
        self._center_tol = float(p("center_tolerance"))
        self._fwd_speed = float(p("forward_speed"))
        self._arrive_ratio = float(p("arrive_bbox_ratio"))
        self._stable_time = float(p("align_stable_time"))
        self._lost_timeout = float(p("lost_timeout"))
        self._search_speed = float(p("search_yaw_speed"))
        self._net_iface = p("network_interface")

        # arm 脚本路径（通过参数指定，默认 ~/g1act_ws/manact_ws/src/g1_yolo_nav_py/arm/）
        arm_dir = p("arm_script_dir")
        self._armup_script = Path(arm_dir) / "armup.py"
        self._armdown_script = Path(arm_dir) / "armdown.py"

        # ---- ROS2 订阅 ----
        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.create_subscription(Detection2DArray, self._det_topic,
                                 self._on_detection, det_qos)

        # ---- ROS2 发布 ----
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

        # ---- LocoClient ----
        self._loco = None
        if LOCO_AVAILABLE:
            self._init_loco()

        # ---- 内部状态 ----
        self._target_u = None
        self._bbox_size_x = 0.0
        self._bbox_size_y = 0.0
        self._last_detect_time = 0.0
        self._align_start = None

        self._state = State.SEARCHING

        # ---- 定时器（10Hz）----
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("=" * 50)
        self.get_logger().info("G1 抓取任务启动")
        self.get_logger().info(f"目标类别: {self._target_class}")
        self.get_logger().info(f"armup: {self._armup_script}")
        self.get_logger().info(f"armdown: {self._armdown_script}")
        self.get_logger().info("=" * 50)

    # ==================================================================
    #  LocoClient
    # ==================================================================
    def _init_loco(self) -> None:
        try:
            # ChannelFactoryInitialize 已在 main() 中提前调用（先于 rclpy.init）
            self._loco = LocoClient()
            self._loco.SetTimeout(5.0)
            self._loco.Init()
            self.get_logger().info("[LocoClient] 初始化成功")
        except Exception as e:
            self.get_logger().error(f"[LocoClient] 初始化失败: {e}")

    def _loco_move(self, vx=0.0, vy=0.0, vyaw=0.0, continuous=False) -> None:
        if self._loco is None:
            return
        try:
            self._loco.Move(vx=vx, vy=vy, vyaw=vyaw, continuous=continuous)
        except Exception as e:
            self.get_logger().warn(f"[LocoClient] Move 失败: {e}")

    def _loco_stop(self) -> None:
        if self._loco is None:
            return
        try:
            self._loco.StopMove()
        except Exception as e:
            self.get_logger().warn(f"[LocoClient] StopMove 失败: {e}")

    # ==================================================================
    #  cmd_vel 发布
    # ==================================================================
    def _publish_cmd(self, vx=0.0, vy=0.0, vz=0.0) -> None:
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = vz
        self._cmd_pub.publish(cmd)

    def _publish_stop(self) -> None:
        self._publish_cmd(0, 0, 0)

    # ==================================================================
    #  检测回调
    # ==================================================================
    def _on_detection(self, msg: Detection2DArray) -> None:
        best_det = None
        best_score = 0.0
        for det in msg.detections:
            if det.results and det.results[0].id == self._target_class:
                if det.results[0].score > best_score:
                    best_score = det.results[0].score
                    best_det = det

        now = time.time()
        if best_det is not None:
            bbox = best_det.bbox
            self._target_u = bbox.center.position.x
            self._bbox_size_x = bbox.size_x
            self._bbox_size_y = bbox.size_y
            self._last_detect_time = now

            # 首次识别到目标时打印日志
            if self._state == State.SEARCHING:
                self.get_logger().info(
                    f"[检测] 识别到目标: {self._target_class}, "
                    f"置信度={best_score:.1%}, u={self._target_u:.3f}"
                )
        else:
            self._target_u = None

    # ==================================================================
    #  状态机 — 主循环
    # ==================================================================
    def _tick(self) -> None:
        if self._state == State.SEARCHING:
            self._tick_searching()
        elif self._state == State.ALIGNING:
            self._tick_aligning()
        elif self._state == State.APPROACHING:
            self._tick_approaching()
        # GRABBING 和 MENU 由子流程处理，不在 tick 中

    def _tick_searching(self) -> None:
        """旋转搜索目标。"""
        if self._target_u is not None:
            self.get_logger().info("[状态] SEARCHING → ALIGNING: 目标已找到")
            self._state = State.ALIGNING
            self._align_start = None
            self._publish_stop()
            return

        # 旋转搜索
        self._publish_cmd(vz=self._search_speed)

    def _tick_aligning(self) -> None:
        """偏航对齐让目标居中。"""
        now = time.time()

        # 目标丢失 → 回搜索
        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            self.get_logger().warn("[状态] ALIGNING → SEARCHING: 目标丢失")
            self._state = State.SEARCHING
            self._publish_stop()
            self._align_start = None
            return

        error = self._target_u - 0.5

        # 居中 → 切换到前进
        if abs(error) < self._center_tol:
            if self._align_start is None:
                self._align_start = now
            if now - self._align_start >= self._stable_time:
                self.get_logger().info(
                    "[状态] ALIGNING → APPROACHING: 目标已居中"
                )
                self._state = State.APPROACHING
                self._publish_stop()
                return
        else:
            self._align_start = None

        # P 控制偏航
        vyaw = self._kp * error * self._fov_rad
        vyaw = max(-self._max_yaw, min(self._max_yaw, vyaw))
        self._publish_cmd(vz=vyaw)

    def _tick_approaching(self) -> None:
        """前进到目标附近。"""
        now = time.time()

        # 目标丢失 → 回搜索
        if self._target_u is None or (now - self._last_detect_time > self._lost_timeout):
            self.get_logger().warn("[状态] APPROACHING → SEARCHING: 目标丢失")
            self._state = State.SEARCHING
            self._loco_stop()
            return

        # 偏离中心 → 回对齐
        if abs(self._target_u - 0.5) > self._center_tol * 2:
            self.get_logger().info("[状态] APPROACHING → ALIGNING: 目标偏离中心")
            self._state = State.ALIGNING
            self._loco_stop()
            self._align_start = None
            return

        # 到达判断
        bbox_max = max(self._bbox_size_x, self._bbox_size_y)
        if bbox_max >= self._arrive_ratio:
            self._loco_stop()
            self.get_logger().info(
                f"[状态] APPROACHING → GRABBING: 到达目标! "
                f"bbox={bbox_max:.2f} >= {self._arrive_ratio}"
            )
            self._state = State.GRABBING
            self._run_grab()
            return

        # 前进
        self._loco_move(vx=self._fwd_speed, continuous=True)

    # ==================================================================
    #  抓取（执行 armup.py）
    # ==================================================================
    def _run_grab(self) -> None:
        """执行 armup.py 抓取目标物。"""
        script = str(self._armup_script)
        if not Path(script).exists():
            self.get_logger().error(f"[抓取] armup.py 不存在: {script}")
            self._state = State.MENU
            self._show_menu()
            return

        self.get_logger().info(f"[抓取] 执行 armup.py ...")

        def _worker():
            try:
                args = [sys.executable, script]
                if self._net_iface:
                    args.append(self._net_iface)
                # 通过 stdin 管道自动确认 armup.py 的 input() 提示
                proc = subprocess.run(
                    args, check=True, input=b"\n",
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                )
                if proc.stdout:
                    for line in proc.stdout.decode(errors="replace").splitlines():
                        self.get_logger().info(f"[armup] {line}")
                self.get_logger().info("[抓取] armup.py 执行完成")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"[抓取] armup.py 执行失败: 返回码={e.returncode}")
            except Exception as e:
                self.get_logger().error(f"[抓取] armup.py 异常: {e}")
            finally:
                self._state = State.MENU
                self._show_menu()

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    # ==================================================================
    #  交互菜单
    # ==================================================================
    def _show_menu(self) -> None:
        """显示交互菜单（在后台线程中运行，不阻塞 ROS2 spin）。"""
        def _menu_loop():
            while self._state == State.MENU:
                print("\n" + "=" * 40)
                print("  G1 抓取任务 — 操作菜单")
                print("=" * 40)
                print("  1. 放下目标物")
                print("  2. 右转放下目标物")
                print("  3. 自定义控制（输入 xyz）")
                print("  4. 退出")
                print("=" * 40)

                try:
                    choice = input("请选择 [1-4]: ").strip()
                except EOFError:
                    choice = "4"

                if choice == "1":
                    self._do_put_down()
                elif choice == "2":
                    self._do_turn_and_put_down()
                elif choice == "3":
                    self._do_custom_control()
                elif choice == "4":
                    self.get_logger().info("[退出] 任务结束")
                    self._state = State.DONE
                    rclpy.shutdown()
                    return
                else:
                    print("无效选择，请输入 1-4")

        t = threading.Thread(target=_menu_loop, daemon=True)
        t.start()

    def _do_put_down(self) -> None:
        """放下目标物。"""
        script = str(self._armdown_script)
        if not Path(script).exists():
            self.get_logger().error(f"[放下] armdown.py 不存在: {script}")
            return
        self.get_logger().info("[放下] 执行 armdown.py ...")
        try:
            args = [sys.executable, script]
            if self._net_iface:
                args.append(self._net_iface)
            # 通过 stdin 管道自动确认 armdown.py 的 input() 提示
            proc = subprocess.run(
                args, check=True, input=b"\n",
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            )
            if proc.stdout:
                for line in proc.stdout.decode(errors="replace").splitlines():
                    self.get_logger().info(f"[armdown] {line}")
            self.get_logger().info("[放下] armdown.py 执行完成")
        except Exception as e:
            self.get_logger().error(f"[放下] armdown.py 失败: {e}")

    def _do_turn_and_put_down(self) -> None:
        """右转 90° 后放下目标物。"""
        self.get_logger().info("[右转] 开始右转 90° ...")
        if self._loco is not None:
            try:
                self._loco.Move(vx=0.0, vy=0.0, vyaw=-1.0, continuous=True)
                time.sleep(1.6)  # ≈ 90° at 1.0 rad/s
                self._loco.StopMove()
                self.get_logger().info("[右转] 右转完成")
            except Exception as e:
                self.get_logger().warn(f"[右转] LocoClient 右转失败: {e}")
        else:
            self._publish_cmd(vz=-0.6)
            time.sleep(2.6)  # ≈ 90° at 0.6 rad/s
            self._publish_stop()
            self.get_logger().info("[右转] 右转完成（cmd_vel）")

        self._do_put_down()

    def _do_custom_control(self) -> None:
        """自定义控制：输入 x y z 控制机器人行动。"""
        print("\n  自定义控制模式")
        print("  输入 x y z（m/s, m/s, rad/s），例如: 0.2 0.0 0.3")
        print("  输入 q 返回菜单")
        print("-" * 40)

        while self._state == State.MENU:
            try:
                line = input("xyz> ").strip()
            except EOFError:
                break

            if line.lower() == "q":
                self._publish_stop()
                if self._loco:
                    self._loco_stop()
                print("  已停止运动，返回菜单")
                return

            parts = line.split()
            if len(parts) != 3:
                print("  格式错误，请输入: x y z（三个数值）")
                continue

            try:
                vx = float(parts[0])
                vy = float(parts[1])
                vz = float(parts[2])
            except ValueError:
                print("  格式错误，请输入数字")
                continue

            # 安全限幅
            vx = max(-1.0, min(1.0, vx))
            vy = max(-0.5, min(0.5, vy))
            vz = max(-1.0, min(1.0, vz))

            self.get_logger().info(f"[自定义] cmd: vx={vx}, vy={vy}, vz={vz}")
            self._publish_cmd(vx=vx, vy=vy, vz=vz)

            if self._loco and (abs(vx) > 0.01 or abs(vy) > 0.01):
                self._loco_move(vx=vx, vy=vy, vyaw=vz, continuous=True)

    def destroy_node(self) -> None:
        self._publish_stop()
        if self._loco:
            self._loco_stop()
        self.get_logger().info("[清理] 抓取任务节点已停止")
        super().destroy_node()


def main(args=None):
    _iface = ""
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        _iface = sys.argv[1]

    # 在 rclpy.init() 之前初始化 unitree DDS（CycloneDDS 兼容层）
    from g1_yolo_nav_py._dds_compat import init_unitree_dds_before_ros2
    init_unitree_dds_before_ros2(_iface)

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
