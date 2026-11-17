"""执行层行为测试。

覆盖"方案 A 契约"里最关键的两条：
  1. 只认 command，忽略 search_direction / regression_distance 等辅助字段；
  2. 上游断流时，看门狗自行停车。

这些逻辑以前只能靠手工注入指令验证，现在固化成回归测试。
"""

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from guide_interfaces.msg import GuideCommand

from guide_executor.executor_node import ExecutorNode


@pytest.fixture(scope='module')
def executor():
    """创建执行层节点（模块级，整个文件共用一个 ROS 上下文）。"""
    if not rclpy.ok():
        rclpy.init()
    node = ExecutorNode()
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _spin_for(node, seconds):
    """在给定时长内持续 spin 节点。"""
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


# ============================================================
# 一、command -> 速度 映射
# ============================================================
def test_map_forward(executor):
    """MODE_FORWARD -> 正常直行速度。"""
    v, w = executor.map_command(GuideCommand.MODE_FORWARD)
    assert v == pytest.approx(executor.max_speed)
    assert w == pytest.approx(0.0)


def test_map_left_and_right(executor):
    """MODE_LEFT/RIGHT -> 对应符号的角速度。"""
    v, w = executor.map_command(GuideCommand.MODE_LEFT)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(executor.turn_speed)

    v, w = executor.map_command(GuideCommand.MODE_RIGHT)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(-executor.turn_speed)


def test_map_stop(executor):
    """MODE_STOP -> 零速。"""
    v, w = executor.map_command(GuideCommand.MODE_STOP)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(0.0)


def test_unknown_command_is_stop(executor):
    """未知 command 按停车处理（fail-safe，宁停勿猜）。"""
    v, w = executor.map_command(200)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(0.0)


def test_caution_keeps_last_motion_direction(executor):
    """MODE_CAUTION 沿用最近一次运动方向，降速一半。"""
    executor.last_motion = GuideCommand.MODE_LEFT
    v, w = executor.map_command(GuideCommand.MODE_CAUTION)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(executor.turn_speed * executor.caution_ratio)

    executor.last_motion = GuideCommand.MODE_FORWARD
    v, w = executor.map_command(GuideCommand.MODE_CAUTION)
    assert v == pytest.approx(executor.caution_speed)


# ============================================================
# 二、加速度限幅
# ============================================================
def test_apply_ramp_limits_change_per_cycle(executor):
    """单周期速度变化不超过 max_accel * dt。"""
    executor.cur_v = 0.0
    executor.cur_w = 0.0
    dt = 0.02
    v, w = executor.apply_ramp(10.0, 10.0, dt)
    assert v == pytest.approx(executor.max_accel * dt)
    assert w == pytest.approx(executor.max_angular_accel * dt)


def test_apply_ramp_converges_to_target(executor):
    """持续逼近目标速度，最终到达目标。"""
    executor.cur_v = 0.0
    executor.cur_w = 0.0
    dt = 0.02
    for _ in range(500):
        v, w = executor.apply_ramp(executor.max_speed, 0.0, dt)
    assert v == pytest.approx(executor.max_speed)


# ============================================================
# 三、集成：指令驱动速度 + 看门狗停车
# ============================================================
def test_command_drives_velocity_then_watchdog_stops(executor):
    """收到 FORWARD 后输出前进速度；停止下发后看门狗自行停车。"""
    peer = rclpy.create_node('executor_test_peer')
    publisher = peer.create_publisher(GuideCommand, '/guide_command', 10)
    received = []
    peer.create_subscription(Twist, '/chassis/velocity', received.append, 10)

    cmd = GuideCommand()
    cmd.command = GuideCommand.MODE_FORWARD
    # 故意带上"给上层的建议"字段：执行层必须忽略它们
    cmd.search_direction = -1
    cmd.regression_distance = 5.0
    cmd.reason = GuideCommand.REASON_BLIND_LOST

    try:
        # 等 DDS 发现
        deadline = time.time() + 5.0
        while publisher.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(peer, timeout_sec=0.05)
            rclpy.spin_once(executor, timeout_sec=0.01)
        assert publisher.get_subscription_count() > 0, '执行层未订阅 /guide_command'

        # 持续下发，直到看到前进速度
        moving = False
        deadline = time.time() + 5.0
        while time.time() < deadline and not moving:
            publisher.publish(cmd)
            rclpy.spin_once(executor, timeout_sec=0.02)
            rclpy.spin_once(peer, timeout_sec=0.01)
            moving = any(t.linear.x > 0.05 for t in received)
        assert moving, '执行层未输出前进速度'

        # 停止下发：看门狗应在 watchdog_timeout 内把速度归零
        index = len(received)
        stopped = False
        deadline = time.time() + executor.watchdog_timeout + 3.0
        while time.time() < deadline and not stopped:
            rclpy.spin_once(executor, timeout_sec=0.02)
            rclpy.spin_once(peer, timeout_sec=0.01)
            if len(received) > index:
                latest = received[-1]
                stopped = (abs(latest.linear.x) < 1e-6 and abs(latest.angular.z) < 1e-6)
        assert stopped, '指令断流后看门狗未触发停车'
    finally:
        peer.destroy_node()
