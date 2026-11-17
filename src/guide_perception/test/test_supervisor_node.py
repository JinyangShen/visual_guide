"""安全仲裁优先级测试。

安全仲裁的核心是"优先级"：任何时候都只能有一个理由生效。
这里把各传感器状态直接注入节点内部缓存，然后调用 decide() 检查仲裁结果，
覆盖七级优先级与几条关键交互（超声波压过视觉障碍、转向时不降速等）。
"""

import pytest
import rclpy
from guide_interfaces.msg import GuideCommand, PerceptionState

from guide_perception.safety_supervisor_node import SafetySupervisor


@pytest.fixture(scope='module')
def supervisor():
    """创建仲裁节点（模块级，整个文件共用一个 ROS 上下文）。"""
    if not rclpy.ok():
        rclpy.init()
    node = SafetySupervisor()
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _set_perception(node, **kwargs):
    """注入一份"感知正常"的视觉结果。"""
    p = PerceptionState()
    p.image_valid = True
    p.blind_path_found = True
    p.lateral_error_m = 0.0
    p.confidence = 0.9
    for key, value in kwargs.items():
        setattr(p, key, value)
    node.perception = p
    node.perception_time = node.get_clock().now()


def _set_ultrasonic(node, distance):
    """注入一个超声波测距值。"""
    node.ultra_dist = distance
    node.ultra_time = node.get_clock().now()


# ============================================================
# 一、优先级 1：感知链路失效
# ============================================================
def test_perception_timeout_gives_emergency_stop(supervisor):
    """感知数据超时 -> 急停（emergency=True）。"""
    supervisor.perception = None
    supervisor.perception_time = None

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_STOP
    assert cmd.reason == GuideCommand.REASON_PERCEPTION_TIMEOUT
    assert cmd.emergency is True


# ============================================================
# 二、优先级 2：超声波急停（压过视觉障碍）
# ============================================================
def test_ultrasonic_emergency_stop(supervisor):
    """超声波 < 0.2m -> 急停。"""
    _set_perception(supervisor)
    _set_ultrasonic(supervisor, 0.1)

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_STOP
    assert cmd.reason == GuideCommand.REASON_ULTRASONIC
    assert cmd.emergency is True
    assert cmd.ultrasonic_override is True


def test_ultrasonic_beats_visual_obstacle(supervisor):
    """超声波近距的优先级高于视觉障碍物。"""
    _set_perception(supervisor, obstacle_near=True)
    _set_ultrasonic(supervisor, 0.1)

    cmd = supervisor.decide()
    assert cmd.reason == GuideCommand.REASON_ULTRASONIC


# ============================================================
# 三、优先级 3：视觉障碍物
# ============================================================
def test_visual_obstacle_stops(supervisor):
    """视觉检测到障碍物 -> 停车（非急停）。"""
    _set_perception(supervisor, obstacle_near=True)
    _set_ultrasonic(supervisor, 1.5)

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_STOP
    assert cmd.reason == GuideCommand.REASON_DNN_OBSTACLE
    assert cmd.emergency is False


# ============================================================
# 四、优先级 4：盲道丢失
# ============================================================
def test_blind_path_lost_stops(supervisor):
    """盲道完全丢失 -> 停车。"""
    _set_perception(supervisor, blind_path_found=False, blind_path_lost=True)
    _set_ultrasonic(supervisor, 1.5)

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_STOP
    assert cmd.reason == GuideCommand.REASON_BLIND_LOST


# ============================================================
# 五、优先级 5~7：正常循迹
# ============================================================
def test_centred_error_goes_forward(supervisor):
    """横向偏差在死区内 -> 直行。"""
    _set_perception(supervisor, lateral_error_m=0.05)
    _set_ultrasonic(supervisor, 1.5)

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_FORWARD


def test_negative_error_turns_left(supervisor):
    """偏差为负 -> 左转。"""
    _set_perception(supervisor, lateral_error_m=-0.5)
    _set_ultrasonic(supervisor, 1.5)

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_LEFT


def test_positive_error_turns_right(supervisor):
    """偏差为正 -> 右转。"""
    _set_perception(supervisor, lateral_error_m=0.5)
    _set_ultrasonic(supervisor, 1.5)

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_RIGHT


def test_mid_range_ultrasonic_cautions_on_forward(supervisor):
    """超声波中距且打算直行 -> 降速慢行。"""
    _set_perception(supervisor, lateral_error_m=0.0)
    _set_ultrasonic(supervisor, 0.3)

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_CAUTION
    assert cmd.ultrasonic_override is True


def test_mid_range_ultrasonic_keeps_turning(supervisor):
    """超声波中距但正在转向避让 -> 保持转向，不降速。"""
    _set_perception(supervisor, lateral_error_m=-0.5)
    _set_ultrasonic(supervisor, 0.3)

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_LEFT


def test_stale_ultrasonic_is_ignored(supervisor):
    """超声波数据超时 -> 视为无效，不参与降速。"""
    _set_perception(supervisor, lateral_error_m=0.0)
    supervisor.ultra_dist = 0.3
    supervisor.ultra_time = None  # 从未收到，等价于超时

    cmd = supervisor.decide()
    assert cmd.command == GuideCommand.MODE_FORWARD
    assert cmd.ultrasonic_valid is False
