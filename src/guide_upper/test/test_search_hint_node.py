"""上层搜索建议消费端测试。

验证三件事：
  1. 只在 valid=true 时才产出提示；
  2. 三种回归方向给出正确的中文提示；
  3. 同一提示在节流窗口内不重复输出。
"""

import pytest
import rclpy
from guide_interfaces.msg import GuideCommand, SearchHint

from guide_upper.search_hint_node import SearchHintNode


@pytest.fixture(scope='module')
def hint_node():
    """创建提示节点（模块级，整个文件共用一个 ROS 上下文）。"""
    if not rclpy.ok():
        rclpy.init()
    node = SearchHintNode()
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _make_hint(direction, distance=1.0, valid=True):
    """构造一条搜索建议。"""
    hint = SearchHint()
    hint.direction = direction
    hint.distance = distance
    hint.reason = GuideCommand.REASON_BLIND_LOST
    hint.valid = valid
    return hint


def test_build_text_left(hint_node):
    """向左回归的提示文本。"""
    text = hint_node.build_text(_make_hint(-1, 1.2))
    assert '向左' in text
    assert '1.2' in text


def test_build_text_right(hint_node):
    """向右回归的提示文本。"""
    text = hint_node.build_text(_make_hint(1, 0.8))
    assert '向右' in text
    assert '0.8' in text


def test_build_text_straight(hint_node):
    """直行回归的提示文本。"""
    text = hint_node.build_text(_make_hint(0, 2.0))
    assert '直行' in text
    assert '2.0' in text


def test_invalid_hint_is_ignored(hint_node):
    """valid=false 的建议不产生提示。"""
    hint_node.hint_count = 0
    hint_node.last_text = ''
    hint_node.last_time = None
    hint_node.on_hint(_make_hint(-1, 1.0, valid=False))
    assert hint_node.hint_count == 0


def test_valid_hint_is_emitted(hint_node):
    """valid=true 的建议产生提示。"""
    hint_node.hint_count = 0
    hint_node.last_text = ''
    hint_node.last_time = None
    hint_node.on_hint(_make_hint(-1, 1.0, valid=True))
    assert hint_node.hint_count == 1


def test_same_hint_is_throttled(hint_node):
    """同一条建议在节流窗口内只提示一次。"""
    hint_node.hint_count = 0
    hint_node.last_text = ''
    hint_node.last_time = None

    hint_node.on_hint(_make_hint(1, 1.0, valid=True))
    hint_node.on_hint(_make_hint(1, 1.0, valid=True))
    assert hint_node.hint_count == 1

    # 关掉节流后，同一条建议应能再次提示
    original = hint_node.throttle_sec
    hint_node.throttle_sec = 0.0
    try:
        hint_node.on_hint(_make_hint(1, 1.0, valid=True))
    finally:
        hint_node.throttle_sec = original
    assert hint_node.hint_count == 2
