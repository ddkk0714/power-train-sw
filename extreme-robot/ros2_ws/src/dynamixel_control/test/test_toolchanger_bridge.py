"""분리/교체 전에 기존 도구의 정지 실패를 확인하는 ROS 어댑터 검증."""

import json
from types import SimpleNamespace

from std_msgs.msg import String

from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge
from dynamixel_control.toolchanger_lock import ToolchangerLock
from dynamixel_control.toolchanger_sequence import ToolchangerSequence
from test_toolchanger_lock import IO


def _bridge(torque_state, zero_state='LOCKED', scope='END_EFFECTOR_ONLY'):
    """노드 생성 없이 잠금 명령 경로만 호출하는 최소 대역."""
    io = IO()
    lock = ToolchangerLock(io)
    lock.capture_zero(zero_state)
    lock.enable()
    replies = []
    bridge = SimpleNamespace(
        toolchanger=lock, read_only=False, emergency_stop_active=False,
        control_scope=scope, control_mode='MANUAL', mock_mode=True,
        tool_ids=[5], _tool_samples={5: {'torque_state': torque_state}},
        _stop_tool=lambda reason: None,
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
        toolchanger_status_pub=SimpleNamespace(publish=lambda message:
                                               replies.append(json.loads(message.data))))
    bridge._prepare_tool_for_lock = lambda: MoveItDynamixelBridge._prepare_tool_for_lock(bridge)
    bridge._toolchanger_snapshot = lambda: MoveItDynamixelBridge._toolchanger_snapshot(bridge)
    bridge.lock_sequence = ToolchangerSequence(lock, bridge._prepare_tool_for_lock)
    return bridge, io, replies


def _send(bridge, payload):
    MoveItDynamixelBridge._toolchanger_command(bridge, String(data=json.dumps(payload)))


def test_failed_tool_stop_prevents_lock_motor_goal():
    bridge, io, replies = _bridge('ON')
    before = list(io.writes)
    _send(bridge, {'command': 'UNLOCK', 'direction': 1})
    assert io.writes == before
    assert replies[-1]['state'] == 'LOCKED'
    assert 'OFF' in replies[-1]['command_error']


def test_change_sequence_requires_confirmed_tool_stop():
    bridge, io, replies = _bridge('ON', zero_state='UNLOCKED')
    before = list(io.writes)
    _send(bridge, {'command': 'CHANGE', 'to': 'LOCKED', 'direction': 1})
    assert io.writes == before
    assert 'OFF' in replies[-1]['command_error']
    assert replies[-1]['sequence_state'] == 'IDLE'


def test_change_sequence_is_rejected_outside_end_effector_bench():
    bridge, io, replies = _bridge('OFF', zero_state='UNLOCKED', scope='FULL_ROBOT')
    before = list(io.writes)
    _send(bridge, {'command': 'CHANGE', 'to': 'LOCKED', 'direction': 1})
    assert io.writes == before
    assert 'MANUAL END_EFFECTOR_ONLY' in replies[-1]['command_error']
    assert replies[-1]['sequence_state'] == 'IDLE'


def test_unknown_change_target_is_rejected_without_lock_write():
    bridge, io, replies = _bridge('OFF', zero_state='UNLOCKED')
    before = list(io.writes)
    _send(bridge, {'command': 'CHANGE', 'to': 'HALF', 'direction': 1})
    assert io.writes == before
    assert 'LOCKED or UNLOCKED' in replies[-1]['command_error']
