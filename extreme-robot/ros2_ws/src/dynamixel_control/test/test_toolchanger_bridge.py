"""분리 전에 기존 도구의 정지 실패를 확인하는 ROS 어댑터 검증."""

import json
from types import SimpleNamespace

from std_msgs.msg import String

from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge
from dynamixel_control.toolchanger_lock import ToolchangerLock
from test_toolchanger_lock import IO


def test_failed_tool_stop_prevents_lock_motor_goal():
    io = IO()
    lock = ToolchangerLock(io)
    lock.capture_zero('LOCKED')
    lock.enable()
    before = list(io.writes)
    replies = []
    bridge = SimpleNamespace(
        toolchanger=lock, read_only=False, emergency_stop_active=False,
        control_scope='END_EFFECTOR_ONLY', control_mode='MANUAL', mock_mode=True,
        tool_ids=[5], _tool_samples={5: {'torque_state': 'ON'}},
        _stop_tool=lambda reason: None,
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
        toolchanger_status_pub=SimpleNamespace(publish=lambda message:
                                              replies.append(json.loads(message.data))))
    MoveItDynamixelBridge._toolchanger_command(
        bridge, String(data='{"command":"UNLOCK","direction":1}'))
    assert io.writes == before
    assert replies[-1]['state'] == 'LOCKED'
    assert 'OFF' in replies[-1]['command_error']
