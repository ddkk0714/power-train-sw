from dataclasses import replace

from operator_console.arm_pad import ArmPadControl
from operator_console.arm_ops.session import ArmCommandSession
from operator_console.arm_ops.policy import ArmCommandContext
from operator_console.arm_ops.transport import RecordingArmTransport


def fixture():
    session = ArmCommandSession(clock=lambda: 100, transport=RecordingArmTransport(),
        context=ArmCommandContext(link_live=True, granted_mode='MANUAL', tool_id='dual_motor_gripper:3,4',
            tool_kind='dual_gripper', tool_generation=7, state_revision=1,
            capabilities=frozenset({'gripper_command', 'tool_enable', 'tool_jog'})))
    return ArmPadControl(session), session


def update(pad, *buttons, **changes):
    status = dict(pad_connected=True, arm_control=True, age_s=0, tool_mapping='known', tool_buttons=buttons)
    status.update(changes)
    pad.update(status, enabled=True, focused=True)


def test_neutral_press_release_uses_authenticated_session_and_relative_lease():
    pad, session = fixture()
    update(pad)
    update(pad, 'open')
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_tool_jog'
    assert session.submitted[-1].params['ttl_s'] == .6
    assert session.submitted[-1].params['target'] == 'both'
    update(pad)
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_tool_jog_stop'


def test_held_button_cannot_restart_after_swap_or_focus_loss():
    pad, session = fixture()
    update(pad, 'open')
    session.pump()
    assert not session.submitted
    update(pad)
    update(pad, 'open')
    session.pump()
    session.update_context(replace(session.context, tool_id='spur_1motor_gripper:5',
                                   tool_kind='single_gripper', tool_generation=8))
    update(pad, 'open')
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_tool_jog_stop'
    update(pad)
    update(pad, 'close')
    session.pump()
    assert session.submitted[-1].params['target'] == 'single'
    pad.update({}, enabled=True, focused=False)
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_tool_jog_stop'


def test_disable_edge_is_once_and_missing_mapping_never_moves():
    pad, session = fixture()
    update(pad, tool_mapping='')
    update(pad, 'open', tool_mapping='')
    assert not session.submitted
    update(pad)
    update(pad, 'disable')
    update(pad, 'disable')
    assert len(session.submitted) == 1
    assert session.submitted[0].params['enabled'] is False
