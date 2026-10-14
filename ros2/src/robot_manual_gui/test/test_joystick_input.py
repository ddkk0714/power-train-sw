"""Joystick edges use the existing GUI dispatcher without a hardware node."""

from test_tool_change_recovery import _ready_status, _window


def buttons(*pressed):
    values = [0] * 12
    for index in pressed:
        values[index] = 1
    return ((0.0,) * 8, tuple(values))


def ready(tool, generation=1, torque='ON'):
    status = _ready_status()
    status.update(tool_type=tool, tool_context_generation=generation,
                  tool_preparation_generation=generation,
                  tool_torque_state=torque, motion_allowed=torque == 'ON')
    if tool == 'cleaner':
        status.update(fsm_class='CleanerFSM',
                      tool_profile={'actuator_ids': [2], 'calibrated': True},
                      actuators=[{'id': 2, 'online': True, 'position': 0,
                                  'hardware_error': 0, 'torque_state': torque}])
    else:
        for sample in status['actuators']:
            sample['torque_state'] = torque
    return status


def test_dual_edges_torque_gate_and_generation():
    app, window, _, _ = _window()
    commands = []
    window.node.command_tool_fsm = lambda command, **values: (
        commands.append(command) or True)
    window._update_tool_status(ready('dual_motor_gripper', torque='OFF'))
    window._update_joystick(buttons())
    window._update_joystick(buttons(2))
    assert window.joy_active is None and commands == []

    # Enabling torque while X is already held cannot start a delayed move.
    window._update_tool_status(ready('dual_motor_gripper'))
    window._update_joystick(buttons(2))
    assert commands == []
    window._update_joystick(buttons())
    window._update_joystick(buttons(2))
    assert window.joy_active == 'X' and commands.count('JOG_OPEN') == 1
    for _ in range(3):
        window._update_joystick(buttons(2))
    assert commands.count('JOG_OPEN') == 1
    window._update_joystick(buttons())
    assert window.joy_active is None and commands.count('HOLD') == 1

    window._update_joystick(buttons(1))
    assert window.joy_active == 'B' and commands.count('JOG_CLOSE') == 1
    window._update_joystick(buttons(1, 2))
    assert window.joy_active is None and commands.count('HOLD') == 2
    window._update_joystick(buttons(1))
    assert window.joy_active is None
    window._update_joystick(buttons())

    # A immediately holds the current position and blocks held X.
    window._update_joystick(buttons(0))
    assert commands.count('HOLD') == 3
    window._update_joystick(buttons(0, 2))
    assert window.joy_active is None
    window._update_joystick(buttons())

    window._update_joystick(buttons(2))
    assert window.joy_active == 'X'
    before = len(commands)
    window._update_tool_status(ready('dual_motor_gripper', generation=2))
    window._update_joystick(buttons(2))
    assert window.joy_active is None and len(commands) == before
    window._update_joystick(buttons())
    window._update_joystick(buttons(2))
    assert window.joy_active == 'X'
    window._update_joystick(buttons())
    window.close()


def test_cleaner_mapping_and_joy_timeout():
    app, window, clock, _ = _window()
    commands = []
    window.node.command_tool_fsm = lambda command, **values: (
        commands.append(command) or True)
    window._update_tool_status(ready('cleaner'))
    window._update_joystick(buttons())
    window._update_joystick(buttons(2))
    assert commands[-1] == 'LEFT'
    window._update_joystick(buttons())
    assert commands[-1] == 'STOP'
    window._update_joystick(buttons(1))
    assert commands[-1] == 'RIGHT'
    clock.advance(0.6)
    window._refresh_joystick_connection()
    assert commands[-1] == 'STOP' and window.joy_active is None
    window.close()


def test_spur_open_close_use_fsm_and_release_hold():
    app, window, _, _ = _window()
    commands = []
    window.node.command_tool_fsm = lambda command, **values: (
        commands.append(command) or True)
    status = ready('spur_1motor_gripper')
    status.update(fsm_class='SingleMotorGripperFSM',
                  tool_profile={'actuator_ids': [5], 'calibrated': True},
                  actuators=[{'id': 5, 'online': True, 'position': 3200,
                              'hardware_error': 0, 'torque_state': 'ON'}])
    window._update_tool_status(status)
    window._update_joystick(buttons())
    window._update_joystick(buttons(2))
    assert commands[-1] == 'OPEN'
    window._update_joystick(buttons())
    assert commands[-1] == 'HOLD'
    window._update_joystick(buttons(1))
    assert commands[-1] == 'CLOSE'
    window._update_joystick(buttons())
    assert commands[-1] == 'HOLD'
    window.close()
