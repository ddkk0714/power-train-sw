"""The ops broker forwards only the existing lock command vocabulary."""
import json
from types import SimpleNamespace

import pytest

from powertrain_ros.ops_broker_node import OpsBrokerNode
from powertrain_ros.ops_contract import ACTIONS, ROLE_CONSOLE


def dispatch(kind, params):
    published, outcomes = [], []
    broker = SimpleNamespace(
        _coupling_lock_pub=SimpleNamespace(publish=lambda m: published.append(json.loads(m.data))),
        _complete_order=lambda *args: outcomes.append(args[-2:]))
    order = SimpleNamespace(kind=kind, params=params)
    OpsBrokerNode._execute(broker, order, None, ROLE_CONSOLE)
    return published, outcomes


@pytest.mark.parametrize('params', [
    dict(command='ENABLE'), dict(command='SET_ZERO', state='LOCKED'),
    dict(command='CHANGE', to='UNLOCKED', direction=-1),
])
def test_dispatch_preserves_the_validated_lock_request(params):
    published, outcomes = dispatch('publish_lock_command', params)
    assert published == [params]
    assert outcomes[0][0] is True
    assert 'verify lock status' in outcomes[0][1]


@pytest.mark.parametrize('params', [
    dict(command='CHANGE', to='LOCKED', direction=True),
    dict(command='ENABLE', actuator_id=1), dict(command=[]),
    dict(command='SET_ZERO', state='UNKNOWN'), dict(command='LOCK'),
])
def test_invalid_ops_request_publishes_nothing(params):
    published, outcomes = dispatch('publish_lock_command', params)
    assert not published and outcomes[0][0] is False


def test_stop_does_not_accept_a_hidden_motor_target():
    assert dispatch('publish_lock_stop', {})[0] == [{'command': 'STOP'}]
    published, outcomes = dispatch('publish_lock_stop', {'actuator_id': 2})
    assert not published and outcomes[0][0] is False


def test_both_lock_actions_are_console_only_and_target_the_existing_ingress():
    for action in ('robot_arm_lock_command', 'robot_arm_lock_stop'):
        spec = ACTIONS[action]
        assert spec.roles == frozenset({ROLE_CONSOLE})
        assert spec.target == ('/toolchanger/command',)
