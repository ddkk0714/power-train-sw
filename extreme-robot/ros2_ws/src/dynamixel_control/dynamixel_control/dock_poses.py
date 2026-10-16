"""도크 표 진입 검증. 실측 포즈가 없으면 자동 교체를 허용하지 않는다."""

import json
import math
from pathlib import Path

from dynamixel_control.joint_limits import JOINT_LIMITS
from dynamixel_control.tool_profiles import SUPPORTED_TOOL_TYPES


POSE_NAMES = ('approach', 'seat', 'retreat')


def positive(value, label):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{label} must be finite and positive')
    return value


def validate_docks(data, joint_names):
    """현재 팔 전체 관절과 실측 허용오차·제한시간이 있는 표만 받는다."""
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('dock table version must be 1')
    names = data.get('joint_names')
    if names != list(joint_names):
        raise ValueError('dock joint names/order do not match JOINT_CONFIG')
    slots = data.get('slots')
    if not isinstance(slots, dict) or len(slots) < 3:
        raise ValueError('at least three complete dock slots are required')
    for slot_id, slot in slots.items():
        if not isinstance(slot_id, str) or not slot_id or not isinstance(slot, dict):
            raise ValueError('invalid dock slot')
        if slot.get('tool_type') not in SUPPORTED_TOOL_TYPES:
            raise ValueError('unknown dock tool type')
        poses = slot.get('poses', {})
        if set(poses) != set(POSE_NAMES):
            raise ValueError('approach, seat and retreat poses are required')
        for pose in poses.values():
            positions = pose.get('positions')
            if not isinstance(positions, list) or len(positions) != len(names):
                raise ValueError('dock pose must contain every arm joint')
            for name, angle in zip(names, positions):
                limits = JOINT_LIMITS[name]
                if (type(angle) not in (int, float) or not math.isfinite(angle)
                        or not limits['lower'] <= angle <= limits['upper']):
                    raise ValueError(f'dock pose outside joint limit: {name}')
            positive(pose.get('tolerance_rad'), 'pose tolerance')
            positive(pose.get('timeout_s'), 'pose timeout')
            positive(pose.get('captured_at'), 'capture timestamp')
    return data


def load_docks(filename, joint_names):
    return validate_docks(json.loads(Path(filename).read_text()), joint_names)
