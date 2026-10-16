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


def validate_docks(data, joint_names, complete=True):
    """현재 팔 전체 관절과 실측 허용오차·제한시간이 있는 표만 받는다."""
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('dock table version must be 1')
    names = data.get('joint_names')
    if names != list(joint_names):
        raise ValueError('dock joint names/order do not match JOINT_CONFIG')
    slots = data.get('slots')
    if not isinstance(slots, dict) or (complete and len(slots) < 3):
        raise ValueError('at least three complete dock slots are required')
    for slot_id, slot in slots.items():
        if not isinstance(slot_id, str) or not slot_id or not isinstance(slot, dict):
            raise ValueError('invalid dock slot')
        if slot.get('tool_type') not in SUPPORTED_TOOL_TYPES:
            raise ValueError('unknown dock tool type')
        poses = slot.get('poses', {})
        if (not isinstance(poses, dict) or not set(poses).issubset(POSE_NAMES)
                or (complete and set(poses) != set(POSE_NAMES))):
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


class DockPoseStore:
    """미완성 teach 표도 원자적으로 보관하며, 완성 표만 운용에 허용한다."""

    def __init__(self, filename, joint_names):
        self.filename = filename
        self.joint_names = list(joint_names)
        self.data = dict(version=1, joint_names=self.joint_names, slots={})

    def load(self):
        self.data = dict(version=1, joint_names=self.joint_names, slots={})
        data = json.loads(Path(self.filename).read_text())
        validate_docks(data, self.joint_names, complete=False)
        self.data = data
        return self.data

    @property
    def calibrated(self):
        try:
            validate_docks(self.data, self.joint_names)
        except (ValueError, TypeError, KeyError):
            return False
        return True

    def capture(self, slot_id, pose_name, tool_type, samples,
                tolerance_rad, timeout_s, captured_at):
        if not self.filename:
            raise ValueError('explicit dock filename is required')
        if (not isinstance(slot_id, str) or not slot_id or pose_name not in POSE_NAMES
                or tool_type not in SUPPORTED_TOOL_TYPES):
            raise ValueError('valid slot, pose name and tool type are required')
        if set(samples) != set(self.joint_names):
            raise ValueError('fresh feedback from every arm joint is required')
        for sample in samples.values():
            if sample.get('torque') != 0 or sample.get('velocity') != 0:
                raise ValueError('capture requires arm torque OFF and stationary joints')
        from copy import deepcopy
        data = deepcopy(self.data)
        slot = data['slots'].setdefault(slot_id, dict(tool_type=tool_type, poses={}))
        if slot['tool_type'] != tool_type:
            raise ValueError('slot already belongs to another tool type')
        slot['poses'][pose_name] = dict(
            positions=[samples[name]['position'] for name in self.joint_names],
            tolerance_rad=tolerance_rad, timeout_s=timeout_s, captured_at=captured_at)
        validate_docks(data, self.joint_names, complete=False)
        self._save(data)
        self.data = data

    def _save(self, data):
        import os
        import tempfile
        path = Path(self.filename)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                             delete=False) as stream:
                temporary = stream.name
                json.dump(data, stream, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
