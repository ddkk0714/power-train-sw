"""교체 전용 ROS 입력. 일반 팔 명령과 교체 세션은 같은 입력을 공유하지 않는다."""

import json
import math
from pathlib import Path

from robot_arm_msgs.msg import ChassisMode
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from dynamixel_control.dock_poses import DockPoseStore, load_docks
from dynamixel_control.qos_profiles import HEARTBEAT_QOS
from dynamixel_control.tool_change_interlock import ToolChangeDenied, ToolChangeInterlock


class ToolChangeBridge:
    """직렬 버스의 단일 소유자인 브리지 안에서 교체 권한을 관리한다."""

    def _init_tool_change(self):
        self._change_gate = ToolChangeInterlock()
        self._dock_table = None
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        self._dock_store = DockPoseStore(
            self.get_parameter('toolchanger_dock_file').value, JOINT_CONFIG)
        self._dock_status_pub = self.create_publisher(String, '/toolchanger/dock_status', 10)
        self.create_subscription(String, '/toolchanger/dock_command', self._dock_command,
                                 10, callback_group=self._command_group)
        if self._dock_store.filename and Path(self._dock_store.filename).is_file():
            try:
                self._dock_store.load()
                if self._dock_store.calibrated:
                    self._dock_table = self._dock_store.data
            except Exception as exc:
                self.get_logger().warn(f'dock table starts uncalibrated: {exc}')
        self._change_status = 'IDLE'
        self._change_error = ''
        self._change_status_pub = self.create_publisher(String, '/toolchanger/change_status', 10)
        self.create_subscription(ChassisMode, '/chassis_mode', self._change_chassis,
                                 HEARTBEAT_QOS, callback_group=self._command_group)
        self.create_subscription(String, '/toolchanger/session_command', self._change_session,
                                 10, callback_group=self._command_group)
        self.create_subscription(JointTrajectory, '/toolchanger/arm_trajectory',
                                 self._change_trajectory, 10,
                                 callback_group=self._command_group)
        self.create_timer(0.05, self._change_watchdog, callback_group=self._command_group)

    def _change_conditions(self):
        return dict(emergency=self.emergency_stop_active,
                    docks_valid=self._dock_table is not None,
                    now_ros=self.get_clock().now().nanoseconds * 1e-9)

    def _change_require(self):
        if self.control_scope != 'TOOL_CHANGE' or self.read_only or self.toolchanger is None:
            raise ToolChangeDenied('writable TOOL_CHANGE with coupling lock is required')
        self._change_gate.require_active(**self._change_conditions())

    def _change_chassis(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._change_gate.observe(msg.mode, stamp,
                                  self.get_clock().now().nanoseconds * 1e-9)
        self._change_watchdog()

    def _change_session(self, msg):
        try:
            request = json.loads(msg.data)
            command = request['command']
            if self.control_scope != 'TOOL_CHANGE':
                raise ToolChangeDenied('session commands require TOOL_CHANGE')
            if command in ('ABORT', 'FINISH'):
                self._change_exit('ABORTED' if command == 'ABORT' else 'DONE', '')
            elif command == 'ARM':
                if self.read_only or self.toolchanger is None:
                    raise ToolChangeDenied('writable bridge and coupling lock are required')
                from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
                self._dock_table = None
                self._dock_table = load_docks(
                    self.get_parameter('toolchanger_dock_file').value, JOINT_CONFIG)
                self._change_gate.arm(**self._change_conditions())
                try:
                    self._stop_tool('tool change arm')
                    self._change_require()
                    if self.mock_mode:
                        self.torque_enabled_ids.update(c['id'] for c in JOINT_CONFIG.values())
                    for name, config in JOINT_CONFIG.items():
                        if not self.mock_mode and not self._enable_torque(config['id'], name):
                            raise ToolChangeDenied(f'arm torque enable failed: {name}')
                except Exception:
                    self._change_exit('ABORTED', 'tool change arm failed')
                    raise
                self._change_status, self._change_error = 'ARMED', ''
            else:
                raise ToolChangeDenied('unknown session command')
        except Exception as exc:
            self._change_error = str(exc)
        self._change_publish()

    def _change_trajectory(self, msg):
        try:
            self._change_require()
            from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
            from dynamixel_control.joint_limits import JOINT_LIMITS
            if len(msg.points) != 1 or msg.joint_names != list(JOINT_CONFIG):
                raise ToolChangeDenied('one complete arm target is required')
            positions = msg.points[0].positions
            if len(positions) != len(JOINT_CONFIG):
                raise ToolChangeDenied('arm target length mismatch')
            for name, value in zip(msg.joint_names, positions):
                limits = JOINT_LIMITS[name]
                if not math.isfinite(value) or not limits['lower'] <= value <= limits['upper']:
                    raise ToolChangeDenied(f'arm target outside limit: {name}')
            self._dispatch_arm_trajectory(msg)
            self._change_status = 'MOVING'
        except Exception as exc:
            self._change_error = str(exc)
            if self._change_gate.active:
                self._change_exit('ABORTED', str(exc))
        self._change_publish()

    def _change_watchdog(self):
        if self._change_gate.active:
            try:
                self._change_require()
            except Exception as exc:
                self._change_exit('ABORTED', str(exc))
        self._change_publish()

    def _change_exit(self, state, reason):
        self._change_gate.exit(reason)
        self._change_status, self._change_error = state, reason
        # 현재 위치를 목표로 잡고 팔 토크를 유지한다.
        try:
            self._change_hold_arm()
        except Exception as exc:
            self._change_status = 'ABORTED'
            self._change_error += f'; arm hold failed: {exc}'
        self._stop_tool('tool change exit')
        self._stop_toolchanger()

    def _change_hold_arm(self):
        if self.mock_mode or self.read_only:
            return
        from dynamixel_control.moveit_dynamixel_bridge import (
            JOINT_CONFIG, ADDR_PRESENT_POSITION, ADDR_GOAL_POSITION)
        with self._bus_lock:
            for config in JOINT_CONFIG.values():
                motor = config['id']
                position = self._read_register(motor, ADDR_PRESENT_POSITION, 4,
                                               'change hold position', signed=True)
                self._write_register(motor, ADDR_GOAL_POSITION, 4, position,
                                     'change hold goal')

    def _change_publish(self):
        self._change_status_pub.publish(String(data=json.dumps(dict(
            state=self._change_status, armed=self._change_gate.active,
            error=self._change_error, docks_valid=self._dock_table is not None))))

    def _dock_command(self, msg):
        error = ''
        try:
            if (self.control_scope != 'TOOL_CHANGE' or self.read_only
                    or self._change_gate.active):
                raise ToolChangeDenied('teach requires writable, disarmed TOOL_CHANGE')
            request = json.loads(msg.data)
            if request['command'] == 'LOAD':
                self._dock_store.load()
            elif request['command'] == 'CAPTURE':
                self._dock_store.capture(
                    request['slot_id'], request['pose'], request['tool_type'],
                    self._dock_arm_samples(), request['tolerance_rad'],
                    request['timeout_s'], self.get_clock().now().nanoseconds * 1e-9)
            else:
                raise ValueError('dock command must be LOAD or CAPTURE')
            self._dock_table = self._dock_store.data if self._dock_store.calibrated else None
        except Exception as exc:
            error = str(exc)
            self._dock_table = None
        self._dock_status_pub.publish(String(data=json.dumps(dict(
            calibrated=self._dock_table is not None, error=error,
            slots=list(self._dock_store.data['slots'])))))

    def _dock_arm_samples(self):
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        if self.mock_mode:
            return {name: dict(position=self._mock_arm_positions[name], velocity=0,
                               torque=int(config['id'] in self.torque_enabled_ids))
                    for name, config in JOINT_CONFIG.items()}
        samples = {}
        with self._bus_lock:
            for name, config in JOINT_CONFIG.items():
                motor = config['id']
                samples[name] = dict(
                    position=self.tick_to_rad(name, self._read_register(
                        motor, 132, 4, 'teach position', signed=True)),
                    velocity=self._read_register(motor, 128, 4, 'teach velocity', signed=True),
                    torque=self._read_register(motor, 64, 1, 'teach torque'))
        return samples
