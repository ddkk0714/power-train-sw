"""Face-button input to the existing authenticated arm command session."""


class ArmPadControl:
    def __init__(self, session):
        self.session = session
        self._identity = None
        self._needs_release = True
        self._target = None
        self._previous = set()
        self.detail = '패드 조작 꺼짐'

    def cancel(self):
        if self._target is not None:
            self.session.release_tool_jog(self._target)
        self._target = None
        self._needs_release = True
        self._previous.clear()

    def update(self, status, *, enabled, focused):
        context = self.session.context
        identity = context.tool_id, context.tool_generation
        if identity != self._identity:
            self.cancel()
            self._identity = identity
        if (not enabled or not focused or not status.get('pad_connected')
                or status.get('age_s', 9) > .6 or not status.get('arm_control')
                or not context.link_live or not context.manual_granted):
            self.cancel()
            self.detail = ('수동 제어권·패드 연결·창 포커스를 확인하세요' if enabled else '패드 조작 꺼짐')
            return
        if status.get('tool_mapping') != 'known':
            self.cancel()
            self.detail = '지원 패드: Xbox / DualSense · 패드 이름을 확인하세요'
            return
        buttons = set(status.get('tool_buttons', []))
        if self._needs_release:
            self._needs_release = bool(buttons)
            self._previous = buttons
            self.detail = '도구 교체·연결 후 버튼을 모두 놓으세요' if buttons else '패드 조작 준비'
            return
        target = 'both' if context.tool_kind == 'dual_gripper' else 'single'
        direction = 1 if 'open' in buttons else -1 if 'close' in buttons else 0
        if 'stop' in buttons or 'disable' in buttons or {'open', 'close'} <= buttons:
            direction = 0
        if not direction and self._target is not None:
            self.session.release_tool_jog(self._target)
            self._target = None
        if 'disable' in buttons - self._previous:
            self.session.set_tool_enabled(False)
        elif 'stop' in buttons - self._previous:
            self.session.soft_stop_tool(target)
        elif direction:
            result = self.session.press_tool_jog(target, direction)
            if result:
                self._target = target
            self.detail = '도구 누름 조작 중' if result else result.korean
        else:
            self.detail = 'X/□ 열기·좌회전 · B/○ 닫기·우회전 · A/× 정지 · Y/△ 토크 해제'
        self._previous = buttons
