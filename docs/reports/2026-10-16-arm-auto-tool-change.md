# 자동 도구 교체 구현·검증 기록

## 범위와 진행 상태

정본: `docs/plans/2026-10-16-arm-auto-tool-change.md`.
실측 도크 포즈·시간·허용오차는 아직 없다. 검증 데이터는 테스트 전용이며 생산 기본값으로
설치하지 않는다. 실기 인수 전까지 AGENTS.md 임시 포인터를 유지한다.

## §5-1 TOOL_CHANGE 인터락

- `TOOL_CHANGE`는 운영자 ARM과 신선한 MISSION_STOP, 유효한 도크 표가 있어야 열린다.
  차체 신선도는 기존 arm_fsm과 같은 1초이며, 수신 나이와 발행 stamp 나이를 함께 검사한다.
  0/미래/동일/역행 stamp는 허가를 갱신하지 않는다.
- 사용자가 교체 전용 입력 방식을 선택했다. 기존 팔 토픽·액션·수동·진단 입력은 이 범위에서
  닫고 `/toolchanger/arm_trajectory`의 단일 목표만 받는다. 도크 표 로더는 진입 전제 검증에
  필요하므로 이 단계에 포함했고, 캡처·저장은 §5-2에서 추가한다.
- 도구 미발견은 FULL_ROBOT의 팔 궤적 허용 조건에서만 제외했다. 도구가 발견된 상태의
  다른 준비 실패는 여전히 팔을 막는다. 미발견 도구 토크·OPEN/CLOSE는 거부한다.
- 완료·중단·허가 끊김·비상정지는 무장을 소비한다. 팔 현재 위치를 홀드하고 도구와 잠금
  토크를 끈다. 자동 재무장하지 않는다. 교체 중 자동 도구 선택은 사용하지 않는다.

### 실행 검증

x86 `extreme-robot-ros2`, 격리 ROS_DOMAIN_ID=77. 소스 읽기 전용 마운트,
컨테이너 내부 `/tmp/build-auto`에서 설치 빌드했다.

```bash
source /opt/ros/humble/setup.bash
cd /tmp/build-auto
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
colcon build --base-paths /ws/extreme-robot/ros2_ws/src \
  --packages-select robot_arm_msgs robot_arm_description robot_arm_moveit_config dynamixel_control
source install/setup.bash
export ROS_DOMAIN_ID=77
python3 /ws/extreme-robot/ros2_ws/src/dynamixel_control/scripts/verify_toolchanger_mock.py
python3 /ws/extreme-robot/ros2_ws/src/dynamixel_control/scripts/verify_auto_tool_change_mock.py
```

- 설치 빌드 통과. 교체 범위·기존 잠금 관련 테스트 43 passed.
- 기존 설치 ROS 검증: 영점·토크·잠금·read-only 재시작·부하 프록시 음성 대조 통과.
- 신규 설치 ROS 검증: 무장 없음/정지 허가 없음 거부, 교체 팔 궤적과 잠금 명령 수락,
  일반 팔 토픽·액션 거부, 신선도 만료·비상정지 시 ABORTED 및 토크OFF 통과.
- 전체 기능 회귀는 원본 HEAD의 217 passed/38 failed와 변경본의 229 passed/38 failed를
  이름별 대조했다. 신규 실패 0. 기존 38건은 오래된 프리셋·테스트 대역 불일치 등이며
  이번 변경으로 고치지 않았다. 새 범위 검사에 필요한 FULL_ROBOT 필드만 기존 테스트
  대역에 명시했다. 전체 스위트가 모두 green이라는 의미는 아니다.
- 무장 검사를 설치본에서 고의 제거하면 신규 ROS 검증이 실패했다. 검사 복원 후 같은
  ROS 검증이 통과했다. 소스의 게이트는 변경하지 않았다.

GitHub fetch와 열린 PR 확인은 실행했다(열린 PR 0). `ssh jetson`은 이름 해석 실패로
원격 체크아웃을 확인하지 못했다. 이 작업에서는 Jetson이나 하드웨어에 쓰지 않았다.

## §5-2 도크 teach & replay 표

`/toolchanger/dock_command`의 CAPTURE가 슬롯 id·도구 종류·포즈 이름과 실측
`tolerance_rad`·`timeout_s`를 받는다. 관절 이름은 현재 JOINT_CONFIG 순서를 사용한다.
팔 전 축을 직접 읽어 토크OFF·속도0인 경우만 저장한다. 미완성 표도 원자적으로 보관하고,
3슬롯 각각 접근·안착·후퇴가 모두 유효할 때만 운용상 보정 완료로 인정한다.
LOAD는 이름·리밋·파일 무결성을 다시 검사하며 오류면 자동 교체를 막는다.

검증: 설치 빌드 통과, 관련 테스트 54 passed. 저장/로드·부분 teach 재시작·리밋/이름 위반·
파일 손상·원자적 교체 실패 보존을 확인했다. 설치된 ROS 브리지에서 토크ON 캡처 거부,
토크OFF 후 캡처·저장·로드, 손상 파일의 미보정 처리를 통과했다.
캡처의 토크/속도 검사를 고의 제거하면 토크ON·이동 중 테스트 2건이 실패했고,
복원하면 도크 테스트 11 passed 및 설치 ROS 검증이 다시 통과했다.
실측 포즈와 허용오차·시간 기본값은 추가하지 않았다.
