# XL430 ID1 결합 잠금 제어

## 확인된 사양

사용자 확인: XL430, ID1, 외부 감속비 1:1. 영점 기준으로 어느 방향이든
22.5° 이동할 때마다 결합/분리가 바뀐다. 추가 endpoint 캘리브레이션 대신
현재 영점 위치와 그때의 결합 상태(LOCKED/UNLOCKED)를 기록한다.

XL430-W250의 모델 번호는1060, 한 회전은4096 count다.
따라서 한 인덱스는256 tick이다. 근거:
[ROBOTIS XL430 매뉴얼](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/).

## 소유권 및 안전 조건

- 기존 MoveItDynamixelBridge가 ID1을 같은 버스 잠금으로 소유한다. 별도 serial 노드를 추가하지 않는다.
- 기본 `toolchanger_enabled=true`. 기존 모듈에서만 명시적으로 false를 사용한다.
- 팔 5축 목록과 그리퍼 allowlist에는 ID1을 넣지 않는다. 자동 도구 탐색에서도 제외한다.
- ID1은 position mode3, velocity-based profile, 자동 torque-on 비활성 상태만 허용한다.
  operating/drive mode, Homing Offset, EEPROM은 자동 변경하지 않는다.
- SET_ZERO는 토크OFF·속도0의 새 피드백에서만 허용하며, 현재 물리 상태를 사용자가 지정한다.
- ENABLE은 profile acceleration1/velocity5 설정 → 현재위치goal → 토크ON
  순서로 점프를 방지하고 위치·토크·profile readback을 검증한다.
- LOCK/UNLOCK 전 선택된 도구를 정지한다. 이동 중, 분리 상태, 영점 미설정,
  피드백 오류/노후화에서는 도구 enable/motion gate를 닫는다.
- 위치 허용오차4tick, 피드백 유효시간0.5s, 이동 제한시간6s. 주기0.2s.
- profile velocity5는 명목6.87°/s이므로22.5° 이동에 가감속 포함 약3.6s가 필요하다.
  기본6s 제한은 이 nominal 이동시간보다 길게 잡았다.
- 단일 회전 범위0~4095 밖으로 넘어가는 방향은 거부한다. 반대 방향을 명시한다.
  0/4095 사이에서 큰 회전으로 래핑하지 않는다.
- 결합 판정은 영점과 인덱스 위치에 기반한 추정이다. 결합 센서 확인을 뜻하지 않으며
  `mechanically_confirmed=false`를 항상 명시한다.
- 비상정지/도구분리/노드종료에는 ID1도 토크OFF한다. read_only는 레지스터 쓰기를 하지 않는다.

## ROS 인터페이스

`/toolchanger/command`: std_msgs/String JSON. 명령 SET_ZERO, ENABLE, LOCK, UNLOCK, STOP.
SET_ZERO에는 state=LOCKED 또는UNLOCKED, LOCK/UNLOCK에는 direction=1 또는-1을 사용한다.
STOP 이외 명령은 writable MANUAL END_EFFECTOR_ONLY 벤치 경로에서만 허용한다.
FULL_ROBOT에서는 상태 관측/도구 게이트가 동작하며 잠금 변경 명령은 거부한다.

`/toolchanger/status` 및 `/tool/status.toolchanger`에 ID1 위치·토크·영점·상태·오류를 제공한다.
`/joint_states`에는 `toolchanger_lock_wheel`의 영점 대비 실제각을 추가한다.
볼은 캠 연동식이 제공되지 않았으므로 encoder로부터 볼 위치를 추정하지 않는다.

`toolchanger_zero_file`을 지정하면 SET_ZERO 때 atomic JSON 저장, 재시작 때 검증 후 로드한다.
파일이 없으면 미보정으로 시작한다. 기본 빈 경로에서는 RAM만 갱신한다.
파일에는 ID/model/gear/step/zero tick/state를 저장한다. 파일 로드가 토크ON을 유발하지 않는다.

## 인수 범위

이번 구현은 제어코어와 ROS/하드웨어 어댑터 및 가상 검증이다.
GUI 잠금 버튼, 자동 도구교체 시퀀스와 물리 결합·탈착·하중 시험은 인수하지 않는다.
