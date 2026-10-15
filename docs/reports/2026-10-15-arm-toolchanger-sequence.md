# 자동 도구교체 시퀀스 구현 결과 (어댑터까지)

레퍼런스 [OSrobotToolChanger](https://github.com/isenklopr/OSrobotToolChanger)의
`changeStatus()` 제어 흐름을 참고해 ID1 잠금 위에 교체 시퀀스를 올렸다.
2026-10-15 사용자 결정에 따라 결합 확인은 **ID1 부하 프록시**를 쓰고,
레퍼런스의 `checkTime()` 주기 감시는 구현하지 않았다. GUI 잠금 버튼은 다음 작업이다.

## 변경

- `toolchanger_sequence.py` 신규: IDLE → TOOL_STOP → MOVING → CONFIRMED,
  이동 불필요 시 UNCHANGED, 모든 실패는 ABORTED.
- `/toolchanger/command`에 `CHANGE`(to/direction) 추가. 기존 `LOCK`/`UNLOCK` 유지.
- `/toolchanger/confirm` 신규: 성공 1회만 발행. 레퍼런스 confirm 릴레이에 대응한다.
- `_LockMotorIO.read()`에 XL430 Present Load(126, 2바이트 부호) 추가.
- 도구 정지·토크OFF 확인 전처리를 `_prepare_tool_for_lock()`으로 뽑아
  원시 LOCK/UNLOCK과 시퀀스가 같은 경로를 쓴다.
- `toolchanger_coupling_load_threshold`(기본0=프록시 OFF), `toolchanger_mock_load` 추가.
- 비상정지·도구분리·노드종료·STOP에서 진행 중 시퀀스를 닫는다.
- 상태 발행은 잠금 snapshot에 시퀀스 snapshot을 합친다.

## 검증 (x86 컨테이너 `extreme-robot-ros2`)

- 설치 빌드: `robot_arm_msgs` `robot_arm_description` `dynamixel_control` 3 packages finished.
- 잠금·시퀀스·브리지 테스트: **31 passed**.
- `dynamixel_control` 전체 스위트: **217 passed / 40 failed**.
  동일 환경의 HEAD 기준선은 201 passed / 44 failed이며 **실패 이름 집합 차집합은 공집합**이다.
  즉 신규 실패 0건이고, 40건은 전부 기존 실패다(flake8·pep257 포함).
  기준선에만 있던 4건은 `dynamixel_control`만 추출한 기준선 사본에서
  GUI/URDF 상대경로가 없어 난 수집 차이다.
- 신규 모듈 `toolchanger_sequence.py`는 flake8·pydocstyle 모두 0건.
  `dynamixel_control`+`test` flake8 총계는 기준선 58 → 현재 56건.
- 설치된 `ros2 run` 엔트리포인트 mock E2E 3블록 PASS, **2회 연속 재현**:
  기존 잠금 블록 → read-only 콜드스타트 → 신규 교체 시퀀스 블록.
- 시퀀스 블록이 확인한 것: UNLOCK CHANGE confirm(RELEASED),
  **음성 대조**(임계값120·주입부하0 → `NOT_ENGAGED` ABORTED, 토크OFF, confirm 미발행,
  `below threshold 120` 사유), 부하150 주입 후 재ENABLE → LOCK CHANGE
  confirm(ENGAGED, peak 150), 동일 상태 재요청 → UNCHANGED·confirm 미발행.
- `mechanically_confirmed`는 전 구간 false를 유지했다.
- 실제 serial/장치 접근과 실기 구동은 수행하지 않았다.

재현:

```bash
# 컨테이너 안
source /opt/ros/humble/setup.bash
source <ws>/install/setup.bash
cd <ws>
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
ROS_DOMAIN_ID=82 python3 src/dynamixel_control/scripts/verify_toolchanger_mock.py
```

## 남은 일

- **GUI 잠금 버튼**(다음 작업). `robot_manual_gui`는 이번에 건드리지 않았다.
- ⚠️ **주기 감시 미구현**: 잠금 이동 순간에만 결합을 추정한다. 그 뒤 정지 상태에서
  도구가 빠지는 것은 감지 경로가 없다. 레퍼런스가 `checkTime()`으로 막던 시나리오다.
- **결합 부하 임계값 실측 미완료.** 기본값0은 프록시를 끈 상태이며,
  실기에서 결합/무결합 스트로크 부하를 측정해 임계값을 정해야 기능이 켜진다.
- 부하 프록시는 결합 센서가 아니다. 물리 결합·탈착·하중 시험은 미완료다.
- 실제 ID1 baud/mode/drive mode, 영점·방향 확인도 기존대로 미완료다.
