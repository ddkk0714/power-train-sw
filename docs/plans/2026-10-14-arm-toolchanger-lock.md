# ID1 잠금 제어 구현 및 검증

1. 영점·256tick 양방향 상태 전환 코어 → 미보정/경계/통신오류/timeout 테스트.
2. 기존 버스 단일 소유권과 도구 게이트 통합 → 설치된 mock 브리지 ROS E2E.
3. 영점 파일 저장/복원 및 읽기 전용 재시작 → 콜드스타트 E2E.
4. 실제 ID1의 mode/drive mode/영점·결합 상태 확인 → 이후 실기 단계.

구현 파일: `toolchanger_lock.py`, `moveit_dynamixel_bridge.py`.
재현 스크립트: `extreme-robot/ros2_ws/src/dynamixel_control/scripts/verify_toolchanger_mock.py`.
영점 가드를 제거한 임시 복사본에서 uncalibrated 검사가 실패했고 원본에서는 통과했다.
모터에 접속하지 않는 mock 환경의 installed ros2 run 엔트리포인트에서
SET_ZERO/ENABLE/UNLOCK(-1)/LOCK(+1)/tool torque 게이트/비상정지 및
read_only 재시작을 확인한다. 실기 연결/구동/NVM 작업은 수행하지 않는다.
