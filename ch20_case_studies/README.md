# Chapter 20 — 산업 적용 사례

실행 코드 블록이 없는 **종합 워크플로** 장이다. 앞 장들에서 검증한 요소를 하나의
실제 시나리오로 엮는 캡스톤이라, 각 단계를 **이 저장소의 실제 검증본**에 연결해 둔다.

> 검증 성격: 돌릴 코드가 없으므로 "실행 검증" 대상이 아니다. 대신 이 장이 참조하는
> 각 구성요소가 저장소에 실제로 **검증된 형태로 존재하는지**를 교차 확인했다 — 전부 존재.

## b01 — 4족 보행 시설 검사 로봇 워크플로

| 단계 | 책 내용 | 저장소 검증본 |
|---|---|---|
| 1 | 디지털 트윈으로 시설 3D 스캔/CAD 확보 | [ch11 pcd_to_mesh.py](../ch11_digital_twin_concept/pcd_to_mesh.py) · [ch12 factory_twin.py](../ch12_factory_twin/factory_twin.py) |
| 2 | 거친 지형·계단 보행 정책 학습 | [ch16 로봇별 RL](../ch16_rl_by_robot/) (보행 보상·환경 cfg) |
| 3 | 열화상/진동 센서 시뮬레이션 통합 | [ch06 센서](../ch06_sensors/) 노이즈 모델 응용 |
| 4 | Push Recovery 로 강건성 확보 | [ch16 push_recovery.py](../ch16_rl_by_robot/push_recovery.py) |
| 5 | Sim-to-Sim 검증 후 단계적 배포 | [ch19 safety_controller.py](../ch19_deployment/safety_controller.py) |
| 6 | 이상 감지 시 디지털 트윈 실시간 경고 | [ch13 anomaly_detector.py](../ch13_realtime_data/anomaly_detector.py) · [mqtt_monitor.py](../ch13_realtime_data/mqtt_monitor.py) |

## b02 — 휴머노이드 협업 로봇 워크플로

| PART | 책 내용 | 저장소 검증본 |
|---|---|---|
| 1~2 | USD 씬 + 휴머노이드 URDF 임포트 | [ch04 USD 씬](../ch04_usd_scene/) · [ch05 로봇 임포트](../ch05_robot_import/) |
| 3 | 사람 동작 인식 카메라 ROS2 연동 | [ch08 ROS2](../ch08_ros2/) |
| 4 | 실제 작업장 재현 + 사람 시뮬레이션 | [ch11](../ch11_digital_twin_concept/) · [ch12](../ch12_factory_twin/) |
| 5 | 비대칭 액터-크리틱 균형+매니퓰레이션 RL | [ch16 asymmetric_obs.py](../ch16_rl_by_robot/asymmetric_obs.py) · [lift_rewards.py](../ch16_rl_by_robot/lift_rewards.py) |
| 6 | 액추에이터 정밀 모델링 + 단계적 배포 | [ch18 actuator_model.py](../ch18_sim2real/actuator_model.py) · [ch19 배포](../ch19_deployment/) |

## 교차참조 검증

위 표의 **모든 참조가 저장소에 실제 검증본으로 존재**함을 확인했다(링크 전부 유효).
이 장은 앞 장의 결과를 조립하는 설계 가이드이므로, 각 구성요소의 동작 근거는
연결된 장의 README·실습 코드·캡처에서 확인할 수 있다.
