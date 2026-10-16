# Isaac Sim & Isaac Lab 실무 완전 정복 — 실습 코드

**《Isaac Sim & Isaac Lab 실무 완전 정복》** 도서의 공식 실습 코드 저장소입니다.

책에 인쇄된 코드를 그대로 옮기지 않았습니다. **모든 코드를 실제 환경에서 실행해 검증**했고, 동작하지 않는 코드는 고쳤습니다. 원문과 무엇이 다른지는 각 장의 `README.md`에 전부 적어 두었습니다.

---

## 검증 환경

| 항목 | 버전 |
|---|---|
| Isaac Sim | 6.1.0.0 (pip) |
| Isaac Lab | 3.0 |
| OS | Ubuntu 24.04 LTS |
| Python | 3.12.14 (conda / miniforge3) |
| PyTorch | 2.11.0+cu128 |
| GPU | NVIDIA GeForce RTX 5090 (32 GB) |
| NVIDIA Driver | 580.159.04 |
| ROS 2 | Jazzy |

---

## 빠른 시작

### 1. 환경 설정

```bash
conda create -n isaac_env python=3.12 -y
conda activate isaac_env

pip install torch==2.11.0 --index-url https://download.pytorch.org/whl/cu128

export OMNI_KIT_ACCEPT_EULA=YES
pip install "isaacsim[all,extscache]==6.1.0.0" --extra-index-url https://pypi.nvidia.com
```

### 2. 실행

```bash
conda activate isaac_env
export OMNI_KIT_ACCEPT_EULA=YES
python ch03_setup/hello_isaac.py
```

> **Tip**: 터미널을 새로 열면 `conda activate`가 풀립니다. 프롬프트 앞에 `(isaac_env)`가 있는지 항상 확인하세요.

---

## 장별 실습 코드

| 장 | 주제 | 실습 파일 |
|---|---|---|
| [Ch01](ch01_why_sim/) | 왜 시뮬레이션인가 | 개념 |
| [Ch02](ch02_tech_stack/) | 기술 스택 | 개념 |
| [Ch03](ch03_setup/) | 설치 · Hello Isaac Sim | `hello_isaac.py` |
| [Ch04](ch04_usd_scene/) | USD 씬 구성 | `warehouse_scene.py`, `lighting.py`, `variant_asset.py` |
| [Ch05](ch05_robot_import/) | 로봇 모델 임포트 | `franka_pick_place.py`, `go2_stand_pose.py`, `amr_differential_drive.py` 외 |
| [Ch06](ch06_sensors/) | 센서 시뮬레이션 | `sensors_verified.py`, `sensor_capture.py`, `hardware_rig.py` 외 |
| [Ch07](ch07_physics/) | 물리 심화 | `solver_iteration_demo.py`, `external_force.py`, `omnigraph_keyboard_control.py` |
| [Ch08](ch08_ros2/) | ROS 2 연동 | `ros2_full_bridge.py`, `ros2_camera_publish.py` |
| [Ch09](ch09_nav_slam/) | SLAM · Nav2 · cuVSLAM | `slam_scene.py`, `occupancy_map.py`, `cuvslam_scene.py` 외 |
| [Ch10](ch10_manipulator/) | 매니퓰레이터 · G코드 | `pick_place.py`, `ik_solver.py`, `gcode_to_robot.py` |
| [Ch11](ch11_digital_twin_concept/) | 디지털 트윈 개념 | `pcd_to_mesh.py` |
| [Ch12](ch12_factory_twin/) | 공장 디지털 트윈 | `factory_twin.py` |
| [Ch13](ch13_realtime_data/) | 실시간 데이터 연동 | `mqtt_monitor.py`, `opcua_bridge.py`, `anomaly_detector.py` |
| [Ch14](ch14_isaaclab_core/) | Isaac Lab 기초 | `cartpole_gui.py`, `cartpole_random_agent.py` |
| [Ch15](ch15_rl_basics/) | 강화학습 기초 | `quadruped_rewards.py`, `domain_randomization.py` |
| [Ch16](ch16_rl_by_robot/) | 로봇별 강화학습 | `lift_rewards.py`, `push_recovery.py`, `amr_nav_env.py` 외 |
| [Ch17](ch17_sdg/) | 합성 데이터 생성 | `sdg_pipeline.py`, `sdg_visualize.py` |
| [Ch18](ch18_sim2real/) | Sim-to-Real 전이 | `actuator_model.py`, `depth_noise_model.py` 외 |
| [Ch19](ch19_deployment/) | 정책 배포 | `policy_controller_node.py`, `safety_controller.py` |
| [Ch20](ch20_case_studies/) | 산업 적용 사례 | 개념 |
| [Ch21](ch21_multi_robot/) | 다중 로봇 협업 | `fleet_task_allocator.py`, `cooperative_transport_env.py` |
| [Ch22](ch22_vision_learning/) | 비전 기반 학습 | `vision_scene_cfg.py`, `distillation.py` 외 |
| [Ch23](ch23_extension/) | 커스텀 확장 개발 | `custom_extension.py`, `warehouse_picking.py` |
| [Ch24](ch24_trends/) | 미래 전망 | 개념 |

---

## 추가 콘텐츠

### 신규 섹션 (v11)

책에 추가된 인터랙티브 학습 섹션의 원고입니다:

| 섹션 | 내용 | 원고 |
|---|---|---|
| Ch3.9 | GUI로 놀아보기 — 코드 없이 로봇 배치·시뮬레이션 (그림 10장) | [`new_sections/ch03_9_gui_playground.md`](new_sections/ch03_9_gui_playground.md) |
| Ch5.9 | Physics Inspector로 관절 디버깅 (그림 7장) | [`new_sections/ch05_9_physics_inspector.md`](new_sections/ch05_9_physics_inspector.md) |
| Ch7.9 | OmniGraph 비주얼 프로그래밍 (그림 2장) | [`new_sections/ch07_9_omnigraph.md`](new_sections/ch07_9_omnigraph.md) |

### 실행 결과 캡처

[`images/`](images/) 폴더에 각 장의 실행 결과 스크린샷과 영상이 있습니다.

---

## 폴더 구조

```
chNN_주제/
├── README.md    ← 실습 설명, 실행법, 원문과의 차이
└── *.py         ← 검증된 실습 코드
images/
├── chNN/        ← 장별 실행 결과 캡처
└── gui/         ← GUI 레퍼런스 스크린샷
new_sections/    ← v11 신규 섹션 원고
qrcodes/         ← 장별 QR코드 이미지
```

---

## 알려진 환경 이슈

### ENV-001: Warp 1.16.0 호환

Isaac Sim 6.1이 번들하는 Warp 1.16.0은 초기화 순서 문제가 있습니다.
Isaac Lab 코드 실행 전에 `import warp`를 먼저 호출하면 해결됩니다:

```python
import warp  # Warp 1.16.0 호환 — 반드시 isaacsim 임포트 전에
from isaacsim.simulation_app import SimulationApp
```

### ENV-002: Warp 1.17.0 ProxyArray

`pip install warp-lang==1.17.0`으로 업그레이드하면 Cartpole/Franka 계열은 정상 동작하지만, AnymalC 등 일부 환경에서 `ProxyArray` 관련 오류가 발생합니다.

---

## 라이선스

- 실습 코드는 책의 예제를 기반으로 하며, 검증·수정 내역을 명시했습니다.
- Isaac Sim은 NVIDIA의 EULA를 따릅니다. 설치 시 `OMNI_KIT_ACCEPT_EULA=YES` 동의가 필요합니다.
