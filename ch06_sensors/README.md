# Chapter 6 — 센서 시뮬레이션

책에서 가장 큰 장이다 (**코드 33블록** = py 22 + txt 11).

**6.1 신규 `experimental` 센서 API를 책이 정확하게 쓰고 있다** — 클래스메서드 패턴,
인자 이름, annotator 문자열까지 맞다. 초반에 "API 사용법이 틀렸다"고 판단했다가
원고 코드를 그대로 옮겨 적어 보고 **철회했다**. 틀린 건 내 검증 코드였다.

진짜 문제는 세 군데에 몰려 있다.

| 구간 | 문제 |
|---|---|
| 6.2 카메라 | 두 API 혼용 (ERR-024), warp 배열 미안내 (ERR-025) |
| 6.9 하드웨어 | **에셋 경로·부착 방식이 통째로 안 됨** (ERR-022/023/034/035/**036**) |
| 6.10 케이블 | **케이블이 성립하지 않음** (ERR-033) |

그중 **ERR-036 이 이 장에서 가장 중요하다.** 6.9.6 실습의 전제 자체가 무너진다.

## 실행 방법

```bash
conda activate isaac_env
export OMNI_KIT_ACCEPT_EULA=YES
python sensors_verified.py          # 센서 6종 한 번에 확인
```

> **GUI 로 볼 때 주의**
> RTX 실시간 렌더링을 길게 돌리면 전력 부하가 크다. 2026-09-21 에 4분 연속
> GUI 렌더 중 **시스템 전원이 끊긴 적이 있다** (모니터 동반 차단 → 본체 밖 전원 경로).
> 그림만 필요하면 `OUT_DIR=` 를 주어 **헤드리스 렌더**를 쓰는 편이 안전하다.
> 감시가 필요하면 `book_verify/powerwatch.sh` 를 켜 두면 된다.

## 파일 목록

### 실습 코드 (책 블록 대응)

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`sensors_verified.py`](sensors_verified.py) | 6.2~6.9 | ✅ | 센서 6종 동작 확인 |
| [`sensor_capture.py`](sensor_capture.py) | 6.8 [실습] b21 | ✅ | 수정 3건 (ERR-028/031/032) |
| [`gripper_control.py`](gripper_control.py) | 6.9 b05·b06 | ✅ | 수정 3건 (ERR-009/022/**035**) |
| [`hardware_rig.py`](hardware_rig.py) | 6.9.6 [실습] b07 | ✅ | 수정 4건 (ERR-022/023/027/**036**) |
| [`ft_and_cable.py`](ft_and_cable.py) | 6.10 [실습] b05 | ✅ | 수정 2건 (ERR-009/**033**) |
| [`demo_ch06_gui.py`](demo_ch06_gui.py) | — | ✅ | 리그+케이블 통합 데모 (GUI/헤드리스 겸용) |

### 검증 도구 (책에 없음 — `book_verify/`)

| 파일 | 무엇을 가렸나 |
|---|---|
| `ch6_remaining.py` | 남은 블록 7개 일괄 검증 (7/11 통과) |
| `cable_anchor_test.py` | 케이블 앵커·신장 3조건 비교 → ERR-033 |
| `lidar_gripper_probe.py` | `Lidar.create()` 시그니처, 그리퍼 mimic → ERR-034/035 |
| `usd_parent_vs_joint.py` | USD 부모-자식 ≠ 물리 부착 → ERR-036 |
| `gripper_attach_correct.py` | 올바른 부착 방법 검증 |
| `robotiq_variants2.py` | 그리퍼 에셋 3종 비교 (OBS-001, 미결) |

## 이 장에서 실측한 것

### ① 센서 6종 (`sensors_verified.py`)

```
[6.2 카메라]     RGB (480,640,3) uint8 평균 69.8 | Depth 유한값 100%
[6.3 LiDAR]      generic-model-output 파싱 성공
[6.4 IMU]        keys=['time','physics_step','linear_acceleration',
                       'angular_velocity','orientation']
[6.5 접촉]        keys=['time','physics_step','in_contact','force',
                       'number_of_contacts']
[6.9 RealSense]  D455 prim 40개 로드
```

### ② LiDAR 데이터 경로가 **둘**이고 좌표계가 다르다 (ERR-026 확장)

| 경로 | 좌표계 | 비고 |
|---|---|---|
| `generic-model-output` | **구면** (방위각·고도각·거리) | 직교로 쓰면 틀린다 |
| Replicator `IsaacCreateRTXLidarScanBuffer` | **직교** (x, y, z) | 변환 불필요 |

ScanBuffer 실측: x −17.08~18.72, y −16.08~16.07, z −1.11~−0.97, r 3.95~19.09

### ③ 센서 융합 실습 (`sensor_capture.py`)

원고를 그대로 돌리면 **세 번 연속으로 막힌다.**

```
ERR-028  WheeledRobot(prim_path=...)         → 신·구 API 혼용
ERR-031  RtxSensorCpuIsaacCreateRTXLidar...  → 접두사가 사라짐
ERR-032  빈 배열이 None 검사를 통과          → cannot reshape array of size 0
```

수정 후:

```
Frame 0000: RGB 저장, LiDAR 198,762 pts
Frame 0001: RGB 저장, LiDAR 198,775 pts
```

### ④ 그리퍼는 6 DOF, 구동은 1개로 충분 (ERR-035)

원고는 "액티브 DOF 1개" 라 하지만 **6개가 노출된다.**
다만 `finger_joint` 하나만 구동하면 **나머지 5개가 전부 따라온다** (mimic).

```
개방 → 폐쇄   finger_joint 0.000 → 0.800 rad
              나머지 5/5 전부 ±0.800 rad 연동
```

### ⑤ ★ USD 계층은 물리 부착이 아니다 (ERR-036)

이 장 최대 발견. 원고는 그리퍼를 `/World/UR5e/tool0/Robotiq2F85` 로 참조한다.

| 방법 | 시작 z | 끝 z | PhysX 경고 | 판정 |
|---|---:|---:|---:|:--|
| ① `tool0` 아래 참조만 **(원고)** | −1498.008 | **nan** | **441** | ❌ 물리 발산 |
| ② 최상위 참조만 | 0.592 | 0.001 | 0 | ❌ 자유낙하 |
| ③ `tool0` 아래 + FixedJoint | 132,836,432 | **nan** | 다수 | ❌ 더 심하게 발산 |
| ④ **형제 참조 + ArticulationRoot 제거 + FixedJoint** | 0.0708 | 0.0708 | **0** | ✅ |

핵심은 **관절체가 하나여야 한다**는 것. UR5e 와 2F-85 는 각자 ArticulationRoot 를
가진 별개의 관절체이고, **PhysX 는 두 관절체를 조인트로 잇지 못한다.**
그리퍼의 루트를 떼어 내면 **팔 6축 + 그리퍼 6축 = 12 DOF 관절체 하나**가 된다.

### ⑥ 케이블이 성립하지 않는다 (ERR-033)

| 조건 | 맨 위 z | 체인 길이 | 판정 |
|---|---:|---:|:--|
| 원고 그대로 | **0.000** | 0.112 | ❌ 월드 원점으로 끌려가 바닥에 쌓임 |
| 앵커만 수정 | 0.900 | **0.888** | ❌ 0.42 m 가 두 배로 신장 |
| 앵커 + 병진축 잠금 | 0.900 | **0.425** | ✅ (이상값 0.420) |

**병진만 잠그고 회전은 열어 둔다.** 둘 다 잠그면 케이블이 아니라 쇠막대인데,
정지 상태에서는 둘 다 곧은 수직선이라 **눈으로 구별할 수 없다.**
그래서 스윙 시험을 넣었다 — 옆으로 치니 **145 mm** 흔들렸다.

### ⑦ F/T 근사 — 관절 반력은 밑동으로 갈수록 커진다

```
[1] shoulder_lift_joint   |F| 166.64 N   |T| 52.36 Nm   ← 가장 큰 부하
[2] elbow_joint           |F| 130.34 N   |T| 52.36 Nm
[3] wrist_1_joint         |F|  48.03 N   |T| 14.46 Nm
[5] wrist_3_joint         |F|  13.79 N   |T|  0.14 Nm   ← 가장 작은 부하
```

`get_measured_joint_forces()` 는 `(로봇 수, 관절 수, 6)` 을 준다. 6 = 힘 3축 + 토크 3축.

### ⑧ 원고 그대로 통과한 블록들

전부 수정 없이 동작했다.

| 블록 | 검증 방법 | 결과 |
|---|---|---|
| b11 PCD 저장 | 5,000점 저장 후 규격 대조 | ✅ 헤더 11행 + 데이터 5,000행, ASCII PCD v0.7 일치 |
| b14 IMU 노이즈 | 20,000 샘플 통계 | ✅ 표준편차 0.0202/0.0198/0.0201 (설정 0.02) |
| b19 레이캐스트 | 높이 아는 상자에 조준 | ✅ 1.300 m (예상 1.3), 2.000 m (예상 2.0) |
| b20 카메라 노이즈 | 블러 적용 전후 분산 | ✅ 8.02 → 1.33 (cv2 4.14.0) |

## 미해결 — OBS-001

`ch06_hardware_rig.png` 에서 **그리퍼 본체가 검게** 렌더링된다 (손가락 링크는 정상).
오류로 확정하지 않았다. 재현은 되지만 원인을 못 짚었다.

조사 과정에서 **같은 실수를 두 번** 했다. 기록해 둔다.

1. **떨어진 줄 모르고 빈 화면의 밝기를 쟀다.** 그리퍼는 고정 베이스가 없으면
   자유낙하한다 (z 0.2418 → 0.0014, 60스텝). 하늘을 찍어 놓고 "평균 밝기 219,
   어두운 픽셀 0.0%" 를 보고 "재질 정상" 이라 결론 냈다.
2. **`XformCache` 로 물리 결과를 읽으려 했다.** 그건 작성된 USD 트랜스폼만 읽는다.
   물리가 움직인 위치는 `RigidPrim.get_world_poses()` 로 읽어야 한다.

> **교훈 — 숫자로만 판단하지 말고 이미지를 눈으로 먼저 볼 것.**
> 물체가 화면에 없으면 재질을 의심하기 전에 **떨어졌는지부터** 확인할 것.

## 책에 넣을 그림

| 파일 | 내용 |
|---|---|
| `ch06_hardware_rig.png` | UR5e + 2F-85 리그 (OBS-001 도 이 그림에 보인다) |
| `ch06_ft_cable.png` | 케이블 + F/T 씬 |
| `ch06_gripper_open/closed.png` | 그리퍼 개폐 |

## 관련 문서

- 오류 표시: 각 실습 파일 상단 주석의 `[ERR-0NN]` 항목

---

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch06/ch06_demo_rig_144406.png`](../_docs/images/book/ch06/ch06_demo_rig_144406.png) | 센서 데모 리그 구성 |
| [`../_docs/images/book/ch06/ch06_demo_rig2_144411.png`](../_docs/images/book/ch06/ch06_demo_rig2_144411.png) | 데모 리그 다른 시점 |

