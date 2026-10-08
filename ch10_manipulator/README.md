# Chapter 10 — 매니퓰레이터 제어

블록 19개 (원고 `ch10` b01~b09 + `ch10.6` b01~b10).

## 이 장에서 찾은 오류

| ID | 위치 | 내용 |
|---|---|---|
| **ERR-063** 🔴 | 10.2 b01 | `ArticulationMotionPolicy(rmpflow=)` 인자 이름 오류 + `panda_hand` ≠ 공식 `right_gripper` |
| **ERR-071** 🔴 | 10.3 b02 | **`PinkIKSolver` 가 6.1 에 없다** — 있는 것은 `Lula`/`Articulation`/`KinematicsSolver` |
| **ERR-084** 🔴 | 10.3 b02 | **URDF 상대경로 문자열** — 설치본에 `franka/franka.urdf` 가 없다, 설정 로더 사용 |
| **ERR-085** 🟠 | 10.3 b02 | **`solve(warm_start=)` API 없음** — 실제는 `compute_inverse_kinematics()` |
| **ERR-072** 🔴 | 10.5 b07~b09 | **Franka 예제 확장이 기본으로 꺼져 있다** — `enable_extension` 을 import 앞에 |
| **ERR-073** 🟠 | 10.4 b04~b06 | MoveIt2 연동 선행 패키지가 하나도 없다 + `/opt/ros/humble` 표기 |
| ERR-074 🟡 | 10.5 b09 | 쓰지 않는 `world.get_observations()` |
| **ERR-081** 🔴 | 10.6 b06·b10 | **도달 판정을 다른 프레임으로 잰다** — 40.7 mm 상수 차이라 `< 0.005` 가 영원히 거짓, 성공률 0% 고정 |
| **ERR-082** 🔴 | 10.6 b06·b10 | **공구 길이(TCP 오프셋)가 없다** — 손목을 구멍 속으로 밀어 넣으라는 명령이 된다 |
| **ERR-083** 🟠 | 10.6 b09 | **쿼터니언 순서 미변환** — Isaac (w,x,y,z) 를 RoboDK (qx,qy,qz,qw) 헤더로 내보낸다 |
| ERR-077 🟠 | 10.6 b07 | `validate_trajectory` 가 `target_position` 을 쓰지 않는다 — 검사를 안 한다 |
| ERR-078 🟡 | 10.6 b10 | 작업물 블록이 드릴 구멍 3개 중 2개를 벗어난다 |
| ERR-080 🟡 | 10.6 b04·b05 | `re` 미import, 죽은 `prev_z`, 안 쓰는 `Gf` |
| **ERR-075** 🔴 | 10.5 b09 | `end_effector_initial_height` 는 **월드 절대 높이** — 안 주면 팔이 테이블을 훑는다 |
| **ERR-076** 🔴 | 10.5 b09 | `world.reset()` 직후 **집게가 닫혀 있다** — 여는 줄이 빠졌다 |

## 블록별 상태

| 블록 | 내용 | 상태 |
|---|---|---|
| b01 | RMPflow 초기화 | ❌ ERR-063 |
| **b02** | **IK 솔버 (Lula 로 재작성)** | ✅ **성공 — 1.3~5.8 mm** (ERR-071·084·085) |
| b03 | MoveIt2 ↔ Isaac Sim 구조도 | ✅ 그림, 코드 아님 |
| b04 | `ros2_control` XML | ❌ ERR-073 — 플러그인 미설치 |
| b05 | 실행 절차 (셸) | ❌ ERR-073 — humble 없음, `franka_moveit_config` 없음 |
| b06 | `moveit_py` Cartesian | ❌ ERR-073 — `moveit_py` 없음 |
| b07 | Pick&Place 상태머신 | ⚠️ 순수 파이썬, b09 로 대체 검증 |
| b08 | 상태머신 실행 | ⚠️ 위와 같음 |
| **b09** | **Franka Pick&Place 종단간** | ✅ **성공 — 목표까지 0.2 cm** |
| **ch10.6 b01~b10** | **G코드 → 로봇 궤적** | ✅ **성공 — 도달률 100%, 최악 5.0 mm** |

## 파일 목록

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`pick_place.py`](pick_place.py) | 10.5 b09 | ✅ | 수정 4건 (ERR-072·074·075·076) |
| [`gcode_to_robot.py`](gcode_to_robot.py) | 10.6 b03~b10 | ✅ | 수정 6건 (ERR-063·077·078·080·081·082·083) |
| [`ik_solver.py`](ik_solver.py) | 10.3 b02 | ✅ | 수정 3건 (ERR-071·084·085) |
| [`drilling.nc`](drilling.nc) | 10.6 b02 | ✅ | 원고에 파일이 없어 새로 만듦 |

## ERR-072 — 확장이 꺼져 있으면 "모듈이 없다"고만 한다

```python
from isaacsim.robot.manipulators.examples.franka import Franka
# ModuleNotFoundError: No module named 'isaacsim.robot.manipulators.examples'
```

확장 목록을 찍으면 이유가 보인다.

```
isaacsim.robot.manipulators-3.4.5           enabled=True
isaacsim.robot.manipulators.examples-2.0.4  enabled=False   ← 여기
```

import **앞에** 켜야 한다.

```python
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.robot.manipulators")
enable_extension("isaacsim.robot.manipulators.examples")
```

Ch7 의 Occupancy Map(ERR-056) 과 같은 함정이다. 파이썬은 "없다"고만 말하고
켜면 된다는 사실은 알려 주지 않는다.

> 이 확장의 실제 경로는 `isaacsim/extsDeprecated/` 아래다 — 6.1 에서
> **폐기 예정**이라는 뜻이다.

## ERR-075 — "초기 높이" 가 로봇 기준이 아니다

컨트롤러 소스 `pick_place_controller.py` 72행:

```python
self._h1 = 0.3 / get_stage_units()      # 기본값 = 월드 절대 0.3 m
```

원고는 Franka 를 높이 0.8 m 테이블 위에 올려 놓고 이 인자를 주지 않는다.
그러면 0단계에서 손끝을 z=0.3 으로 내리려 하는데, 테이블 윗면보다
**0.5 m 아래**다. 팔이 테이블을 훑으며 박스를 쳐낸다.

원고 그대로 돌린 실측:

```
step  450  박스 (+0.379, +0.183, +0.825)
step  600  박스 (-0.008, -1.804, +0.436)   ← 날아갔다
step  750  박스 (-0.106, -2.091, +0.025)   ← 바닥
목표까지 거리 190.9 cm   ❌
```

## ERR-076 — 집게가 닫힌 채로 내려간다

ERR-075 를 고쳐도 박스가 1 mm 도 안 움직인다. 단계별로 찍으면 보인다.

```
 step   단계   손끝 z   집게폭   박스z
    0    0     1.209   0.0000   0.825    ← 시작부터 닫혀 있다
  324    2     0.894   0.0076   0.825
  336    4     0.890  -0.0000   0.825    ← 닫는 단계인데 이미 닫혀 있다
  903    9     1.125   0.0800   0.825    ← 놓는 단계에서야 벌어진다
```

```python
world.reset()
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
```

## 실행 방법

```bash
conda activate isaac_env
python pick_place.py              # GUI
HEADLESS=1 python pick_place.py   # 화면 없이
HOLD=8000 python pick_place.py    # 결과를 오래 보고 싶을 때
```

## 실측 결과

```
Pick & Place 시작  집기 [0.4  0.1  0.825]  놓기 [0.4 -0.25 0.825]
  step  450  단계 6  박스 (+0.355, -0.189, +1.084)
  step  750  단계 6  박스 (+0.402, -0.250, +0.829)
  컨트롤러 완료 — step 916

최종 박스 위치 (+0.402, -0.250, +0.825)
목표까지 거리   0.2 cm
테이블에서 떨어졌나: 아니오 ✅
✅ 성공
```

![Pick & Place](../_docs/images/book/ch10/ch10_pick_place_mid.png)

영상: [`ch10_pick_place.mp4`](../_docs/images/book/ch10/ch10_pick_place.mp4)

## 10.6 — G코드 → 로봇 궤적

CNC 가공 파일을 읽어 로봇이 구멍 자리를 순서대로 찍는다.
**원고를 그대로 돌리면 로봇이 완벽해도 성공률 0% 가 찍힌다.**

| | 원고 그대로 | 고친 뒤 |
|---|---|---|
| 도달 성공률 | **0%** (판정 불능) | **100%** |
| 최악 오차 | 410 mm | **5.0 mm** |
| 관절 경고 | 8건(허위 포함) | **0건** |

### ERR-081 — 판정을 다른 프레임으로 잰다

```python
ee_pos = franka.end_effector.get_world_pose()[0]     # 원고
if np.linalg.norm(ee_pos - target_position) < 0.005: # 5 mm
```

RMPflow 가 모는 것은 `franka.end_effector` 가 아니다.

| 프레임 | 목표까지 |
|---|---|
| `rmpflow.get_end_effector_pose()` (`right_gripper`) | **1.1 mm** |
| `franka.end_effector` (`panda_rightfinger`) | **40.7 mm** |
| `panda_hand` | 99.1 mm |

40.7 mm 는 **자세와 무관한 상수**다. 기준이 5 mm 이므로 이 조건은
절대 참이 될 수 없다.

높이(0.10~0.55 m)·거리(원점 x 0.40~0.25)·자세구속(고정/없음) 12가지를
쓸어 봐도 **전부 ~40 mm** 로 일정했다. 작업영역 문제라면 조건에 따라
달라져야 한다. 상수라는 것이 프레임 불일치의 증거였다.

### ERR-082 — 공구 길이(TCP)가 없다

프레임을 고치면 67%. 접근·후퇴 6개는 4.8 mm 인데 **작업 지점 3개만
17.5 mm** 에서 멈춘다. G코드 Z 는 드릴 **끝** 기준, 로봇 목표는 **손목**
기준이다. 빼 주지 않으면 손목을 구멍 속으로 밀어 넣으라는 말이 된다.

```python
TOOL_LENGTH = 0.05
work_target = drill_point + np.array([0, 0, TOOL_LENGTH])
```

### ERR-083 — 쿼터니언 순서 (실물 로봇으로 나간다)

Isaac Sim 은 **스칼라 앞** `(w,x,y,z)`, RoboDK·ROS 는 **스칼라 뒤**
`(qx,qy,qz,qw)` 다. 원고는 헤더만 바꾸고 값은 그대로 쓴다.

실측 — `[0,1,0,0]` 을 Isaac 이 해석한 회전행렬:

```
[[ 1.  0.  0.]
 [ 0. -1.  0.]      = X축 180°
 [ 0.  0. -1.]]
```

같은 숫자를 스칼라 뒤로 읽으면 **Y축 180°** 다. 공구는 둘 다 아래를
보지만 **수직축 기준 180° 돌아간다.** 회전대칭 드릴이면 무해하고,
방향성 있는 공구면 틀린 자세가 실물로 나간다.

### 오류가 아니었던 것

작업물을 `add_obstacle()` 로 등록하면 표면 가공을 방해할 것으로
의심했으나 **아니었다.** TCP 오프셋을 넣으면 손목이 블록 위에 머무르므로
등록 여부와 무관하게 100% 다. 둘 다 실측했다.

### 실행 방법

```bash
python gcode_to_robot.py                    # GUI
HEADLESS=1 python gcode_to_robot.py         # 화면 없이
OBSTACLE=1 python gcode_to_robot.py         # 원고처럼 작업물을 장애물로
TOOL_LENGTH=0.08 python gcode_to_robot.py   # 공구 길이 바꿔 보기
```

### 실측 결과

```
④ 9개 세부 지점 순회
    구멍1 접근 ✅ 4.9 mm   구멍1 작업 ✅ 4.7 mm   구멍1 후퇴 ✅ 4.9 mm
    구멍2 접근 ✅ 4.9 mm   구멍2 작업 ✅ 4.6 mm   구멍2 후퇴 ✅ 5.0 mm
    구멍3 접근 ✅ 4.9 mm   구멍3 작업 ✅ 4.7 mm   구멍3 후퇴 ✅ 4.8 mm
   도달 성공률 100%   최악 오차 5.0 mm
⑤ 궤적 검증 — 표본 477개  ✅ 관절 한계·속도 이상 없음
```

![G코드 궤적](../_docs/images/book/ch10/ch10_gcode_robot_mid.png)

영상: [`ch10_gcode_robot.mp4`](../_docs/images/book/ch10/ch10_gcode_robot.mp4)

---

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch10/ch10_pick_place_mid.png`](../_docs/images/book/ch10/ch10_pick_place_mid.png) | Pick & Place 중간 프레임 |
| [`../_docs/images/book/ch10/ch10_pick_place.mp4`](../_docs/images/book/ch10/ch10_pick_place.mp4) | Pick & Place 전체 영상 |
| [`../_docs/images/book/ch10/ch10_gcode_robot_mid.png`](../_docs/images/book/ch10/ch10_gcode_robot_mid.png) | G코드 궤적 중간 프레임 |
| [`../_docs/images/book/ch10/ch10_gcode_robot.mp4`](../_docs/images/book/ch10/ch10_gcode_robot.mp4) | G코드 → 로봇 궤적 영상 |

## 남은 것

- b04~b06 은 `topic_based_ros2_control` · `moveit_py` 설치 후 (승인 필요)
