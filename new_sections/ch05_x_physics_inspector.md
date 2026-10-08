# 5.X Physics Inspector로 관절 디버깅

> **이 절의 목표**: Physics Inspector를 사용하여 로봇의 관절을 시각적으로 확인하고, 슬라이더로 직접 조작해 보는 것.
> URDF나 USD로 임포트한 로봇이 "정말 관절이 제대로 설정되었는지" 확인하는 가장 빠른 방법이다.

---

## 5.X.1 Physics Inspector란?

로봇을 임포트한 직후 가장 먼저 해야 할 일은 **관절이 제대로 돌아가는지 확인**하는 것이다. USD 파일을 열어 속성을 하나씩 읽는 것보다, Physics Inspector를 쓰면 슬라이더 하나로 관절을 직접 돌려 볼 수 있다.

Physics Inspector가 알려 주는 정보:

- 로봇이 가진 **관절(Joint) 목록과 개수**
- 각 관절의 **타입**(Revolute, Prismatic, Fixed 등)
- 관절의 **현재 위치, 제한(Limit), 강성(Stiffness), 감쇠(Damping)**
- 관절을 **슬라이더로 직접 움직여** 예상대로 회전/이동하는지 확인

⚠️ **주의**: Physics Inspector는 **시뮬레이션이 멈춘 상태**(Stop)에서만 관절을 자유롭게 조작할 수 있다. Play 중에는 물리 엔진이 관절 위치를 제어하기 때문에 슬라이더가 무시된다.

---

## 5.X.2 Physics Inspector 열기

1. 상단 메뉴에서 **Tools → Physics → Physics Inspector**를 선택한다.
2. 새 패널이 열린다. 처음에는 비어 있다.

### 로봇 선택 및 관절 탐색

1. Stage 패널에서 로봇의 **루트 prim**을 선택한다 (예: `/World/Franka`).
2. Physics Inspector 패널의 **Refresh** 버튼을 누른다.
3. 로봇이 가진 모든 관절이 목록으로 나타난다.

Franka Panda의 경우 다음과 같은 관절이 보인다:

| 관절 이름 | 타입 | 제한(도) | 역할 |
|---|---|---|---|
| `panda_joint1` | Revolute | -166° ~ +166° | 베이스 회전 |
| `panda_joint2` | Revolute | -101° ~ +101° | 어깨 |
| `panda_joint3` | Revolute | -166° ~ +166° | 팔꿈치 회전 |
| `panda_joint4` | Revolute | -176° ~ -4° | 팔꿈치 굽힘 |
| `panda_joint5` | Revolute | -166° ~ +166° | 손목 회전 |
| `panda_joint6` | Revolute | -1° ~ +215° | 손목 굽힘 |
| `panda_joint7` | Revolute | -166° ~ +166° | 손목 비틀기 |
| `panda_finger_joint1` | Prismatic | 0 ~ 0.04m | 왼쪽 집게 |
| `panda_finger_joint2` | Prismatic | 0 ~ 0.04m | 오른쪽 집게 |

---

## 5.X.3 슬라이더로 관절 조작하기

Physics Inspector에서 각 관절 옆의 **슬라이더**를 드래그하면 관절이 즉시 움직인다.

### 확인할 것

1. **회전 방향**: 슬라이더를 오른쪽으로 밀면 관절이 어느 방향으로 회전하는가? 예상과 다르면 URDF/USD의 축(axis) 설정을 확인한다.

2. **제한(Limit)**: 슬라이더가 끝까지 갔을 때 로봇의 자세가 자연스러운가? 제한이 너무 넓으면 자기 몸과 충돌할 수 있다.

3. **관절 개수**: Go2(4족)라면 12개(= 4다리 × 3관절), Franka라면 7+2(팔+집게)가 맞다. 숫자가 다르면 USD 파일이 잘못된 것이다.

💡 **팁**: 관절 하나를 선택한 상태에서 키보드 **↑/↓ 화살표**를 누르면 미세 조정이 가능하다.

### 실전 체크리스트

로봇을 임포트한 뒤 반드시 확인할 항목:

- [ ] 관절 개수가 스펙과 일치하는가?
- [ ] 모든 관절이 올바른 축으로 회전/이동하는가?
- [ ] 관절 제한이 물리적으로 합리적인가?
- [ ] Fixed Joint가 실제로 고정되어 있는가?
- [ ] Prismatic Joint(집게 등)가 올바른 방향으로 열리는가?

---

## 5.X.4 시뮬레이션 ON/OFF 차이

Physics Inspector의 동작은 **시뮬레이션 상태에 따라 달라진다**.

### Stop 상태 (시뮬레이션 OFF)

- 슬라이더로 관절을 **자유롭게** 조작할 수 있다.
- 물리 엔진이 개입하지 않으므로 중력도 없다.
- **관절 설정을 확인하는 데 적합**하다.

### Play 상태 (시뮬레이션 ON)

- 물리 엔진이 관절 위치를 결정한다.
- 슬라이더 값은 물리 결과를 **실시간으로 표시**한다.
- 슬라이더를 움직여도 물리가 즉시 되돌린다.
- **물리 응답을 관찰하는 데 적합**하다.

📌 **NOTE**: Play 상태에서 관절을 직접 제어하려면 Physics Inspector가 아니라 **관절 컨트롤러**(Ch7)나 **Python 스크립트**를 사용해야 한다.

---

## 5.X.5 관절 속성 튜닝

Physics Inspector에서 관절을 선택하면 Property 패널에 관절의 물리 속성이 나타난다. 핵심 속성 세 가지:

### Stiffness (강성)

관절이 목표 위치로 돌아가려는 **스프링 힘**의 세기.
- 높으면: 위치 제어가 정확하지만, 너무 높으면 진동한다.
- 낮으면: 부드럽지만, 외력에 밀린다.
- 0이면: 관절이 자유롭게 움직인다 (토크 제어 모드).

### Damping (감쇠)

관절 속도에 비례하는 **저항력**.
- 높으면: 관절이 천천히 움직이고, 진동이 억제된다.
- 낮으면: 빠르게 움직이지만, 오버슈트가 생긴다.

### Limit (제한)

관절이 움직일 수 있는 **범위**.
- Revolute: 각도(도) 단위.
- Prismatic: 거리(m) 단위.

💡 **팁**: Stiffness와 Damping의 초기값은 URDF에서 가져온 값이다. Isaac Sim에서 시뮬레이션이 불안정하면 이 값을 조정해야 한다. 일반적으로 `Damping ≈ Stiffness × 0.1` 정도가 안정적이다.

---

## 5.X.6 실습 — Franka 관절 점검

> 🛠️ **실습**: Franka Panda를 로드하고 Physics Inspector로 관절 7개를 하나씩 확인한다.

### 준비

```python
from isaacsim.simulation_app import SimulationApp
app = SimulationApp({"headless": False})

from isaacsim.core.api.world import World
from isaacsim.core.api.robots import Robot

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

nucleus = "omniverse://localhost/NVIDIA/Assets/Isaac/6.1"
franka_usd = nucleus + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
franka = world.scene.add(
    Robot(prim_path="/World/Franka", name="franka", usd_path=franka_usd)
)
world.reset()
```

### 확인 절차

1. **Tools → Physics → Physics Inspector**를 연다.
2. Stage에서 `/World/Franka`를 선택하고 **Refresh**를 누른다.
3. 다음을 확인한다:

| 확인 항목 | 기대 결과 |
|---|---|
| 관절 개수 | 9개 (7축 팔 + 2 집게) |
| joint1 회전 | 로봇 전체가 베이스 위에서 좌우 회전 |
| joint4 회전 | 팔꿈치가 안쪽으로 접힘 |
| finger_joint1,2 | 집게가 열리고 닫힘 (Prismatic) |

4. 각 관절 슬라이더를 **끝까지** 밀어 본다. 팔이 자기 몸과 부딪히는 지점이 있으면 제한을 기록한다.

### 코드로 동일하게 확인하기

Physics Inspector에서 본 것을 코드로도 확인할 수 있다:

```python
import numpy as np

world.reset()

print(f"관절 수: {franka.num_dof}")
print(f"관절 이름: {franka.dof_names}")

dof_props = franka.dof_properties
for i, name in enumerate(franka.dof_names):
    lower = np.degrees(dof_props["lower"][i])
    upper = np.degrees(dof_props["upper"][i])
    stiff = dof_props["stiffness"][i]
    damp = dof_props["damping"][i]
    print(f"  {name:25s}  제한 [{lower:7.1f}°, {upper:7.1f}°]  "
          f"강성 {stiff:.0f}  감쇠 {damp:.0f}")
```

**예상 출력** (Franka Panda):

```
관절 수: 9
관절 이름: ['panda_joint1', 'panda_joint2', ..., 'panda_finger_joint2']
  panda_joint1               제한 [ -166.0°,  166.0°]  강성 800  감쇠 40
  panda_joint2               제한 [ -101.0°,  101.0°]  강성 800  감쇠 40
  ...
  panda_finger_joint1        제한 [    0.0°,    2.3°]  강성 800  감쇠 40
```

GUI(Physics Inspector)와 코드 출력이 일치하면 관절 설정이 올바른 것이다.

---

## 5.X.7 Go2에 Physics Inspector 적용하기 — 함정 사례

4족보행 로봇 Go2로 같은 확인을 해 보자. 여기에는 Ch5.5에서 다룬 **함정**이 있다.

### 올바른 USD 파일 선택

```python
# ❌ 이 파일은 시각 형상만 있고 관절이 없다!
# go2_usd = nucleus + "/Isaac/Robots/Unitree/Go2/go2.usd"

# ✅ 물리·관절이 포함된 파일을 써야 한다
go2_usd = nucleus + "/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd"
```

Physics Inspector에서 **관절이 0개**로 나타나면 잘못된 USD 파일을 로드한 것이다.

### Go2 관절 확인

올바른 파일을 로드하면 **12개 관절**이 나타난다:

```
FL_hip_joint, FL_thigh_joint, FL_calf_joint   (앞왼)
FR_hip_joint, FR_thigh_joint, FR_calf_joint   (앞오른)
RL_hip_joint, RL_thigh_joint, RL_calf_joint   (뒤왼)
RR_hip_joint, RR_thigh_joint, RR_calf_joint   (뒤오른)
```

Physics Inspector 슬라이더로 hip → thigh → calf 순서로 각 관절을 움직여 보면, 다리 하나가 앞뒤로 접히고 옆으로 벌어지는 것을 확인할 수 있다.

---

## 이 절에서 배운 것

| 항목 | 핵심 |
|---|---|
| Physics Inspector | Tools → Physics → Physics Inspector |
| 관절 확인 | 루트 prim 선택 → Refresh → 슬라이더 조작 |
| 시뮬 OFF vs ON | OFF: 자유 조작 / ON: 물리 결과 관찰 |
| 핵심 속성 | Stiffness(강성), Damping(감쇠), Limit(제한) |
| Go2 함정 | `go2.usd`는 시각만, `go2_description_physics.usd`를 써야 함 |

다음 절에서는 이 관절들을 **Python 코드와 물리 컨트롤러**로 제어하는 방법을 다룬다.
