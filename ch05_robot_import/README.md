# Chapter 5 — 로봇 모델 불러오기 및 설정

## 실행 방법

```bash
conda activate isaac_env
export OMNI_KIT_ACCEPT_EULA=YES
python asset_path_test.py     # 에셋 경로가 살아있는지 먼저 확인
```

## 파일 목록

### 실습 코드 (책 블록 대응)

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`warehouse_with_robots.py`](warehouse_with_robots.py) | 5.8 [실습] | ✅ | 수정 3건 (ERR-022/027/028) |
| [`franka_pick_place.py`](franka_pick_place.py) | 5.3 Pick & Place | ✅ | **큐브 추가** + 확장 활성화 |
| [`amr_differential_drive.py`](amr_differential_drive.py) | 5.4 AMR 제어 | ✅ | 곡선 주행, 경로 검증 |
| [`go2_stand_pose.py`](go2_stand_pose.py) | 5.5 Go2 | ✅ | ERR-029/030 재현 |
| [`robot_gallery_gui.py`](robot_gallery_gui.py) | 5.4~5.6 | ✅ | **4종 동시 직립** (GUI) |
| [`render_robot_portraits.py`](render_robot_portraits.py) | — | ✅ | 책 삽입용 렌더 (5장 생성) |

### 검증 도구 (책에 없음)

| 파일 | 용도 |
|---|---|
| [`asset_path_test.py`](asset_path_test.py) | 에셋 경로 생존 확인 — prim 개수로 빈 참조 판별 |
| [`joint_limits.py`](joint_limits.py) | 관절 한계·기본값 조회 (부호 규약 파악) |
| [`h1_torque.py`](h1_torque.py) | 게인별 직립 성공 여부 실측 |
| [`h1_policy.py`](h1_policy.py) | 번들 보행 정책 사용법 |
| [`stand_sweep.py`](stand_sweep.py) / [`stand_default.py`](stand_default.py) | 서기 자세 각도 훑기 |
| [`h1_stance.py`](h1_stance.py) / [`h1_diag.py`](h1_diag.py) | 실패 가설 검증 기록 (보존용) |
| [`ch5_verify.py`](ch5_verify.py) | 5장 API 일괄 검증 |

---

## 블록별 실행 현황 (12개)

| 블록 | 내용 | 실행 | 결과 |
|---|---|---|---|
| b01 | 기본 제공 로봇 불러오기 | ✅ | 경로 4종 검증 |
| b02 | GUI로 URDF 임포트 | — | 조작 설명문, 실행 대상 아님 |
| b03 | URDF 임포트 자동화 | ✅ | 실제 URDF → USD 9파일 4.3MB |
| b04 | 조인트 드라이브 | ✅ | Franka 9 DOF, 오차 0.033 rad |
| b05 | Pick & Place | ✅ | 큐브 0.301m 운반, 오차 0.019m |
| b06 | 차동 구동 AMR | ✅ | 회전반경 1.57m (이론 1.67m) |
| b07 | Nova Carter | ✅ | 7 DOF, `joint_wheel_left/right` 확인 |
| b08 | Unitree Go2 | ✅ | 12 DOF (ERR-029 경로 수정 후) |
| b09 | Spot | ✅ | 12 DOF, 에셋 기본 자세로 직립 |
| b10 | Unitree H1 | ✅ | 19 DOF, kp 8000으로 직립 |
| b11 | PD 게인 공식 | ✅ | kp 394.8 / kd 12.57 — 책과 일치 |
| b12 | warehouse 실습 | ✅ | AMR 2.82m 주행 |

---

## 5.3 Pick & Place — 원문에 빠진 것

원문은 `picking_position`을 지정하지만 **그 자리에 물체를 놓지 않는다.**
로봇 팔은 정확히 움직이지만 **허공을 집었다 놓는다.** 에러는 없다.

```python
# 4장에서 배운 DynamicCuboid 를 그대로 쓰면 된다
cube = world.scene.add(DynamicCuboid(
    prim_path="/World/TargetCube", name="target_cube",
    position=pick_position, scale=np.array([0.05]*3),
    color=np.array([0.9, 0.2, 0.2]),
    mass=0.03,          # ★ 없으면 그리퍼 파지력보다 무거워 미끄러진다
))
```

확장 활성화도 필요하다 (ERR-015):
```python
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot.manipulators.examples", True)
```

실행 결과:
```
step 400  큐브 z=0.193  집어 올림
step 916  ✅ 완료 — 목표 오차 0.019 m
```

---

## 🔴 가장 먼저 알아야 할 것 — **책의 에셋 경로 대부분이 404다**

책이 쓰는 로봇·센서 USD 경로 15개를 NVIDIA 에셋 서버에 전부 조회한 결과,
**10개가 존재하지 않는다.** Isaac Sim 5.x → 6.x 에서 로봇 에셋이
**제조사(vendor)별 디렉터리로 재편**되었기 때문이다.

### 경로 대응표

| 대상 | 책 경로 (404) | Isaac Sim 6.1 실제 경로 |
|---|---|---|
| **Franka Panda** | `/Isaac/Robots/Franka/franka.usd` | `/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` |
| **Nova Carter** | `/Isaac/Robots/Carter/nova_carter.usd` | `/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd` |
| **Carter v1** | `/Isaac/Robots/Carter/carter_v1.usd` | `/Isaac/Robots/NVIDIA/Carter/carter_v1.usd` |
| **Robotiq 2F-85** | `/Isaac/Robots/Robotiq/2F85/2f85_instanceable.usd` | `/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd` |
| **Robotiq Hand-E** | `/Isaac/Robots/Robotiq/HandE/hand_e.usd` | `/Isaac/Robots/Robotiq/Hand-E/Robotiq_Hand_E_edit.usd` |
| **RealSense** | `/Isaac/Sensors/Intel/RealSense/rsd435.usd` | `/Isaac/Sensors/RealSense/D455/rsd455.usd` ⚠️ D435 없음 |
| **사람(여성)** | `.../female_adult_business_02/…` | `.../F_Business_02/…` |
| **사람(남성)** | `.../male_adult_construction_01/…` | `.../male_adult_construction_01_new/…` |

### 책이 맞은 경로 (수정 불필요)

```
✅ /Isaac/Robots/Unitree/Go2/go2.usd
✅ /Isaac/Robots/Unitree/H1/h1.usd
✅ /Isaac/Robots/UniversalRobots/ur5e/ur5e.usd
✅ /Isaac/Robots/BostonDynamics/spot/spot.usd
✅ /Isaac/Environments/Simple_Warehouse/warehouse.usd
```

### 실제 로드 검증

| 대상 | 책 경로 | 수정 경로 | 생성된 prim |
|---|---|---|---|
| Franka Panda | ❌ | ✅ | 35개 |
| Nova Carter | ❌ | ✅ | 1,362개 |
| Carter v1 | ❌ | ✅ | 73개 |

> ⚠️ **로봇이 안 보이는데 에러도 없다면 경로부터 의심하세요.**
> Isaac Sim은 없는 USD를 참조해도 **예외를 던지지 않고 빈 prim만 만듭니다.**
> `asset_path_test.py`가 prim 개수를 세어 이를 판별합니다.

### 권장 — 경로를 코드에 박지 말 것

이 오류의 근본 원인은 경로 하드코딩이다. 버전이 바뀌면 전부 깨진다.

```python
ROBOT_PATHS = {   # 버전이 바뀌면 이 표만 고치면 된다
    "franka": "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    "ur5e":   "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    "go2":    "/Isaac/Robots/Unitree/Go2/go2.usd",
}
```

---

## 🔴 5.2 URDF → USD 변환 API가 통째로 바뀌었다

```python
# ❌ 책 — 첫 import부터 실패
from isaacsim.asset.importer.urdf import _urdf
import_config = _urdf.ImportConfig()
omni.kit.commands.execute("URDFParseAndImportFile", ...)   # 이 커맨드도 6.1에 없음
# ImportError: cannot import name '_urdf' from 'isaacsim.asset.importer.urdf'

# ✅ 6.1
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

config = URDFImporterConfig(
    urdf_path="/path/to/my_robot.urdf",
    usd_path="/path/to/output_dir",   # ← 파일이 아니라 디렉터리다
    merge_fixed_joints=True,
    collision_type="Convex Hull",     # 구 convex_decomp
    allow_self_collision=False,       # 구 self_collision
    fix_base=True,
)
usd_output = URDFImporter(config).import_urdf()   # 반환값이 최상위 USD 경로
```

### 필드 이름 대응표

| 책 (구) | 6.1 (신) |
|---|---|
| `convex_decomp = False` | `collision_type = "Convex Hull"` (불리언 → 문자열) |
| `self_collision` | `allow_self_collision` |
| `import_inertia_tensor` | 없어짐 (항상 임포트) |
| `distance_scale` | 없어짐 |
| `merge_fixed_joints` / `fix_base` | 동일 ✅ |

### ⚠️ 출력이 단일 파일이 아니다

실제 URDF(조인트 16개)로 변환한 결과 — **9개 파일 / 4.3MB**:

```
<usd_path>/<robot_name>/
├── <robot_name>.usda          ← 최상위 (이걸 참조한다)
└── payloads/
    ├── base.usda  robot.usda  geometries.usd  instances.usda  materials.usda
    └── Physics/
        ├── physics.usda   ← 공통
        ├── physx.usda     ← PhysX 백엔드
        └── mujoco.usda    ← MuJoCo 백엔드
```

> 📌 이 구조가 바로 책 **4.8.3절 "USD Asset Structure 3.0 — 멀티 물리 백엔드 대응"** 에서
> 설명한 것이다. `physx.usda`와 `mujoco.usda`가 나란히 생성되는 것을 실제로 확인했다.
> **4.8.3의 서술은 정확하다.** 다만 5.2절이 그 구조를 만드는 주체라는 연결이 책에 없다.

### 변환 중 나오는 경고는 정상이다

```
[Warning] Stiffness and damping not available joint .../wheel1,
          actuator will be created without gain parameters
```
URDF에 `<dynamics>` 게인이 없는 조인트에서 나온다. 오류가 아니다.
임포트 후 5.7절 방식으로 게인을 설정하면 된다.

---

## ✅ 책이 맞은 부분 — 실행으로 확인

### 5.3 Franka 조인트 드라이브

```
num_dof = 9
dof_names = ['panda_joint1' … 'panda_joint7',
             'panda_finger_joint1', 'panda_finger_joint2']
```

책의 게인 배열이 **정확히 9개**(팔 7축 + 그리퍼 2축)로 실제 DOF와 일치한다.

```python
controller.set_gains(
    kps=np.array([600, 600, 600, 600, 250, 150, 50, 200, 200]),
    kds=np.array([ 50,  50,  50,  50,  30,  25, 15,  10,  10]),
)
```
`ArticulationAction`으로 목표 자세를 주고 60스텝 후 **최대 오차 0.0332 rad**로 수렴했다. ✅

### 5.7 자연 주파수 기반 PD 게인 공식

```
kp = (2π × fn)² × inertia
kd = 2 × ζ × (2π × fn) × inertia
```
`fn=10Hz, inertia=0.1, ζ=1.0` → **kp = 394.8, kd = 12.57**
책에 적힌 출력값과 **정확히 일치**한다. ✅

---

## 캡처

### 로봇 모델 개별 확인

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch05/ch05_robot_h1.png`](../_docs/images/book/ch05/ch05_robot_h1.png) | Unitree H1 휴머노이드 |
| [`../_docs/images/book/ch05/ch05_robot_spot.png`](../_docs/images/book/ch05/ch05_robot_spot.png) | Boston Dynamics Spot |
| [`../_docs/images/book/ch05/ch05_robot_carter.png`](../_docs/images/book/ch05/ch05_robot_carter.png) | NVIDIA Carter AMR |
| [`../_docs/images/book/ch05/ch05_robot_go2.png`](../_docs/images/book/ch05/ch05_robot_go2.png) | Unitree Go2 |

### H1 상세 검증

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch05/ch05_h1_standing_090312.png`](../_docs/images/book/ch05/ch05_h1_standing_090312.png) | H1 직립 자세 확인 |
| [`../_docs/images/book/ch05/ch05_h1_front.png`](../_docs/images/book/ch05/ch05_h1_front.png) | H1 정면 |
| [`../_docs/images/book/ch05/ch05_h1_closeup.png`](../_docs/images/book/ch05/ch05_h1_closeup.png) | H1 관절 클로즈업 |
| [`../_docs/images/book/ch05/ch05_stand_full_092718.png`](../_docs/images/book/ch05/ch05_stand_full_092718.png) | 전신 직립 |

### 로봇 갤러리 + AMR 주행

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch05/ch05_gallery_gui_153851.png`](../_docs/images/book/ch05/ch05_gallery_gui_153851.png) | GUI에서 로봇 갤러리 |
| [`../_docs/images/book/ch05/ch05_gallery_fixed_154123.png`](../_docs/images/book/ch05/ch05_gallery_fixed_154123.png) | 갤러리 수정 후 |
| [`../_docs/images/book/ch05/ch05_gallery_settled_154241.png`](../_docs/images/book/ch05/ch05_gallery_settled_154241.png) | 물리 안정화 후 |
| [`../_docs/images/book/ch05/ch05_gui_robots_152713.png`](../_docs/images/book/ch05/ch05_gui_robots_152713.png) | 다중 로봇 배치 |
| [`../_docs/images/book/ch05/ch05_amr_driving_152726.mp4`](../_docs/images/book/ch05/ch05_amr_driving_152726.mp4) | AMR 자율주행 영상 |

### 기타

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch05/ch05_facing_090612.png`](../_docs/images/book/ch05/ch05_facing_090612.png) | 로봇 정면 배치 |
| [`../_docs/images/book/ch05/ch05_facing_closeup.png`](../_docs/images/book/ch05/ch05_facing_closeup.png) | 정면 클로즈업 |
| [`../_docs/images/book/ch05/ch05_standing_155201.png`](../_docs/images/book/ch05/ch05_standing_155201.png) | 직립 자세 |
| [`../_docs/images/book/ch05/ch05_standing_closeup.png`](../_docs/images/book/ch05/ch05_standing_closeup.png) | 직립 클로즈업 |
| [`../_docs/images/book/ch05/ch05_final_155938.png`](../_docs/images/book/ch05/ch05_final_155938.png) | 최종 확인 |
| [`../_docs/images/book/ch05/ch05_final_closeup.png`](../_docs/images/book/ch05/ch05_final_closeup.png) | 최종 클로즈업 |
