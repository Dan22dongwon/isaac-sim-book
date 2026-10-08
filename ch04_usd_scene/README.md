# Chapter 4 — USD 씬 구성 실무

책 4장의 실습 코드다. **4개 파일 모두 Isaac Sim 6.1에서 실행을 확인했다.**

## 실행 방법

```bash
conda activate isaac_env
export OMNI_KIT_ACCEPT_EULA=YES
python warehouse_scene.py
```

## 파일 목록

| 파일 | 책 위치 | 상태 | 비고 |
|---|---|---|---|
| [`warehouse_scene.py`](warehouse_scene.py) | 4.7 [실습] 산업용 창고 환경 | ✅ 동작 | **수정 3건 + 카메라 추가** |
| [`warehouse_scene_original.py`](warehouse_scene_original.py) | 4.7 원문 | ❌ **실행 불가** | 비교 학습용으로만 보존 |
| [`lighting.py`](lighting.py) | 4.5 조명 설정 | ✅ 동작 | **이동 API 교체** |
| [`variant_asset.py`](variant_asset.py) | 4.8.5 [실습] Variant Set 자산 | ✅ 동작 | **import 분리** |
| [`light_direction_test.py`](light_direction_test.py) | — | ✅ 동작 | 조명 방향 검증 스크립트 (책에 없음) |

---

## 원문과 무엇이 다른가

### `warehouse_scene.py` — 수정 3건

원문을 그대로 실행하면 **65번째 줄에서 죽는다.**

```
pxr.Tf.ErrorException:
  Error in 'UsdStage::_IsValidPathForCreatingPrim' : 'Path must be an absolute path: <>'
```

#### 수정 1·2 — USD prim 이름 규칙 위반 (ERR-020)

```python
# ❌ 원문
for dx, dy in [(-0.9, -1.8), (0.9, -1.8), (-0.9, 1.8), (0.9, 1.8)]:
    prim_path=f"/World/Environment/Shelf_{i}/Post_{dx}_{dy}"
    #  실제 생성 경로 → "Post_-0.9_-1.8"   ← '-' 와 '.' 은 USD 금지 문자

for floor_h in [1.0, 2.0, 3.0]:
    prim_path=f"/World/Environment/Shelf_{i}/Board_{floor_h}"
    #  실제 생성 경로 → "Board_1.0"        ← '.' 금지

# ✅ 수정
for j, (dx, dy) in enumerate([(-0.9, -1.8), (0.9, -1.8), (-0.9, 1.8), (0.9, 1.8)]):
    prim_path=f"/World/Environment/Shelf_{i}/Post_{j}"

for k, floor_h in enumerate([1.0, 2.0, 3.0]):
    prim_path=f"/World/Environment/Shelf_{i}/Board_{k}"
```

> **왜?** USD prim 이름은 C 식별자 규칙을 따른다 — `[A-Za-z_][A-Za-z0-9_]*`.
> 마침표(`.`)는 USD에서 **속성 구분자**(`prim.attribute`)로 예약돼 있고, 하이픈(`-`)은
> 식별자에 쓸 수 없다. **f-string으로 경로를 만들 때 좌표값을 그대로 끼워 넣으면 이 사고가 난다.**

#### 수정 3 — 조명이 위를 향한다 (ERR-021)

```python
# ❌ 원문
xform.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))  # 아래로

# ✅ 수정 — 그 줄을 지운다
```

UsdLux의 면 광원(RectLight/DiskLight)은 **로컬 -Z 방향으로 발광**한다. 즉 회전을 주지 않은
상태가 이미 "아래로"다. X축 180° 회전은 빛을 +Z(위)로 뒤집어 천장만 비추게 만든다.

같은 씬·같은 카메라로 렌더링해 픽셀 밝기를 측정한 결과:

| 조명 회전 | 평균 밝기 (0–255) | 밝은 픽셀 비율 |
|---|---|---|
| 원문 `RotateXYZ(180,0,0)` | **0.01** | **0.0 %** |
| 수정 (회전 없음) | 12.58 | 21.2 % |

원문 코드에서는 밝은 픽셀이 **하나도 없다.** 에러는 나지 않으므로, 독자는
"씬은 만들어졌다는데 왜 안 보이지?" 하며 카메라만 돌리게 된다.
직접 확인하려면 [`light_direction_test.py`](light_direction_test.py)를 실행하면 된다.

#### 추가 — 뷰포트 카메라

원문에는 카메라 배치가 없어 기본 시점에서 씬이 잘 보이지 않는다. 다음을 추가했다.

```python
from isaacsim.core.utils.viewports import set_camera_view
set_camera_view(eye=np.array([14.0, -20.0, 9.0]), target=np.array([0.0, 0.0, 1.5]))
```

### 실행 결과

수정본의 출력은 **책의 "예상 결과"와 한 글자도 다르지 않다.**

```
✅ 창고 씬 생성 완료!
  - 바닥: 20m x 30m
  - 벽: 4면, 높이 5m
  - 선반: 3개 (3단)
  - 낙하 박스: 5개
  - 천장 조명: 6개
  Step   0 진행 중...
  Step  60 진행 중...
  Step 120 진행 중...
  Step 180 진행 중...
  Step 240 진행 중...
```

즉 저자는 동작하는 코드로 한 번 실행했고, 이후 편집 과정에서 코드가 깨진 것으로 보인다.

---

### `lighting.py` — 이동 API 교체 (ERR-011)

```python
# ❌ 원문
from isaacsim.core.utils.prims import set_prim_translation, set_prim_rotation
set_prim_translation(prim_path="/World/Lights/CeilingLight_01", translation=[0.0, 0.0, 5.0])
# ImportError: cannot import name 'set_prim_translation' from 'isaacsim.core.utils.prims'

# ✅ 수정
from pxr import Gf, UsdGeom
xform_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/Lights/CeilingLight_01"))
xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 5.0))
```

`isaacsim.core.utils.prims`의 `set_` 계열 함수 전체 목록에 이동/회전이 없다.
```
set_prim_attribute_value, set_prim_visibility, set_prim_property,
set_prim_hide_in_stage_window, set_prim_no_delete, set_targets
```

> 📌 **조명 밝기는 Python API 쪽이 정답이다.**
> 책 2.1절의 `.usda` 텍스트 예시는 `float intensity = 3000`으로 적혀 있는데 이건 틀렸다
> (→ ERR-007). 반면 **`CreateIntensityAttr()`는 자동으로 `inputs:intensity`를 만든다.**
> 실행 결과로 확인된다:
> ```
> RectLight 속성 이름 : inputs:intensity
> RectLight intensity : 5000.0
> ```

---

### `variant_asset.py` — import 분리 (ERR-001)

```python
# ❌ 원문 — 세 줄이 한 줄로 붙어 있어 즉시 SyntaxError
import omni.usdfrom isaacsim.storage import native as nucleus_utilsfrom pxr import Usd, UsdGeom, Sdf

# ✅ 수정
import omni.usd
from isaacsim.storage import native as nucleus_utils
from pxr import Usd, UsdGeom, Sdf
```

또한 책의 코드 조각에는 `SimulationApp` 기동 부분이 없어 단독 실행이 안 된다.
실행 가능하도록 앞뒤에 다음을 추가했다.

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
...
app.close()
```

실행 결과:
```
✅ Variant + Payload 최적화 자산 저장 완료
   그리퍼 변형: robotiq_2f85, robotiq_hand_e
   센서: Payload로 분리 (기본 미로드)
→ ur5e_configurable.usd 생성
```

---

## 이 장에서 확인된 정상 동작 API

Isaac Sim 6.1에서 직접 실행해 확인한 것들이다. 책이 맞게 쓴 부분이다.

```
✅ omni.usd.get_context().get_stage()
✅ UsdGeom.SetStageUpAxis / SetStageMetersPerUnit
✅ stage.GetRootLayer().subLayerPaths / stage.GetLayerStack()
✅ isaacsim.storage.native.get_assets_root_path()
✅ isaacsim.core.utils.stage.add_reference_to_stage(usd_path, prim_path, prim_type)
✅ UsdPhysics.Scene.Define / CreateGravityDirectionAttr / CreateGravityMagnitudeAttr
✅ PhysxSchema.PhysxSceneAPI.Apply / EnableCCD / EnableGPUDynamics / BroadphaseType("MBP")
✅ UsdPhysics.CollisionAPI.Apply / omni.physx.scripts.utils.setCollider
✅ UsdPhysics.MaterialAPI (StaticFriction / DynamicFriction / Restitution)
✅ stage.SetDefaultPrim / GetDefaultPrim
✅ Prim.GetVariantSets().AddVariantSet / AddVariant / SetVariantSelection
✅ Prim.SetInstanceable / GetPayloads / Load / Unload
```

---

## 캡처

| 파일 | 내용 |
|---|---|
| [`../_docs/images/book/ch04/ch4_warehouse_scene_100211.png`](../_docs/images/book/ch04/ch4_warehouse_scene_100211.png) | 창고 씬 초기 구성 |
| [`../_docs/images/book/ch04/ch4_warehouse_scene_100609.png`](../_docs/images/book/ch04/ch4_warehouse_scene_100609.png) | 창고 씬 조명 조정 후 |
| [`../_docs/images/book/ch04/ch4_warehouse_fixed_100824.png`](../_docs/images/book/ch04/ch4_warehouse_fixed_100824.png) | 충돌 설정 수정 후 |
| [`../_docs/images/book/ch04/ch4_warehouse_boxes_100211.mp4`](../_docs/images/book/ch04/ch4_warehouse_boxes_100211.mp4) | 박스 물리 시뮬레이션 영상 |
