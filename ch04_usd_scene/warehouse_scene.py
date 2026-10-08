import isaacsim
from isaacsim.simulation_app import SimulationApp

app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
import isaacsim.core.utils.stage as stage_utils
import isaacsim.core.utils.prims as prim_utils
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid, DynamicCuboid
from pxr import UsdGeom, UsdPhysics, UsdLux, Gf, PhysxSchema

# ── 1. World 생성 ──────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# ── 2. 물리 씬 설정 ────────────────────────────────────
from isaacsim.core.api.physics_context import PhysicsContext
physics = PhysicsContext()
physics.set_gravity(-9.81)

# ── 3. 창고 바닥 (20m x 30m) ──────────────────────────
floor = world.scene.add(
    FixedCuboid(
        prim_path="/World/Environment/Floor",
        name="warehouse_floor",
        position=np.array([0.0, 0.0, -0.05]),  # 바닥 두께 10cm
        scale=np.array([20.0, 30.0, 0.1]),
        color=np.array([0.4, 0.4, 0.45]),      # 회색 콘크리트
    )
)

# ── 4. 창고 벽 (4면) ───────────────────────────────────
wall_configs = [
    # (이름, 위치, 크기)
    ("North", [0.0,  15.0, 2.5], [20.0, 0.3, 5.0]),
    ("South", [0.0, -15.0, 2.5], [20.0, 0.3, 5.0]),
    ("East",  [10.0,  0.0, 2.5], [0.3, 30.0, 5.0]),
    ("West",  [-10.0, 0.0, 2.5], [0.3, 30.0, 5.0]),
]

for name, pos, scale in wall_configs:
    world.scene.add(
        FixedCuboid(
            prim_path=f"/World/Environment/Wall_{name}",
            name=f"wall_{name.lower()}",
            position=np.array(pos),
            scale=np.array(scale),
            color=np.array([0.8, 0.8, 0.75]),
        )
    )

# ── 5. 선반 3개 배치 ───────────────────────────────────
shelf_positions = [
    [-6.0, 0.0],   # 왼쪽 선반
    [ 0.0, 0.0],   # 가운데 선반
    [ 6.0, 0.0],   # 오른쪽 선반
]

for i, (sx, sy) in enumerate(shelf_positions):
    # 선반 기둥 (4개)
    # [수정 ERR-020] prim 이름에 좌표(-0.9 등)를 쓰면 USD 규칙 위반 → 인덱스 사용
    for j, (dx, dy) in enumerate([(-0.9, -1.8), (0.9, -1.8), (-0.9, 1.8), (0.9, 1.8)]):
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Environment/Shelf_{i}/Post_{j}",
                name=f"shelf{i}_post_{j}",
                position=np.array([sx+dx, sy+dy, 1.5]),
                scale=np.array([0.1, 0.1, 3.0]),
                color=np.array([0.6, 0.5, 0.3]),
            )
        )
    # 선반 판 (3단)
    # [수정 ERR-020] Board_1.0 의 '.' 도 USD prim 이름에 쓸 수 없다
    for k, floor_h in enumerate([1.0, 2.0, 3.0]):
        world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Environment/Shelf_{i}/Board_{k}",
                name=f"shelf{i}_board_{k}",
                position=np.array([sx, sy, floor_h]),
                scale=np.array([2.0, 4.0, 0.05]),
                color=np.array([0.7, 0.6, 0.4]),
            )
        )

# ── 6. 낙하 박스 (물리 테스트용) ─────────────────────
for i in range(5):
    world.scene.add(
        DynamicCuboid(
            prim_path=f"/World/Props/Box_{i:02d}",
            name=f"box_{i:02d}",
            position=np.array([i*0.5 - 1.0, 0.0, 4.0]),
            scale=np.array([0.3, 0.3, 0.3]),
            color=np.array([0.9, 0.3, 0.1]),
            mass=1.0,
        )
    )

# ── 7. 천장 조명 배치 ──────────────────────────────────
light_positions = [
    [-5.0, -10.0], [5.0, -10.0],
    [-5.0,   0.0], [5.0,   0.0],
    [-5.0,  10.0], [5.0,  10.0],
]

for i, (lx, ly) in enumerate(light_positions):
    rect_light = UsdLux.RectLight.Define(
        stage, f"/World/Lights/CeilingLight_{i:02d}"
    )
    rect_light.CreateIntensityAttr().Set(8000.0)
    rect_light.CreateWidthAttr().Set(1.5)
    rect_light.CreateHeightAttr().Set(0.5)
    rect_light.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.95, 0.85))
    # 위치 설정
    xform = UsdGeom.Xformable(rect_light.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(lx, ly, 4.8))
    # [수정 ERR-021] RectLight는 로컬 -Z(아래)로 발광하는 것이 기본이다.
    # 원문의 X축 180도 회전은 빛을 위(+Z)로 뒤집어 씬이 완전히 캄캄해진다.

# ── 8. 씬 초기화 & 실행 ───────────────────────────────
world.reset()

# [추가] 뷰포트 카메라 배치 — 원문에는 없다. 기본 카메라로는 씬이 잘 안 보인다.
from isaacsim.core.utils.viewports import set_camera_view
set_camera_view(eye=np.array([14.0, -20.0, 9.0]),
                target=np.array([0.0, 0.0, 1.5]))
print("✅ 창고 씬 생성 완료!")
print(f"  - 바닥: 20m x 30m")
print(f"  - 벽: 4면, 높이 5m")
print(f"  - 선반: {len(shelf_positions)}개 (3단)")
print(f"  - 낙하 박스: 5개")
print(f"  - 천장 조명: {len(light_positions)}개")

# 시뮬레이션 실행 (박스 낙하 확인)
for step in range(300):
    world.step(render=True)
    if step % 60 == 0:
        print(f"  Step {step:3d} 진행 중...")

app.close()
print("\n✅ 창고 씬 실습 완료!")
