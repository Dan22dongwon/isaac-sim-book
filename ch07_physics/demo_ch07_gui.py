"""7장 실습 결과를 **화면으로 직접 보는** 데모.

같은 탑을 눈앞에서 **무너뜨렸다 세웠다** 반복한다. 바뀌는 것은 솔버 반복 횟수뿐이다.

  1단계  반복 1회  → 상자들이 서로 파고들어 무너진다
  2단계  반복 32회 → 곧게 선다

옆에서는 상자 둘이 외력을 받는다 (7.4 절).
  파랑  순간 충격 — 한 스텝만 밀고 그 뒤 등속
  주황  지속 압력 — 매 스텝 밀어 계속 가속

## 왜 나란히 비교하지 않고 번갈아 보여 주는가

솔버 반복은 **씬 단위(PhysxSceneAPI)** 설정이다. 물체 단위 설정
(`PhysxRigidBodyAPI.CreateSolverPositionIterationCountAttr`)도 있어서 처음에는
탑 셋을 나란히 세우려 했는데, **실측해 보니 물체 단위로는 차이가 나지 않았다**
(dt=1/60 에서 1회·4회·32회 모두 1.100 m 로 정상). 그래서 같은 탑을 두고
씬 설정을 바꿔 가며 보여 주는 방식으로 바꿨다.

## 타임스텝에 따라 결과가 달라진다

| physics_dt | 반복 1회 | 반복 4회 | 반복 32회 |
|---|---:|---:|---:|
| **1/60** | **0.100 m 무너짐** | 1.096 m | 1.100 m |
| 1/120 | 1.079 m | 1.099 m | 1.100 m |
| 1/240 | 1.096 m | 1.100 m | 1.100 m |

타임스텝을 줄이면 반복이 적어도 버틴다. **둘은 서로를 보완한다** — 이것이
7장의 교환관계다. 이 데모는 차이가 가장 뚜렷한 **1/60** 을 쓴다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python demo_ch07_gui.py              # 기본 3분
    MINUTES=6 python demo_ch07_gui.py
"""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": False})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import RigidPrim  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, Vt  # noqa: E402

import omni.usd  # noqa: E402

MINUTES = float(os.environ.get("MINUTES", "3"))

DT = 1.0 / 60          # 차이가 가장 뚜렷한 타임스텝
N_BOX, SIZE, MASS = 6, 0.20, 5.0
IDEAL_TOP = SIZE / 2 + (N_BOX - 1) * SIZE
PHASE = int(18 / DT)   # 18초마다 설정을 바꾼다
FORCE = 40.0

World.clear_instance()
world = World(stage_units_in_meters=1.0, physics_dt=DT)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

scene = [p for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene"][0]
phys = PhysxSchema.PhysxSceneAPI.Apply(scene)
phys.CreateSolverTypeAttr().Set("TGS")
phys.CreateEnableStabilizationAttr().Set(False)   # 차이를 또렷이 보려고 끈다


def set_iterations(n):
    """씬 단위 솔버 반복을 바꾼다.

    ⚠️ 속성만 바꾸면 **반영되지 않는다.** PhysX 는 이 값을 씬을 올릴 때 읽는다.
    반드시 world.reset() 을 거쳐야 한다 (아래 apply_phase 참고).

        속성만 변경            → 꼭대기 0.100m (반영 안 됨)
        속성 변경 + world.reset() → 꼭대기 1.100m (반영됨)
    """
    phys.CreateMinPositionIterationCountAttr().Set(n)
    phys.CreateMaxPositionIterationCountAttr().Set(n)
    phys.CreateMinVelocityIterationCountAttr().Set(1)
    phys.CreateMaxVelocityIterationCountAttr().Set(1)


def box(path, pos, color, mass=MASS, size=SIZE):
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(size)
    c.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr().Set(mass)
    return path


# ── 탑 (7.1 솔버) ────────────────────────────────────────────────
stage.DefinePrim("/World/Tower", "Xform")
TOWER = [box(f"/World/Tower/box_{i:02d}", (0.0, 0.0, SIZE / 2 + i * SIZE),
             (0.80, 0.82, 0.86)) for i in range(N_BOX)]

# ── 외력 상자 (7.4) ──────────────────────────────────────────────
IMP = box("/World/BoxImpulse", (-1.6, 0.0, SIZE / 2), (0.25, 0.45, 0.90))
PER = box("/World/BoxPermanent", (-1.6, 0.6, SIZE / 2), (0.95, 0.55, 0.15))

key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(16000.0)
key.CreateWidthAttr().Set(10.0)
key.CreateHeightAttr().Set(10.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(1.0, -3.0, 4.5))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(1100.0)

set_iterations(1)
world.reset()

boxes = []
for i, p in enumerate(TOWER):
    v = RigidPrim(prim_paths_expr=p, name=f"tb{i}")
    v.initialize()
    boxes.append(v)
imp = RigidPrim(prim_paths_expr=IMP, name="imp")
per = RigidPrim(prim_paths_expr=PER, name="per")
imp.initialize()
per.initialize()

HOME = np.array([[0.0, 0.0, SIZE / 2 + i * SIZE] for i in range(N_BOX)])
IMP_HOME = np.array([[-1.6, 0.0, SIZE / 2]])
PER_HOME = np.array([[-1.6, 0.6, SIZE / 2]])
UP = np.array([[1.0, 0.0, 0.0, 0.0]])


def reset_all():
    """탑과 상자를 제자리로 돌린다. 속도도 반드시 0으로 (안 그러면 튄다)."""
    for i, v in enumerate(boxes):
        v.set_world_poses(positions=HOME[i:i + 1], orientations=UP)
        v.set_velocities(np.zeros((1, 6)))
    imp.set_world_poses(positions=IMP_HOME, orientations=UP)
    imp.set_velocities(np.zeros((1, 6)))
    per.set_world_poses(positions=PER_HOME, orientations=UP)
    per.set_velocities(np.zeros((1, 6)))


def top_z():
    return float(np.asarray(boxes[-1].get_world_poses()[0])[0][2])


def apply_phase(n):
    """솔버 반복을 n 으로 바꾸고 **실제로 반영시킨다.**

    world.reset() 이 핵심이다. 그 뒤 뷰를 다시 initialize 해야 한다 —
    리셋으로 물리 핸들이 새로 만들어지기 때문이다.
    """
    set_iterations(n)
    world.reset()
    for v in boxes + [imp, per]:
        v.initialize()
    reset_all()
    world.step(render=True)


try:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[1.6, -4.0, 1.7], target=[-0.5, 0.2, 0.6])
except Exception as e:
    print(f"(뷰포트 이동 실패: {e})", flush=True)

print("\n" + "=" * 68, flush=True)
print("  7장 데모 — 창을 보세요", flush=True)
print("=" * 68, flush=True)
print(f"  가운데 흰 탑   상자 {N_BOX}개, 각 {MASS:.0f}kg. 정상 높이 {IDEAL_TOP:.2f}m", flush=True)
print(f"                 {18}초마다 **솔버 반복 횟수만** 바꿉니다.", flush=True)
print(f"                   반복  1회 → 서로 파고들어 무너짐", flush=True)
print(f"                   반복 32회 → 곧게 섬", flush=True)
print(f"\n  왼쪽 파란 상자  순간 충격 — 한 번만 밀고 그 뒤 **등속**", flush=True)
print(f"  왼쪽 주황 상자  지속 압력 — 매 스텝 밀어 계속 **가속**", flush=True)
print(f"                 둘 다 {FORCE:.0f}N. 같은 힘인데 결과가 다릅니다.", flush=True)
print(f"\n  마우스 좌클릭 드래그=회전  휠=확대  가운데드래그=이동", flush=True)
print(f"  {MINUTES:.0f}분 뒤 자동 종료. 먼저 닫으려면 이 터미널에서 Ctrl+C.", flush=True)
print("=" * 68 + "\n", flush=True)

F = np.array([[FORCE, 0.0, 0.0]])
ITERS = [1, 32]
steps = int(MINUTES * 60 / DT)
phase = -1

try:
    for s in range(steps):
        p = (s // PHASE) % 2
        if p != phase:                      # 단계가 바뀌면 설정 변경 + 반영
            phase = p
            apply_phase(ITERS[p])
            print(f"\n  ── 솔버 반복 {ITERS[p]}회 ── ({s*DT:.0f}초)", flush=True)

        # 외력: 파랑은 단계 시작 때 한 번만, 주황은 매 스텝
        if s % PHASE == 1:
            imp.apply_forces(F, is_global=True)
        per.apply_forces(F, is_global=True)

        world.step(render=True)

        if s % int(6 / DT) == 0:
            z = top_z()
            print(f"    [{s*DT:5.0f}초] 탑 꼭대기 {z:.3f}m "
                  f"{'✅ 서 있음' if abs(z - IDEAL_TOP) < 0.05 else '❌ 무너짐'}"
                  f"   파랑 v={float(np.asarray(imp.get_velocities())[0][0]):+.2f}"
                  f"  주황 v={float(np.asarray(per.get_velocities())[0][0]):+.2f} m/s",
                  flush=True)
except KeyboardInterrupt:
    print("\n  사용자가 중단했습니다.", flush=True)

print("\n  데모 종료", flush=True)
app.close()
