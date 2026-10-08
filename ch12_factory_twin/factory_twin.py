"""12장 b10 — 물류 창고 디지털 트윈 통합 (원고 수정본)

컨베이어 + AMR 2대 + 선반 3개를 한 씬에 넣고 돌린다.

원고 대비 고친 곳
------------------------------------------------------------------
  [ERR-087] PhysxConveyorBeltAPI 는 Isaac Sim 6.1 에 없다.
            PhysxSurfaceVelocityAPI 로 대체한다.

  [ERR-088] 원고 주석 "1m × 6m × 0.1m" 은 실제 치수의 2배다.
            size=1, scale=(0.5, 3.0, 0.05) → 0.5m × 3.0m × 0.05m.

  [ERR-089] Carter 경로 /Isaac/Robots/Carter/ →
            /Isaac/Robots/NVIDIA/Carter/ (6.1 에서 변경됨).

  [ERR-092] 원고는 `experimental.wheeled_robots` 에서 import 하면서
            비-experimental API(`prim_path`, `name`, `create_robot`)를 쓴다.
            experimental 버전의 시그니처는 `paths`, `positions` 등 다르다.
            비-experimental 모듈을 쓰면 원고의 호출 패턴이 동작한다.

실행:
    conda activate isaac_env
    python factory_twin.py
    HEADLESS=1 python factory_twin.py
"""
import os

import numpy as np
from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
app = SimulationApp({"headless": HEADLESS})

from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf  # noqa: E402
import omni.usd                                        # noqa: E402
from isaacsim.storage import native as nu               # noqa: E402
import isaacsim.core.utils.stage as stage_utils         # noqa: E402
from isaacsim.core.api import World                     # noqa: E402
from isaacsim.core.api.objects import FixedCuboid, DynamicCuboid  # noqa: E402
from isaacsim.robot.wheeled_robots.robots import WheeledRobot       # noqa: E402
from isaacsim.robot.wheeled_robots.controllers import DifferentialController  # noqa: E402

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
nucleus = nu.get_assets_root_path()
world.scene.add_default_ground_plane()

# ── 1. 컨베이어 벨트 ──────────────────────────────────────
belt = UsdGeom.Cube.Define(stage, "/World/Conveyor")
belt.GetSizeAttr().Set(1.0)
xf = UsdGeom.Xformable(belt)
xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))
xf.AddScaleOp().Set(Gf.Vec3f(0.5, 3.0, 0.05))
UsdPhysics.CollisionAPI.Apply(belt.GetPrim())
rb = UsdPhysics.RigidBodyAPI.Apply(belt.GetPrim())
rb.CreateKinematicEnabledAttr().Set(True)

# [ERR-087] PhysxConveyorBeltAPI → PhysxSurfaceVelocityAPI
sv = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(belt.GetPrim())
sv.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, 0.4, 0.0))
sv.CreateSurfaceVelocityEnabledAttr().Set(True)

# ── 2. 컨베이어 박스 ──────────────────────────────────────
conv_box = world.scene.add(
    DynamicCuboid(
        prim_path="/World/ConvBox", name="conv_box",
        position=np.array([0.0, -1.0, 0.60]),
        scale=np.array([0.15, 0.15, 0.15]),
        color=np.array([0.9, 0.5, 0.1]), mass=0.5,
    )
)

# ── 3. AMR 2대 ────────────────────────────────────────────
# [ERR-089] /Isaac/Robots/Carter/ → /Isaac/Robots/NVIDIA/Carter/
carter_usd = nucleus + "/Isaac/Robots/NVIDIA/Carter/carter_v1.usd"
amrs = []
for i, x in enumerate([-4.0, 4.0]):
    amr = world.scene.add(
        WheeledRobot(
            prim_path=f"/World/AMR_{i}", name=f"amr_{i}",
            wheel_dof_names=["left_wheel", "right_wheel"],
            create_robot=True, usd_path=carter_usd,
            position=np.array([x, -8.0, 0.1]),
        )
    )
    amrs.append(amr)

# ── 4. 창고 선반 (간이) ───────────────────────────────────
for i, x in enumerate([-5, 0, 5]):
    world.scene.add(
        FixedCuboid(
            prim_path=f"/World/Shelf_{i}", name=f"shelf_{i}",
            position=np.array([float(x), 5.0, 1.0]),
            scale=np.array([1.5, 3.0, 2.0]),
            color=np.array([0.6, 0.5, 0.35]),
        )
    )

# ── 5. 실행 ───────────────────────────────────────────────
world.reset()
controllers = [
    DifferentialController(f"ctrl_{i}", wheel_radius=0.125, wheel_base=0.46)
    for i in range(2)
]

if not HEADLESS:
    from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
    set_camera_view(
        eye=np.array([12.0, -12.0, 10.0]),
        target=np.array([0.0, 0.0, 0.5]),
    )

STEPS = int(os.environ.get("STEPS", "800"))
print(f"\n물류 창고 디지털 트윈 — {STEPS} 스텝")
print(f"  컨베이어 벨트: 0.4 m/s (SurfaceVelocityAPI)")
print(f"  AMR: 2대 (Carter v1)")
print(f"  선반: 3개\n")

for step in range(STEPS):
    for i, (amr, ctrl) in enumerate(zip(amrs, controllers)):
        cmd = np.array([0.3, 0.1 if i == 0 else -0.1])
        action = ctrl.forward(command=cmd)
        amr.apply_wheel_actions(action)

    world.step(render=not HEADLESS)

    if step % 200 == 0:
        box_pos, _ = conv_box.get_world_pose()
        amr_positions = []
        for amr in amrs:
            p, _ = amr.get_world_pose()
            amr_positions.append(f"({p[0]:+.1f}, {p[1]:+.1f})")
        print(f"  step {step:4d}  박스 y={box_pos[1]:+.2f}m  "
              f"AMR {' '.join(amr_positions)}")

# 최종 결과
box_final, _ = conv_box.get_world_pose()
box_moved = box_final[1] - (-1.0)
print(f"\n결과:")
print(f"  박스 이동: {box_moved*100:+.1f} cm")
print(f"  벨트 작동: {'✅' if box_moved > 0.1 else '❌'}")
for i, amr in enumerate(amrs):
    p, _ = amr.get_world_pose()
    dist = float(np.linalg.norm(np.array([p[0], p[1]]) - np.array([(-4.0, 4.0)[i], -8.0])))
    print(f"  AMR_{i} 이동거리: {dist:.1f} m {'✅' if dist > 1.0 else '❌'}")

if not HEADLESS:
    for _ in range(int(os.environ.get("HOLD", "600"))):
        world.step(render=True)

app.close()
