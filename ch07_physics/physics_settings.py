"""책 7.1 타임스텝 · 솔버 설정 — 오류 3건을 고쳐 동작시킨 판본.

원문과의 차이:
  [ERR-037] 솔버 반복 속성 Min/MaxIterationCount 가 **존재하지 않는다.**
            PhysX 는 위치와 속도 반복을 따로 세므로 속성이 네 개다.
              MinPositionIterationCount / MaxPositionIterationCount
              MinVelocityIterationCount / MaxVelocityIterationCount
  [ERR-038] PhysicsScene 경로가 '/World/physicsScene' 이 아니라 '/physicsScene'.
            틀린 경로로 Apply() 하면 ErrorException 이 난다 (조용히 넘어가지는 않는다).
  [ERR-039] PhysicsContext(substeps=2) — 그런 인자가 없다.
            실제 인자는 physics_dt, prim_path, sim_params, set_defaults 뿐이다.
            원문 주석의 "실효 타임스텝 1/480s" 도 따라서 성립하지 않는다.

  [주의-001] **World 는 싱글턴이다.** 두 번째 World(physics_dt=...) 는 새 객체를
            만들지 않고 기존 인스턴스를 돌려주며, **생성자 인자를 조용히 무시한다.**
            타임스텝을 바꾸려면 World.clear_instance() 를 부르거나
            world.set_simulation_dt(physics_dt=...) 를 쓴다.
            원고에는 이 언급이 없다. 여러 설정을 비교하는 코드에서 반드시 걸린다.

이 파일은 '고쳤다'로 끝내지 않는다. **고친 설정이 실제로 먹히는지**를
자유낙하 거리로 역산해 확인한다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python physics_settings.py
"""
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import RigidPrim  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402

import omni.usd  # noqa: E402

G = 9.81


# ── [ERR-038] 경로를 하드코딩하지 않는다 ─────────────────────────
def find_physics_scene(stage):
    """World() 가 만든 PhysicsScene 을 찾아 돌려준다.

    원문은 '/World/physicsScene' 을 하드코딩하는데 실제 위치가 다르다.
    버전에 따라 또 바뀔 수 있으므로 타입으로 찾는 편이 안전하다.
    """
    for p in stage.Traverse():
        if p.GetTypeName() == "PhysicsScene":
            return p
    raise RuntimeError("PhysicsScene 을 찾지 못했다")


# ── [ERR-037] 올바른 속성 이름으로 솔버를 설정한다 ───────────────
def configure_solver(stage, pos_iter=8, vel_iter=1, *, stabilization=True):
    api = PhysxSchema.PhysxSceneAPI.Apply(find_physics_scene(stage))
    api.CreateSolverTypeAttr().Set("TGS")
    api.CreateMinPositionIterationCountAttr().Set(pos_iter)
    api.CreateMaxPositionIterationCountAttr().Set(max(pos_iter, 64))
    api.CreateMinVelocityIterationCountAttr().Set(vel_iter)
    api.CreateMaxVelocityIterationCountAttr().Set(max(vel_iter, 8))
    # 아래 셋은 원고가 맞다 — 전부 존재한다
    api.CreateSolveArticulationContactLastAttr().Set(True)
    api.CreateEnableStabilizationAttr().Set(stabilization)
    api.CreateEnableCCDAttr().Set(False)
    return api


def drop_ball(stage, path, z0):
    s = UsdGeom.Sphere.Define(stage, path)
    s.GetRadiusAttr().Set(0.05)
    UsdGeom.Xformable(s.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z0))
    UsdPhysics.RigidBodyAPI.Apply(s.GetPrim())
    UsdPhysics.MassAPI.Apply(s.GetPrim()).CreateMassAttr().Set(1.0)
    # 충돌은 주지 않는다 — 바닥에 닿으면 자유낙하가 끝나 측정이 망가진다
    return s


def measure_dt(physics_dt, steps=120, z0=50.0):
    """자유낙하 거리로 **실제** 타임스텝을 역산한다.

    설정값을 읽어 보는 것만으로는 '먹혔다'를 알 수 없다.
    낙하 거리 d = ½·g·t² 이므로  t = √(2d/g),  dt = t / steps.
    """
    # [주의-001] World 는 싱글턴이다. 비우지 않으면 두 번째 호출부터
    # physics_dt 인자가 **조용히 무시되고** 이전 값이 그대로 쓰인다.
    World.clear_instance()
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt,
                  rendering_dt=physics_dt)
    stage = omni.usd.get_context().get_stage()
    configure_solver(stage)
    drop_ball(stage, "/World/Ball", z0)
    world.reset()

    v = RigidPrim(prim_paths_expr="/World/Ball", name=f"ball_{physics_dt:.6f}")
    v.initialize()
    v.set_velocities(np.zeros((1, 6)))
    start = float(np.asarray(v.get_world_poses()[0])[0][2])

    for _ in range(steps):
        world.step(render=False)

    end = float(np.asarray(v.get_world_poses()[0])[0][2])
    drop = start - end
    t = float(np.sqrt(max(drop, 0.0) * 2.0 / G))
    measured = t / steps
    reported = world.get_physics_dt()
    world.clear()
    return reported, measured, drop


print("\n===SETTINGS===", flush=True)

# ── ERR-038 — 원고 경로는 유효하지 않다 ──────────────────────────
print("[ERR-038] 원고의 PhysicsScene 경로 확인", flush=True)
World.clear_instance()
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

wrong = stage.GetPrimAtPath("/World/physicsScene")       # 원고가 쓰는 경로
right = find_physics_scene(stage)
print(f"  원고 경로 '/World/physicsScene' → 유효한가? {wrong.IsValid()}", flush=True)
print(f"  실제 경로 '{right.GetPath()}'  → 유효한가? {right.IsValid()}", flush=True)

try:
    api_w = PhysxSchema.PhysxSceneAPI.Apply(wrong)
    api_w.CreateMinPositionIterationCountAttr().Set(32)
    print(f"  ⚠ 틀린 경로인데 예외 없이 통과했다 "
          f"(읽으면 {api_w.GetMinPositionIterationCountAttr().Get()})", flush=True)
except Exception as e:
    head = str(e).strip().splitlines()[0] if str(e).strip() else ""
    print(f"  ✅ 틀린 경로 → {type(e).__name__} 발생 (조용히 넘어가지는 않는다)",
          flush=True)
    print(f"     {head[:80]}", flush=True)

api_r = configure_solver(stage, pos_iter=32, vel_iter=4)
print(f"  ✅ 올바른 경로에 설정 → MinPositionIterationCount = "
      f"{api_r.GetMinPositionIterationCountAttr().Get()}, "
      f"SolverType = {api_r.GetSolverTypeAttr().Get()}", flush=True)
world.clear()

# ── ERR-037 — 원고 속성 이름은 아예 없다 ─────────────────────────
print("\n[ERR-037] 원고의 속성 이름이 존재하는지 확인", flush=True)
for name in ("CreateMinIterationCountAttr", "CreateMaxIterationCountAttr"):
    print(f"  원고 {name:<32} → "
          f"{'있음' if hasattr(PhysxSchema.PhysxSceneAPI, name) else '❌ 없음'}", flush=True)
for name in ("CreateMinPositionIterationCountAttr", "CreateMaxPositionIterationCountAttr",
             "CreateMinVelocityIterationCountAttr", "CreateMaxVelocityIterationCountAttr"):
    print(f"  수정 {name:<36} → "
          f"{'✅ 있음' if hasattr(PhysxSchema.PhysxSceneAPI, name) else '없음'}", flush=True)

# ── ERR-039 — substeps 대신 physics_dt 를 직접 준다 ──────────────
print("\n[ERR-039] PhysicsContext(substeps=) 대신 physics_dt 로 지정한다", flush=True)
try:
    from isaacsim.core.api.physics_context import PhysicsContext
    PhysicsContext(physics_dt=1.0 / 240, substeps=2)
    print("  원고 방식 → 통과 (예상 밖)", flush=True)
except TypeError as e:
    print(f"  원고 방식 PhysicsContext(substeps=2) → ❌ {e}", flush=True)

print("\n  수정 방식 — 자유낙하 거리로 **실제** 타임스텝을 역산해 확인한다", flush=True)
print(f"    d = ½·g·t² 이므로 t = √(2d/g), dt = t / 스텝수\n", flush=True)
print(f"  {'설정 physics_dt':>18}{'보고값':>12}{'실측값':>12}{'낙하거리':>11}{'판정':>7}",
      flush=True)
print("  " + "-" * 60, flush=True)

for hz in (60, 120, 240, 480):
    dt = 1.0 / hz
    reported, measured, drop = measure_dt(dt)
    ok = abs(measured - dt) / dt < 0.05
    print(f"  {f'1/{hz} ({dt:.6f})':>18}{reported:>12.6f}{measured:>12.6f}"
          f"{drop:>10.3f}m{'✅' if ok else '❌':>7}", flush=True)

print("\n  → 원고가 substeps=2 로 노리던 '실효 1/480s' 는 "
      "physics_dt=1/480 을 그대로 주면 된다.", flush=True)

print("\n===SETTINGS_END===", flush=True)
app.close()
