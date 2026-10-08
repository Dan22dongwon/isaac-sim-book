"""책 7.4 [실습] 외력 인가 — 순간 충격 vs 지속 압력. 동작 검증본.

원문과의 차이:
  [ERR-043] 원문이 쓰는 API 는 **Isaac Sim 에 없다.**
      robot.instantaneous_wrench_composer.add_forces_and_torques(...)
      robot.permanent_wrench_composer.set_forces_and_torques(...)
      robot.write_data_to_sim()
    이 셋은 **Isaac Lab** 의 API 다 (그래서 torch 를 쓴다).
    Isaac Sim 에서는 RigidPrim 의 메서드를 쓴다.
      RigidPrim.apply_forces(forces, indices, is_global)
      RigidPrim.apply_forces_and_torques_at_pos(forces, torques, positions, ...)

  '순간'과 '지속'의 구분은 **전용 API 가 아니라 호출 시점**으로 만든다.
      순간 충격 — 한 스텝에만 호출한다
      지속 압력 — 매 스텝 호출한다
    PhysX 의 외력은 원래 한 스텝만 유효하고 자동으로 사라지기 때문이다.
    원문의 "permanent 는 0으로 재설정하기 전까지 계속 적용된다" 는
    Isaac Lab 의 래퍼가 매 스텝 다시 써 주기 때문이지, PhysX 의 성질이 아니다.

검증 방법
  질량을 아는 상자에 힘을 주고 **뉴턴 역학으로 검산한다.**
      Δv = F·Δt / m        (운동량 변화)
  이론값과 실측값이 맞으면 API 가 제대로 동작한다는 뜻이다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python external_force.py
"""
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import RigidPrim  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

import omni.usd  # noqa: E402

DT = 1.0 / 120
MASS = 5.0
FORCE = 40.0        # N, +X 방향

World.clear_instance()
world = World(stage_units_in_meters=1.0, physics_dt=DT)
stage = omni.usd.get_context().get_stage()
# 바닥과 마찰을 빼야 F=ma 를 깨끗하게 검산할 수 있다.
# 중력도 빼면 순수하게 외력의 효과만 남는다.
scene = [p for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene"][0]
UsdPhysics.Scene(scene).CreateGravityMagnitudeAttr().Set(0.0)


def make_box(path, x):
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(0.2)
    UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 1.0))
    UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr().Set(MASS)
    return path


make_box("/World/BoxImpulse", -1.0)      # 순간 충격용
make_box("/World/BoxPermanent", 1.0)     # 지속 압력용
world.reset()

imp = RigidPrim(prim_paths_expr="/World/BoxImpulse", name="imp")
per = RigidPrim(prim_paths_expr="/World/BoxPermanent", name="per")
imp.initialize()
per.initialize()
for v in (imp, per):
    v.set_velocities(np.zeros((1, 6)))
world.step(render=False)

F = np.array([[FORCE, 0.0, 0.0]])


def vx(view):
    return float(np.asarray(view.get_velocities())[0][0])


def px(view):
    return float(np.asarray(view.get_world_poses()[0])[0][0])


print("\n===FORCE===", flush=True)
print(f"상자 {MASS:.0f}kg, 외력 {FORCE:.0f}N(+X), 타임스텝 1/{1/DT:.0f}s, 중력 0", flush=True)
print("\n[ERR-043] 원문의 wrench_composer 는 Isaac Lab API 다. "
      "Isaac Sim 은 RigidPrim 을 쓴다.\n", flush=True)

STEPS = 120
x0_i, x0_p = px(imp), px(per)

for step in range(STEPS):
    # ── 순간 충격: **한 스텝에만** 힘을 준다 ──
    if step == 0:
        imp.apply_forces(F, is_global=True)
    # ── 지속 압력: **매 스텝** 힘을 준다 ──
    per.apply_forces(F, is_global=True)
    world.step(render=False)

    if step in (0, 1, 30, 119):
        print(f"  step {step:>4}   순간 v={vx(imp):+.4f} m/s   "
              f"지속 v={vx(per):+.4f} m/s", flush=True)

v_i, v_p = vx(imp), vx(per)
d_i, d_p = px(imp) - x0_i, px(per) - x0_p

# ── 뉴턴 역학으로 검산 ────────────────────────────────────────────
v_i_theory = FORCE * DT / MASS                       # 한 스텝만 가속
v_p_theory = FORCE * (STEPS * DT) / MASS             # 계속 가속
d_i_theory = v_i_theory * (STEPS * DT)               # 등속 이동
d_p_theory = 0.5 * (FORCE / MASS) * (STEPS * DT) ** 2  # 등가속 이동

print(f"\n{'':<14}{'실측 v':>12}{'이론 v':>12}{'실측 거리':>12}{'이론 거리':>12}",
      flush=True)
print("-" * 62, flush=True)
print(f"{'순간 충격':<14}{v_i:>11.4f}m/s{v_i_theory:>11.4f}m/s"
      f"{d_i:>11.4f}m{d_i_theory:>11.4f}m", flush=True)
print(f"{'지속 압력':<14}{v_p:>11.4f}m/s{v_p_theory:>11.4f}m/s"
      f"{d_p:>11.4f}m{d_p_theory:>11.4f}m", flush=True)

ok_i = abs(v_i - v_i_theory) < 1e-4
ok_p = abs(v_p - v_p_theory) / v_p_theory < 0.02
print(f"\n  순간 충격  Δv = F·Δt/m = {FORCE}×{DT:.5f}/{MASS} = "
      f"{v_i_theory:.4f} m/s   {'✅ 일치' if ok_i else '❌ 불일치'}", flush=True)
print(f"  지속 압력  Δv = F·t/m  = {FORCE}×{STEPS*DT:.3f}/{MASS} = "
      f"{v_p_theory:.4f} m/s   {'✅ 일치' if ok_p else '❌ 불일치'}", flush=True)
print(f"\n  속도 비율 {v_p/v_i:.0f}배 — {STEPS}스텝 내내 밀었으니 "
      f"{STEPS}배가 맞다.", flush=True)
print(f"  순간 충격은 한 번 밀린 뒤 **등속**으로 간다 (중력·마찰이 없으므로).",
      flush=True)
print(f"  지속 압력은 계속 **가속**한다.", flush=True)

print(f"\n  판정: {'✅ 외력 API 가 뉴턴 역학대로 동작한다' if ok_i and ok_p else '❌ 불일치'}",
      flush=True)
print("===FORCE_END===", flush=True)
app.close()
