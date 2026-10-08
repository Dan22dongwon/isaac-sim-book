"""H1이 넘어지는 진짜 원인을 관절 단위로 추적한다.

의심: "2족이라 원리적으로 불가능"이 아니라, 관절이 체중을 못 버티는 것.
     t=1.0s 에서 관절 오차가 0.238 rad(13.6°)까지 벌어졌다.
확인할 것:
  ① 관절별 최대 토크(effort) 한계
  ② 어느 관절이 가장 많이 밀리는가
  ③ 게인을 크게 올리면 버티는가
"""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402

root = nucleus_utils.get_assets_root_path()
PHYS_HZ = float(os.environ.get("PHYS_HZ", "200"))
world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYS_HZ, rendering_dt=8.0 / PHYS_HZ)
world.scene.add_default_ground_plane()
stage_utils.add_reference_to_stage(usd_path=root + "/Isaac/Robots/Unitree/H1/h1.usd",
                                   prim_path="/World/R")
world.reset()
r = Robot(prim_path="/World/R", name="r")
world.scene.add(r)
world.reset()
for _ in range(3):
    world.step(render=False)

names = list(r.dof_names)
n = len(names)
ctrl = r.get_articulation_controller()

print(f"\n===TORQUE=== (물리 {PHYS_HZ:.0f}Hz)", flush=True)

# ① 토크 한계 조회
try:
    eff = np.asarray(r.get_max_efforts()).ravel()
    print("관절별 최대 토크(N·m):", flush=True)
    for i in range(0, n, 5):
        print("   " + "  ".join(f"{names[j][:14]}={eff[j]:.0f}" for j in range(i, min(i + 5, n))),
              flush=True)
except Exception as e:
    print(f"토크 한계 조회 실패: {type(e).__name__}: {str(e)[:60]}", flush=True)

POSE = np.zeros(n)   # 전 관절 0 = 곧게 선 자세

for kp, kd in [(500.0, 25.0), (2000.0, 100.0), (8000.0, 400.0), (30000.0, 1500.0)]:
    r.set_world_pose(position=np.array([0.0, 0.0, 1.02]),
                     orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    try:
        r.set_linear_velocity(np.zeros(3))
        r.set_angular_velocity(np.zeros(3))
    except Exception:
        pass
    r.set_joint_positions(POSE)
    r.set_joint_velocities(np.zeros(n))
    ctrl.set_gains(kps=np.ones(n) * kp, kds=np.ones(n) * kd)
    act = ArticulationAction(joint_positions=POSE)
    world.step(render=False)

    worst = None
    for step in range(int(4.0 * PHYS_HZ)):
        r.apply_action(act)
        world.step(render=False)
        if step == int(1.0 * PHYS_HZ):
            e = np.abs(np.asarray(r.get_joint_positions()).ravel() - POSE)
            worst = (names[int(np.argmax(e))], float(e.max()))

    pos, q = r.get_world_pose()
    z = float(np.asarray(pos).ravel()[2])
    q = np.asarray(q, float).ravel()
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1)))
    err = float(np.abs(np.asarray(r.get_joint_positions()).ravel() - POSE).max())
    ok = z > 0.75 and tilt < 20
    print(f"\nkp={kp:>8.0f} kd={kd:>6.0f}  →  z={z:.3f}  기울기={tilt:5.1f}°  "
          f"최종오차={err:.3f}  {'✅ 섬' if ok else '❌'}", flush=True)
    if worst:
        print(f"    t=1.0s 에서 가장 많이 밀린 관절: {worst[0]} ({worst[1]:.3f} rad)", flush=True)

print("===TORQUE_END===", flush=True)
app.close()
