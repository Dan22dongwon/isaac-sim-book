"""에셋의 '기본 관절값'을 서기 자세로 쓰고, 스폰 높이만 훑는다.

앞선 실험에서 자세를 직접 지어냈다가 실패했다.
관절 한계를 조회해 보니 로봇 에셋은 이미 서기 자세로 저장돼 있었다.
  Spot : hx 좌우 반대 부호(±0.10), hy 0.96~1.14, kn -1.47
  H1   : 전 관절 ≈ 0 (직립)
따라서 자세는 로드 직후 값을 그대로 쓰고, 스폰 높이만 맞추면 된다.
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
ROBOT = os.environ.get("ROBOT", "Spot")
SPEC = {
    "Spot": dict(usd="/Isaac/Robots/BostonDynamics/spot/spot.usd",
                 heights=[0.50, 0.58, 0.62, 0.66, 0.72], kp=150.0, kd=8.0, ok_z=0.40),
    "H1": dict(usd="/Isaac/Robots/Unitree/H1/h1.usd",
               heights=[0.95, 1.00, 1.05, 1.10, 1.15], kp=250.0, kd=12.0, ok_z=0.85),
    "Go2": dict(usd="/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd",
                heights=[0.30, 0.35, 0.40, 0.45], kp=60.0, kd=3.0, ok_z=0.25),
}[ROBOT]

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage_utils.add_reference_to_stage(usd_path=root + SPEC["usd"], prim_path="/World/R")
world.reset()
r = Robot(prim_path="/World/R", name="r")
world.scene.add(r)
world.reset()
for _ in range(3):
    world.step(render=False)

names = list(r.dof_names)
n = len(names)
# ★ 에셋에 저장된 기본 관절값 = 제작자가 의도한 서기 자세
DEFAULT_POSE = np.asarray(r.get_joint_positions()).ravel().copy()
r.get_articulation_controller().set_gains(kps=np.ones(n) * SPEC["kp"], kds=np.ones(n) * SPEC["kd"])
action = ArticulationAction(joint_positions=DEFAULT_POSE)

print(f"\n===DEFAULT_{ROBOT}===", flush=True)
print(f"기본 관절값(서기 자세로 사용): {np.round(DEFAULT_POSE[:6], 3).tolist()} ...", flush=True)
print(f"{'스폰 높이':>10}{'최종 몸통 z':>13}{'기울기':>9}{'관절오차':>10}  판정", flush=True)
print("-" * 56, flush=True)

best = None
for h in SPEC["heights"]:
    r.set_world_pose(position=np.array([0.0, 0.0, h]),
                     orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    try:
        r.set_linear_velocity(np.zeros(3))
        r.set_angular_velocity(np.zeros(3))
    except Exception:
        pass
    r.set_joint_positions(DEFAULT_POSE)
    r.set_joint_velocities(np.zeros(n))
    world.step(render=False)

    for _ in range(240):
        r.apply_action(action)
        world.step(render=False)

    pos, quat = r.get_world_pose()
    z = float(np.asarray(pos).ravel()[2])
    q = np.asarray(quat, dtype=float).ravel()
    # (w,x,y,z) 가정 — 몸통 로컬 +Z 가 월드 +Z 와 이루는 각
    up = 1 - 2 * (q[1] ** 2 + q[2] ** 2)
    tilt = np.degrees(np.arccos(np.clip(up, -1, 1)))
    err = float(np.abs(np.asarray(r.get_joint_positions()).ravel() - DEFAULT_POSE).max())
    stand = z >= SPEC["ok_z"] and tilt < 25.0
    print(f"{h:>10.2f}{z:>13.3f}{tilt:>9.1f}°{err:>10.3f}  {'✅ 섬' if stand else '❌'}", flush=True)
    if stand and (best is None or abs(z - h) < abs(best[1] - best[0])):
        best = (h, z, tilt)

print(f"\n{'✅ 최적 스폰 높이 = %.2f m (몸통 z=%.3f, 기울기 %.1f°)' % best if best else '❌ 이 조건으로는 서지 못함'}",
      flush=True)
print(f"===DEFAULT_{ROBOT}_END===", flush=True)
app.close()
