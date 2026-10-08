"""H1 휴머노이드 직립 — 다리 벌림(hip_roll)으로 좌우 지지폭을 넓혀 본다.

관찰: H1은 앞뒤가 아니라 **옆으로(89°)** 넘어진다.
      두 발이 모여 있으면 좌우 지지폭이 거의 0이라 조금만 흔들려도 쓰러진다.
가설: hip_roll 로 다리를 벌리면 지지면이 넓어져 정적으로 설 수 있다.
"""
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402

root = nucleus_utils.get_assets_root_path()
world = World(stage_units_in_meters=1.0)
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


def make(roll, hip_p, knee, ankle):
    """roll > 0 이면 두 다리를 바깥으로 벌린다 (좌우 지지폭 확대)."""
    p = np.zeros(n)
    for i, nm in enumerate(names):
        if nm == "left_hip_roll":
            p[i] = roll
        elif nm == "right_hip_roll":
            p[i] = -roll
        elif nm.endswith("hip_pitch"):
            p[i] = hip_p
        elif nm.endswith("knee"):
            p[i] = knee
        elif nm.endswith("ankle"):
            p[i] = ankle
    return p


# (다리벌림, 엉덩이피치, 무릎, 발목)
COMBOS = [
    (0.00, 0.0, 0.0, 0.0),
    (0.15, 0.0, 0.0, 0.0),
    (0.30, 0.0, 0.0, 0.0),
    (0.42, 0.0, 0.0, 0.0),
    (0.30, -0.25, 0.5, -0.25),
    (0.42, -0.25, 0.5, -0.25),
    (0.42, -0.40, 0.8, -0.40),
]

print("\n===STANCE===", flush=True)
print(f"{'벌림':>6}{'엉덩이':>8}{'무릎':>7}{'발목':>7}{'몸통z':>9}{'기울기':>8}  판정", flush=True)
print("-" * 52, flush=True)

best = None
for roll, hp, kn, an in COMBOS:
    pose = make(roll, hp, kn, an)
    r.get_articulation_controller().set_gains(kps=np.ones(n) * 500.0, kds=np.ones(n) * 25.0)
    act = ArticulationAction(joint_positions=pose)

    r.set_world_pose(position=np.array([0.0, 0.0, 1.02]),
                     orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    try:
        r.set_linear_velocity(np.zeros(3))
        r.set_angular_velocity(np.zeros(3))
    except Exception:
        pass
    r.set_joint_positions(pose)
    r.set_joint_velocities(np.zeros(n))
    world.step(render=False)

    for _ in range(360):
        r.apply_action(act)
        world.step(render=False)

    pos, q = r.get_world_pose()
    z = float(np.asarray(pos).ravel()[2])
    q = np.asarray(q, float).ravel()
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1)))
    ok = z > 0.75 and tilt < 20
    print(f"{roll:>6.2f}{hp:>8.2f}{kn:>7.2f}{an:>7.2f}{z:>9.3f}{tilt:>7.1f}°  "
          f"{'✅ 섬' if ok else '❌'}", flush=True)
    if ok and (best is None or z > best[4]):
        best = (roll, hp, kn, an, z, tilt)

msg = ("✅ 최적: 벌림=%.2f 엉덩이=%.2f 무릎=%.2f 발목=%.2f → z=%.3f (%.1f°)" % best
       if best else "❌ 다리를 벌려도 정적 직립 불가 — 균형 제어 필요")
print(f"\n{msg}", flush=True)
print("===STANCE_END===", flush=True)
app.close()
