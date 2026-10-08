"""4족 로봇의 '서기 자세' 각도를 훑어서 몸통 높이를 실측한다.

앞선 실험에서 관절은 목표에 정확히 도달(오차 0.04 rad)했는데도 몸통이 0.2m로 낮았다.
→ 스폰 높이 문제가 아니라 **자세 값 자체가 웅크린 자세**라는 뜻이다.
여기서는 허벅지/무릎 각도를 바꿔가며 실제 몸통 높이를 잰다.
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

SPECS = {
    "Go2": dict(
        usd="/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd",
        key=lambda n: ("hip" if "hip" in n else "thigh" if "thigh" in n else "calf"),
        abduct="hip", thigh="thigh", knee="calf",
        # (허벅지, 무릎) — 책 값은 (0.8, -1.6)
        combos=[(0.8, -1.6), (0.6, -1.2), (0.45, -0.9), (0.35, -0.7), (0.25, -0.5)],
        kp=60.0, kd=3.0, spawn=0.45,
    ),
    "Spot": dict(
        usd="/Isaac/Robots/BostonDynamics/spot/spot.usd",
        key=lambda n: n.split("_")[-1],
        abduct="hx", thigh="hy", knee="kn",
        combos=[(0.9, -1.8), (0.7, -1.4), (0.55, -1.1), (0.45, -0.9), (0.30, -0.6)],
        kp=150.0, kd=8.0, spawn=0.70,
    ),
}
s = SPECS[ROBOT]

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage_utils.add_reference_to_stage(usd_path=root + s["usd"], prim_path="/World/R")
world.reset()
r = Robot(prim_path="/World/R", name="r")
world.scene.add(r)
world.reset()

names = list(r.dof_names)
n = len(names)
r.get_articulation_controller().set_gains(kps=np.ones(n) * s["kp"], kds=np.ones(n) * s["kd"])

print(f"\n===SWEEP_{ROBOT}===", flush=True)
print(f"관절 {n}개: {names[:4]} ...", flush=True)
print(f"{'허벅지':>8}{'무릎':>8}{'몸통 높이':>12}{'관절오차':>10}  판정", flush=True)
print("-" * 50, flush=True)

results = []
for thigh, knee in s["combos"]:
    target = {s["abduct"]: 0.0, s["thigh"]: thigh, s["knee"]: knee}
    pose = np.array([target.get(s["key"](nm), 0.0) for nm in names])

    # ⚠️ 위치만 되돌리면 안 된다. 한 번 옆으로 누우면 그 자세 그대로 들어올려져
    #    똑같이 다시 넘어진다. 자세(orientation)와 몸통 속도까지 초기화해야 한다.
    r.set_world_pose(position=np.array([0.0, 0.0, s["spawn"]]),
                     orientation=np.array([1.0, 0.0, 0.0, 0.0]))   # 단위 쿼터니언
    try:
        r.set_linear_velocity(np.zeros(3))
        r.set_angular_velocity(np.zeros(3))
    except Exception:
        pass
    r.set_joint_positions(pose)
    r.set_joint_velocities(np.zeros(n))
    world.step(render=False)          # 초기화 반영
    action = ArticulationAction(joint_positions=pose)
    for _ in range(200):
        r.apply_action(action)
        world.step(render=False)

    z = float(np.asarray(r.get_world_pose()[0]).ravel()[2])
    err = float(np.abs(np.asarray(r.get_joint_positions()) - pose).max())
    tag = "책 값" if (thigh, knee) == s["combos"][0] else ""
    stand = err < 0.2 and z > 0.30
    print(f"{thigh:>8.2f}{knee:>8.2f}{z:>12.3f}{err:>10.3f}  "
          f"{'✅ 섬' if stand else '❌ 낮음'} {tag}", flush=True)
    results.append((thigh, knee, z, err))

best = max(results, key=lambda t: t[2])
print(f"\n가장 높이 선 조합: 허벅지 {best[0]:.2f} / 무릎 {best[1]:.2f} → 몸통 z = {best[2]:.3f} m",
      flush=True)
print(f"책 값 대비 높이 차이: {best[2] - results[0][2]:+.3f} m", flush=True)
print(f"===SWEEP_{ROBOT}_END===", flush=True)
app.close()
