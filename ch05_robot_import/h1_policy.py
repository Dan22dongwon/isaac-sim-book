"""H1 휴머노이드를 **학습된 정책**으로 세운다.

앞선 실험 결론:
  H1은 자세 목표만으로는 설 수 없다. 처음 1초는 서 있다가(기울기 0.1°)
  서서히 무너져 2초에 89°로 넘어진다 — 역진자 불안정성이다.
  4족(Spot)은 같은 방식으로 6초 내내 0.5° 안정.

해법:
  Isaac Sim 6.1에 NVIDIA가 학습시킨 보행 정책이 번들로 들어 있다.
      from isaacsim.robot.policy.examples import RobotPolicyRunner, get_h1_spec
  이 정책이 매 스텝 균형을 보정한다.
"""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "1") == "1"})

import omni.kit.app  # noqa: E402

mgr = omni.kit.app.get_app().get_extension_manager()
mgr.set_extension_enabled_immediate("isaacsim.robot.policy.examples", True)

from isaacsim.core.api import World  # noqa: E402
from isaacsim.robot.policy.examples import (  # noqa: E402
    RobotPolicyRunner,
    get_go2_spec,
    get_h1_spec,
    get_spot_spec,
)

SPECS = {"H1": get_h1_spec, "Spot": get_spot_spec, "Go2": get_go2_spec}
WHICH = os.environ.get("ROBOT", "H1")

# ★ 정책은 학습 당시의 물리 주기를 전제한다.
#   공식 H1 예제 기준: 물리 200Hz, 렌더 25Hz (물리 8스텝당 1렌더)
#   Isaac Sim 기본 World 는 60Hz 라 그대로 쓰면 정책이 과도하게 움직여 넘어진다.
PHYS_HZ = 200.0
world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYS_HZ,
              rendering_dt=8.0 / PHYS_HZ)
world.scene.add_default_ground_plane()
world.reset()

print(f"\n===POLICY_{WHICH}===", flush=True)
runner = RobotPolicyRunner(
    SPECS[WHICH](),
    prim_path=f"/World/{WHICH}",
    position=np.array([0.0, 0.0, 1.05 if WHICH == "H1" else 0.7]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
)
# 순서가 중요하다: spawn(재생 전 로봇 저작) → reset(물리 시작) → initialize(정책 준비)
runner.spawn()
world.reset()
runner.initialize()

dt = world.get_physics_context().get_physics_dt()
command = np.zeros(3)          # [전진, 횡이동, 회전] — 0 이면 제자리 서기

print(f"physics_dt = {dt:.5f} s,  command = {command.tolist()} (제자리 서기)", flush=True)
print(f"{'step':>6}{'t(s)':>8}{'몸통z':>9}{'기울기':>9}", flush=True)
print("-" * 34, flush=True)

for step in range(2001):
    runner.step(dt, command)
    world.step(render=False)
    if step % 250 == 0:
        pos, quat = runner.articulation.get_world_poses()
        z = float(np.asarray(pos).ravel()[2])
        q = np.asarray(quat, float).ravel()
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1)))
        print(f"{step:>6}{step * dt:>8.2f}{z:>9.3f}{tilt:>8.1f}°", flush=True)

pos, quat = runner.articulation.get_world_poses()
z = float(np.asarray(pos).ravel()[2])
q = np.asarray(quat, float).ravel()
tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1)))
print(f"\n{'✅ 10초간 서 있음' if (z > 0.7 if WHICH == 'H1' else z > 0.35) and tilt < 20 else '❌ 넘어짐'}"
      f"  (최종 z={z:.3f}, 기울기={tilt:.1f}°)", flush=True)
print(f"===POLICY_{WHICH}_END===", flush=True)
app.close()
