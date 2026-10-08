"""로봇의 관절 한계·기본값을 조회한다. 서기 자세를 정하려면 부호 규약을 알아야 한다."""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402

root = nucleus_utils.get_assets_root_path()
ROBOT = os.environ.get("ROBOT", "Spot")
USD = {
    "Spot": "/Isaac/Robots/BostonDynamics/spot/spot.usd",
    "H1": "/Isaac/Robots/Unitree/H1/h1.usd",
    "Go2": "/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd",
}[ROBOT]

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage_utils.add_reference_to_stage(usd_path=root + USD, prim_path="/World/R")
world.reset()
r = Robot(prim_path="/World/R", name="r")
world.scene.add(r)
world.reset()
for _ in range(3):
    world.step(render=False)

names = list(r.dof_names)
lim = np.asarray(r.get_articulation_controller().get_joint_limits()
                 if hasattr(r.get_articulation_controller(), "get_joint_limits")
                 else r.get_dof_limits())
cur = np.asarray(r.get_joint_positions()).ravel()

print(f"\n===LIMITS_{ROBOT}===", flush=True)
print(f"{'#':>3} {'관절':<26}{'최소':>9}{'최대':>9}{'현재':>9}", flush=True)
print("-" * 58, flush=True)
lo_hi = lim.reshape(-1, 2) if lim.ndim <= 2 else lim[0]
for i, nm in enumerate(names):
    lo, hi = float(lo_hi[i][0]), float(lo_hi[i][1])
    print(f"{i:>3} {nm:<26}{lo:>9.3f}{hi:>9.3f}{cur[i]:>9.3f}", flush=True)
print(f"===LIMITS_{ROBOT}_END===", flush=True)
app.close()
