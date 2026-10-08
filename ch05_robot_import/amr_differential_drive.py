"""책 5.4 차동 구동 AMR 제어 — 동작 검증본.

원문과의 차이:
  [ERR-022] carter_v1.usd 경로가 404. NVIDIA/ 하위로 옮겨졌다.
  [ERR-028] 신규 import 경로에 구형 인자를 넘긴다.
            prim_path→paths(위치인자), name/create_robot 없음, position→positions

원문 명령: command = [0.5, 0.3]  → 선속도 0.5 m/s, 각속도 0.3 rad/s (곡선 주행)
이론상 반지름 r = v/ω = 0.5/0.3 ≈ 1.67 m 의 원을 그린다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python amr_differential_drive.py
"""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "1") == "1"})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.controllers import (  # noqa: E402
    DifferentialController,
)
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
nucleus = nucleus_utils.get_assets_root_path()

# [ERR-022] 원문 "/Isaac/Robots/Carter/carter_v1.usd" 는 404
carter_usd = nucleus + "/Isaac/Robots/NVIDIA/Carter/carter_v1.usd"

# [ERR-028] 신규 API 시그니처
carter = WheeledRobot(
    "/World/Robots/Carter",
    wheel_dof_names=["left_wheel", "right_wheel"],
    usd_path=carter_usd,
    positions=np.array([[0.0, 0.0, 0.1]]),
)
world.reset()

# [ERR-028] 신규 DifferentialController 는 name 인자가 없고 전부 키워드 전용
controller = DifferentialController(wheel_radius=0.125, wheel_base=0.46)

V, W = 0.5, 0.3          # 선속도 m/s, 각속도 rad/s
dt = world.get_physics_context().get_physics_dt()

print("\n===AMR===", flush=True)
print(f"관절 {list(carter.dof_names)}", flush=True)
print(f"명령 [{V}, {W}]  →  이론 회전반경 r = v/ω = {V / W:.2f} m", flush=True)
print(f"{'step':>6}{'t(s)':>7}{'x':>8}{'y':>8}{'방위(°)':>9}", flush=True)

start = np.asarray(carter.get_world_poses()[0], dtype=float).ravel().copy()
path = []

for step in range(500):
    action = controller.forward(command=np.array([V, W]))
    carter.apply_wheel_actions(action)
    world.step(render=False)

    pos, quat = carter.get_world_poses()
    p = np.asarray(pos, float).ravel()
    q = np.asarray(quat, float).ravel()
    yaw = np.degrees(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                1 - 2 * (q[2] ** 2 + q[3] ** 2)))
    path.append(p[:2].copy())
    if step % 100 == 0:
        print(f"{step:>6}{step * dt:>7.2f}{p[0]:>8.3f}{p[1]:>8.3f}{yaw:>9.1f}", flush=True)

path = np.array(path)
end = path[-1]
t_total = 500 * dt

# 원 궤적이면 중심에서의 거리가 일정하다 — 반경을 추정해 이론값과 비교
mid = path.mean(axis=0)
radii = np.linalg.norm(path - mid, axis=1)
turned = W * t_total

print(f"\n  주행 시간     : {t_total:.2f} s", flush=True)
print(f"  이동 거리(직선): {np.linalg.norm(end - start[:2]):.3f} m", flush=True)
print(f"  경로 길이     : {np.linalg.norm(np.diff(path, axis=0), axis=1).sum():.3f} m "
      f"(이론 v·t = {V * t_total:.3f} m)", flush=True)
print(f"  회전량        : {np.degrees(turned):.1f}° (이론 ω·t)", flush=True)
print(f"  추정 회전반경  : {radii.mean():.2f} m ± {radii.std():.2f} "
      f"(이론 {V / W:.2f} m)", flush=True)
print("===AMR_END===", flush=True)
app.close()
