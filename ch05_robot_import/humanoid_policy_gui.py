"""H1 휴머노이드를 **학습된 정책**으로 세우고 걷게 한다 (GUI).

왜 정책이 필요한가:
  4족(Spot)은 관절 자세 목표만으로 정적으로 선다.
  2족(H1)은 지지면이 좁아 **원리적으로 불가능**하다. 실측하면
  처음 1초는 기울기 0.1°로 서 있다가 서서히 무너져 2초에 89°로 넘어진다.
  매 순간 균형을 보정하는 제어가 있어야 하고, 그것이 학습된 보행 정책이다.

Isaac Sim 6.1에는 NVIDIA가 학습시킨 정책이 번들로 들어 있다.
  H1 · Spot · Go2 · ANYmal · Franka · Cartpole

⚠️ 가장 흔한 함정 — 물리 주기
  정책은 학습 당시의 물리 주기를 전제한다. H1 정책은 **200Hz** 기준이다.
  Isaac Sim 기본 World 는 60Hz 라 그대로 쓰면 정책이 3.3배 과하게 움직여
  **에러 없이 로봇만 넘어진다.**

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python humanoid_policy_gui.py
"""
import os
import time

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "0") == "1"})

import omni.kit.app  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot.policy.examples", True
)

import numpy as np  # noqa: E402,F811
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.robot.policy.examples import (  # noqa: E402
    RobotPolicyRunner,
    get_go2_spec,
    get_h1_spec,
    get_spot_spec,
)
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

# ★ 정책 학습 시의 물리 주기를 맞춘다 (공식 H1 예제 기준)
PHYS_HZ = 200.0
world = World(stage_units_in_meters=1.0,
              physics_dt=1.0 / PHYS_HZ,
              rendering_dt=8.0 / PHYS_HZ)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(15000.0)
key.CreateWidthAttr().Set(14.0)
key.CreateHeightAttr().Set(10.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.0, -2.0, 7.0))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(700.0)
world.reset()

# 3종을 나란히 — 전부 학습된 정책으로 구동한다
ROBOTS = [
    ("Go2", get_go2_spec, (-2.2, 0.0, 0.45)),
    ("Spot", get_spot_spec, (0.0, 0.0, 0.70)),
    ("H1", get_h1_spec, (2.4, 0.0, 1.05)),
]

runners = {}
for name, spec_fn, pos in ROBOTS:
    try:
        r = RobotPolicyRunner(
            spec_fn(),
            prim_path=f"/World/{name}",
            position=np.array(pos),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        # 순서가 중요하다: spawn(재생 전 저작) → reset(물리 시작) → initialize(정책 준비)
        r.spawn()
        runners[name] = r
        print(f"  {name:<6} spawn 완료  위치 {pos}", flush=True)
    except Exception as e:
        print(f"  {name:<6} ❌ {type(e).__name__}: {str(e)[:70]}", flush=True)

world.reset()
for name, r in runners.items():
    r.initialize()
    print(f"  {name:<6} 정책 초기화  (학습 물리주기 {1 / r.physics_dt:.0f}Hz, "
          f"decimation {r.decimation})", flush=True)

set_camera_view(eye=np.array([0.2, -6.0, 2.4]), target=np.array([0.2, 0.0, 0.7]))

dt = world.get_physics_context().get_physics_dt()
print(f"\n물리 주기 {1 / dt:.0f} Hz", flush=True)
print("✅ 준비 완료 — 3초 제자리 서기 → 전진 보행", flush=True)
print("===GUI_READY===", flush=True)

hold = float(os.environ.get("HOLD_SECONDS", "900"))
t_end = time.time() + hold
step = 0
STAND_STEPS = int(3.0 / dt)          # 처음 3초는 제자리

while time.time() < t_end:
    # command = [전진(m/s), 횡이동(m/s), 회전(rad/s)]
    cmd = np.zeros(3) if step < STAND_STEPS else np.array([0.6, 0.0, 0.0])
    for r in runners.values():
        r.step(dt, cmd)
    world.step(render=True)
    step += 1

app.close()
