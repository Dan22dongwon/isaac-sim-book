"""책 5.4~5.6 — 로봇 4종을 GUI로 띄워 눈으로 확인한다.

핵심: world.reset() 이후에는 물리 엔진이 포즈를 관장한다.
      USD xform(SetTranslate)이 아니라 Robot.set_world_pose() 로 배치해야 한다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python robot_gallery_gui.py
"""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": False})

import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

# H1 은 200Hz 에서 검증했다. 60Hz 에서는 발목이 더 쉽게 밀린다.
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=8.0 / 200.0)
world.scene.add_default_ground_plane()
nucleus = nucleus_utils.get_assets_root_path()
stage = omni.usd.get_context().get_stage()

# [ERR-022] 경로 교정 / [ERR-029] Go2는 physics 레이어를 써야 관절이 있다
ROBOTS = [
    # 스폰 높이는 '서 있을 때의 몸통 높이'에 맞춘다.
    # 너무 높으면 떨어지며 착지 충격으로 자세가 무너진다.
    ("NovaCarter", "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd", -3.6, 0.30),
    ("Go2", "/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd", -1.2, 0.45),
    ("Spot", "/Isaac/Robots/BostonDynamics/spot/spot.usd", 1.2, 0.50),
    ("H1", "/Isaac/Robots/Unitree/H1/h1.usd", 3.8, 1.02)   # 실측 성공 높이,
]

for name, rel, x, z in ROBOTS:
    stage_utils.add_reference_to_stage(usd_path=nucleus + rel, prim_path=f"/World/{name}")

# 조명
key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(15000.0)
key.CreateWidthAttr().Set(16.0)
key.CreateHeightAttr().Set(10.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.0, -2.0, 7.0))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(700.0)

world.reset()

# ── 로봇 객체로 감싸고, 물리 기준 포즈로 배치 ──────────────────────
robots = {}
for name, rel, x, z in ROBOTS:
    try:
        r = Robot(prim_path=f"/World/{name}", name=name.lower())
        world.scene.add(r)
        robots[name] = (r, np.array([x, 0.0, z]))
    except Exception as e:
        print(f"  {name}: {type(e).__name__}: {str(e)[:70]}", flush=True)

world.reset()

# 서기 자세 — [ERR-030] 배열 순서를 믿지 말고 관절 이름으로 매핑한다.
# 각도는 실측으로 정했다 (stand_sweep.py). 책 값(허벅지 0.8 / 무릎 -1.6)은
# 넘어지지는 않지만 웅크린 자세이고 관절 오차가 크다.
#   Go2  : 책 0.293m  →  (0.35, -0.70) 에서 0.410m
#   Spot : 책 0.311m  →  (0.30, -0.60) 에서 0.431m
GO2_TARGET = {
    "FL_hip_joint": 0.0, "FL_thigh_joint": 0.35, "FL_calf_joint": -0.70,
    "FR_hip_joint": 0.0, "FR_thigh_joint": 0.35, "FR_calf_joint": -0.70,
    "RL_hip_joint": 0.0, "RL_thigh_joint": 0.35, "RL_calf_joint": -0.70,
    "RR_hip_joint": 0.0, "RR_thigh_joint": 0.35, "RR_calf_joint": -0.70,
}
SPOT_TARGET = {"hx": 0.0, "hy": 0.30, "kn": -0.60}
H1_TARGET = {}   # 전 관절 0 = 곧게 선 자세 (실측 성공 조건)


def pose_for(r, kind):
    """서기 자세를 돌려준다.

    ★ 중요 — 로봇마다 출처가 다르다.
      Spot : 에셋에 저장된 **기본 관절값이 이미 서기 자세**다.
             hx 가 좌우 반대 부호(+0.105 / -0.103)로 다리를 벌리고,
             앞다리(hy 0.955)와 뒷다리(hy 1.136)의 각도가 다르다.
             이 규약을 무시하고 0으로 덮어쓰면 다리가 몸통 아래로 모여 쓰러진다.
      Go2  : physics USD의 기본값은 다리를 편 '누운 자세'라 그대로 쓰면 뒤집힌다.
             실측으로 찾은 (허벅지 0.35 / 무릎 -0.70)을 쓴다.
    """
    names = list(r.dof_names)
    if kind == "Spot":
        # 에셋 기본값을 그대로 쓴다 (로드 직후 값)
        return np.asarray(r.get_joint_positions()).ravel().copy()
    if kind == "Go2":
        return np.array([GO2_TARGET.get(n, 0.0) for n in names])
    if kind == "H1":
        out = []
        for n in names:
            v = 0.0
            for k, val in H1_TARGET.items():
                if n.endswith(k):
                    v = val
            out.append(v)
        return np.array(out)
    return np.zeros(len(names))


# ★ 필요한 관절 강성(kp)은 로봇마다 크게 다르다 — 실측으로 정했다.
#   Spot(약 32kg) : kp 120  으로 충분
#   H1  (약 47kg) : kp 8000 필요. 500/2000 에서는 **발목이 밀려** 넘어진다.
#                   책의 kp=80 은 H1 기준 100배 부족하다.
#   게인이 모자라면 에러 없이 로봇만 서서히 무너진다.
GAINS = {"Go2": (60.0, 3.0), "Spot": (120.0, 4.0), "H1": (8000.0, 400.0)}

actions = {}
for name, (r, pos) in robots.items():
    n = len(r.dof_names)
    if n:
        kp, kd = GAINS.get(name, (120.0, 4.0))
        r.get_articulation_controller().set_gains(
            kps=np.ones(n) * kp, kds=np.ones(n) * kd
        )
        actions[name] = ArticulationAction(joint_positions=pose_for(r, name))
    print(f"  {name:<12} DOF {n:>3}  배치 {pos.tolist()}", flush=True)

set_camera_view(eye=np.array([0.5, -8.5, 3.2]), target=np.array([0.5, 0.0, 0.6]))

print("\n✅ 로봇 4종 배치 완료 — GUI 창을 확인하세요", flush=True)
print("===GUI_READY===", flush=True)

# ── 초기 상태 만들기 — 5가지를 **모두** 해야 한다 ─────────────────
#  ① 몸통 위치   ② 몸통 자세(안 하면 누운 채로 시작)
#  ③ 몸통 속도   ④ 관절 각도(목표가 아니라 '상태')  ⑤ 관절 속도
#
#  관절 '목표'(apply_action)만 주면 로봇은 기본 자세로 떨어진 뒤
#  착지 충격에 무너지고, 넘어진 다음에는 목표만으로 일어나지 못한다.
#  그리고 이 배치는 **한 번만** 한다 — 주기적으로 부르면 통통 튄다.
# 로봇 기본 방향은 +X 이고 카메라는 -Y 쪽에 있다.
# 카메라를 바라보게 하려면 Z축으로 -90도 돌린다.
#   쿼터니언 (w, x, y, z) = (cos(-45°), 0, 0, sin(-45°))
import math  # noqa: E402

_FACE_CAM = np.array([math.cos(math.radians(-45.0)), 0.0, 0.0,
                      math.sin(math.radians(-45.0))])

for name, (r, pos) in robots.items():
    r.set_world_pose(position=pos, orientation=_FACE_CAM)
    try:
        r.set_linear_velocity(np.zeros(3))
        r.set_angular_velocity(np.zeros(3))
    except Exception:
        pass
    nd = len(r.dof_names)
    if nd:
        r.set_joint_positions(pose_for(r, name))
        r.set_joint_velocities(np.zeros(nd))
world.step(render=True)

hold = float(os.environ.get("HOLD_SECONDS", "600"))
import time  # noqa: E402

t_end = time.time() + hold
while time.time() < t_end:
    # 관절 목표는 반대로 **매 스텝** 유지해야 한다.
    # 한 번만 주면 드라이브가 목표를 놓치고 관절이 늘어진다.
    for name, a in actions.items():
        robots[name][0].apply_action(a)
    world.step(render=True)

app.close()
