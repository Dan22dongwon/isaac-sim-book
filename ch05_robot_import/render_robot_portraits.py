"""서 있는 로봇을 로봇별로 깨끗하게 렌더링한다 (책 삽입용).

GUI 창 캡처는 다른 창이 겹치면 그대로 찍히고 프레이밍도 제각각이다.
헤드리스 렌더는 창 간섭이 없고, 로봇마다 카메라를 맞춰 일정한 구도를 얻는다.

산출물
    ch05_portrait_<robot>.png   로봇별 정면 초상
    ch05_portrait_group.png     4종 단체

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python render_robot_portraits.py
"""
import math
import os
import struct
import zlib

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "./out")
os.makedirs(OUT, exist_ok=True)
root = nucleus_utils.get_assets_root_path()

# 정책 학습 주기와 동일하게 — H1 은 200Hz 에서만 안정적으로 선다
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=8.0 / 200.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

# ── 로봇 정의 ─────────────────────────────────────────────────────
#   [ERR-022] 경로 교정 / [ERR-029] Go2 는 physics 레이어
#   kp 는 실측값 — 로봇 질량에 따라 100배까지 차이 난다
ROBOTS = [
    dict(name="NovaCarter", usd="/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
         x=-3.6, z=0.30, kp=120.0, kd=4.0, cam_d=2.0, cam_h=0.8, look=0.25),
    dict(name="Go2", usd="/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd",
         x=-1.2, z=0.45, kp=60.0, kd=3.0, cam_d=1.8, cam_h=0.7, look=0.28),
    dict(name="Spot", usd="/Isaac/Robots/BostonDynamics/spot/spot.usd",
         x=1.2, z=0.50, kp=120.0, kd=4.0, cam_d=2.4, cam_h=0.9, look=0.40),
    dict(name="H1", usd="/Isaac/Robots/Unitree/H1/h1.usd",
         x=3.8, z=1.02, kp=8000.0, kd=400.0, cam_d=3.6, cam_h=1.4, look=0.85),
]

GO2_POSE = {"thigh": 0.35, "calf": -0.70}

for r in ROBOTS:
    stage_utils.add_reference_to_stage(usd_path=root + r["usd"], prim_path=f"/World/{r['name']}")

# 조명
key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(18000.0)
key.CreateWidthAttr().Set(16.0)
key.CreateHeightAttr().Set(12.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.0, -3.0, 7.0))
fill = UsdLux.RectLight.Define(stage, "/World/Lights/Fill")
fill.CreateIntensityAttr().Set(6000.0)
fill.CreateWidthAttr().Set(10.0)
fill.CreateHeightAttr().Set(8.0)
UsdGeom.XformCommonAPI(fill.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 4.0, 4.0))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(900.0)

world.reset()

# 로봇 객체
for r in ROBOTS:
    r["obj"] = Robot(prim_path=f"/World/{r['name']}", name=r["name"].lower())
    world.scene.add(r["obj"])
world.reset()


def stand_pose(r):
    """로봇별 서기 자세. 출처가 다르다는 점이 핵심이다."""
    names = list(r["obj"].dof_names)
    if r["name"] == "Spot":
        return np.asarray(r["obj"].get_joint_positions()).ravel().copy()   # 에셋 기본값
    if r["name"] == "Go2":
        return np.array([GO2_POSE.get("thigh" if "thigh" in n else
                                      "calf" if "calf" in n else "", 0.0) for n in names])
    return np.zeros(len(names))     # H1: 전 관절 0 = 직립 / Carter: 바퀴


# 카메라를 향하도록 Z축 -90도
FACE = np.array([math.cos(math.radians(-45.0)), 0.0, 0.0, math.sin(math.radians(-45.0))])

actions = {}
for r in ROBOTS:
    o = r["obj"]
    n = len(o.dof_names)
    pose = stand_pose(r)
    if n:
        o.get_articulation_controller().set_gains(kps=np.ones(n) * r["kp"],
                                                  kds=np.ones(n) * r["kd"])
        actions[r["name"]] = ArticulationAction(joint_positions=pose)
    o.set_world_pose(position=np.array([r["x"], 0.0, r["z"]]), orientation=FACE)
    try:
        o.set_linear_velocity(np.zeros(3))
        o.set_angular_velocity(np.zeros(3))
    except Exception:
        pass
    if n:
        o.set_joint_positions(pose)
        o.set_joint_velocities(np.zeros(n))
world.step(render=False)


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    e, t = np.asarray(eye, float), np.asarray(target, float)
    f = t - e
    f /= np.linalg.norm(f)
    rr = np.cross(f, np.asarray(up, float))
    rr /= np.linalg.norm(rr)
    u = np.cross(rr, f)
    m = np.stack([rr, u, -f], axis=1)
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    return [w, (m[2, 1] - m[1, 2]) / (4 * w),
            (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)]


def save_png(path, arr):
    h, w = arr.shape[:2]
    a = np.ascontiguousarray(arr[:, :, :3].astype(np.uint8))
    raw = b"".join(b"\x00" + a[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    open(path, "wb").write(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


# ── 카메라 구성: 로봇별 초상 + 단체 ───────────────────────────────
cams = {}
for r in ROBOTS:
    eye = (r["x"], -r["cam_d"], r["cam_h"])
    tgt = (r["x"], 0.0, r["look"])
    rtx = RtxCamera(f"/World/Cam_{r['name']}", tick_rate=30.0,
                    translations=[list(eye)], orientations=[look_at_quat(eye, tgt)])
    cams[r["name"]] = CameraSensor(rtx, resolution=(900, 700), annotators=["rgb"])

g_eye, g_tgt = (0.2, -9.0, 2.6), (0.2, 0.0, 0.55)
g_rtx = RtxCamera("/World/Cam_Group", tick_rate=30.0,
                  translations=[list(g_eye)], orientations=[look_at_quat(g_eye, g_tgt)])
cams["group"] = CameraSensor(g_rtx, resolution=(700, 1800), annotators=["rgb"])

app_utils.play(commit=True)

print("\n===PORTRAIT===", flush=True)
# 자세가 안정될 때까지 (관절 목표는 매 스텝 유지)
for _ in range(400):
    for r in ROBOTS:
        if r["name"] in actions:
            r["obj"].apply_action(actions[r["name"]])
    world.step(render=True)

for r in ROBOTS:
    o = r["obj"]
    pos, q = o.get_world_pose()
    z = float(np.asarray(pos).ravel()[2])
    q = np.asarray(q, float).ravel()
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[1] ** 2 + q[2] ** 2)))))
    img = cams[r["name"]].get_data("rgb")[0].numpy()
    f = f"{OUT}/ch05_portrait_{r['name'].lower()}.png"
    save_png(f, img)
    print(f"  {r['name']:<11} DOF {len(o.dof_names):>2}  kp {r['kp']:>7.0f}  "
          f"z {z:.3f}  기울기 {tilt:5.1f}°  → {os.path.basename(f)}", flush=True)

img = cams["group"].get_data("rgb")[0].numpy()
save_png(f"{OUT}/ch05_portrait_group.png", img)
print(f"\n  단체 → ch05_portrait_group.png ({img.shape[1]}x{img.shape[0]})", flush=True)
print("===PORTRAIT_END===", flush=True)
app.close()
