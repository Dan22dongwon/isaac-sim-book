"""책 5.5 Go2 서기 자세 검증 — 관절 순서 가정이 맞는지 실측한다.

책의 stand_pose 배열은 '다리별' 순서(FL hip/thigh/calf → FR …)를 가정하지만,
Isaac Sim의 실제 DOF 순서는 '관절 종류별'(전체 hip → 전체 thigh → 전체 calf)이다.
두 방식을 각각 적용해 로봇의 몸통 높이와 자세를 비교한다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    HEADLESS=1 OUT_DIR=./out python go2_stand_pose.py
"""
import math
import os
import struct
import zlib

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "1") == "1"})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "")
if OUT:
    os.makedirs(OUT, exist_ok=True)
root = nucleus_utils.get_assets_root_path()

# [ERR-029] 책의 go2.usd 에는 관절·ArticulationRoot 가 없다 (시각 형상만).
#           물리가 들어 있는 configuration/go2_description_physics.usd 를 써야 한다.
GO2 = "/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd"

# 책의 stand_pose — '다리별' 순서를 가정한다
BOOK_POSE = np.array([
    0.0, 0.8, -1.6,   # FL (앞왼쪽)
    0.0, 0.8, -1.6,   # FR (앞오른쪽)
    0.0, 1.0, -1.8,   # RL (뒤왼쪽)
    0.0, 1.0, -1.8,   # RR (뒤오른쪽)
])

# 같은 의도를 '관절 이름'으로 표현한 것 (순서에 의존하지 않는다)
TARGET_BY_NAME = {
    "FL_hip_joint": 0.0, "FL_thigh_joint": 0.8, "FL_calf_joint": -1.6,
    "FR_hip_joint": 0.0, "FR_thigh_joint": 0.8, "FR_calf_joint": -1.6,
    "RL_hip_joint": 0.0, "RL_thigh_joint": 1.0, "RL_calf_joint": -1.8,
    "RR_hip_joint": 0.0, "RR_thigh_joint": 1.0, "RR_calf_joint": -1.8,
}


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


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    e, t = np.asarray(eye, float), np.asarray(target, float)
    f = t - e
    f /= np.linalg.norm(f)
    r = np.cross(f, np.asarray(up, float))
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    m = np.stack([r, u, -f], axis=1)
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    return [w, (m[2, 1] - m[1, 2]) / (4 * w),
            (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)]


def run(label, use_book_order, tag):
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()

    stage_utils.add_reference_to_stage(usd_path=root + GO2, prim_path="/World/Go2")
    UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/Go2")).SetTranslate(
        Gf.Vec3d(0.0, 0.0, 0.45))

    key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr().Set(10000.0)
    key.CreateWidthAttr().Set(6.0)
    key.CreateHeightAttr().Set(6.0)
    UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 0.0, 4.0))
    UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(500.0)

    go2 = Robot(prim_path="/World/Go2", name="go2")
    world.scene.add(go2)
    world.reset()

    names = list(go2.dof_names)
    if use_book_order:
        pose = BOOK_POSE.copy()                      # 순서를 그대로 믿는다
    else:
        pose = np.array([TARGET_BY_NAME[n] for n in names])  # 이름으로 매핑

    ctrl = go2.get_articulation_controller()
    ctrl.set_gains(kps=np.ones(len(names)) * 80.0, kds=np.ones(len(names)) * 2.0)
    action = ArticulationAction(joint_positions=pose)
    go2.apply_action(action)

    cam = None
    if OUT:
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

        EYE, TGT = (1.5, -1.6, 0.6), (0.0, 0.0, 0.25)
        rtx = RtxCamera(f"/World/Cam_{tag}", tick_rate=30.0,
                        translations=[list(EYE)], orientations=[look_at_quat(EYE, TGT)])
        cam = CameraSensor(rtx, resolution=(540, 720), annotators=["rgb"])
        app_utils.play(commit=True)

    for _ in range(240):
        go2.apply_action(action)          # 매 스텝 목표 유지
        world.step(render=bool(OUT))

    dbg = np.asarray(go2.get_joint_positions(), dtype=float)
    print(f"  [진단] 목표 vs 실제 관절각 차이 max = "
          f"{np.abs(dbg - pose).max():.3f} rad", flush=True)
    kp, kd = go2.get_articulation_controller().get_gains()[:2]
    print(f"  [진단] kp[0]={np.asarray(kp).ravel()[0]:.1f}  "
          f"kd[0]={np.asarray(kd).ravel()[0]:.1f}", flush=True)

    base_z = float(np.asarray(go2.get_world_pose()[0])[2])
    q = np.asarray(go2.get_world_pose()[1], dtype=float)      # (w, x, y, z)
    # 몸통이 얼마나 기울었는가 — 로컬 +Z 와 월드 +Z 의 각도
    w_, x_, y_, z_ = q
    up_z = 1 - 2 * (x_ * x_ + y_ * y_)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))

    print(f"\n[{label}]", flush=True)
    print(f"  적용 배열 앞 6개 : {np.round(pose[:6], 2).tolist()}", flush=True)
    print(f"  → 실제 관절      : {names[:6]}", flush=True)
    print(f"  몸통 높이 z      = {base_z:.3f} m", flush=True)
    print(f"  몸통 기울기      = {tilt:.1f}°", flush=True)

    if OUT and cam is not None:
        img = cam.get_data("rgb")[0].numpy()
        save_png(f"{OUT}/ch05_go2_{tag}.png", img)
        print(f"  저장: ch05_go2_{tag}.png", flush=True)
    return base_z, tilt


MODE = os.environ.get("MODE", "book")     # book | name
print("\n===GO2_STAND===", flush=True)
if MODE == "book":
    run("책 그대로 — 배열 순서를 그대로 신뢰", True, "book_order")
else:
    run("수정안 — 관절 이름으로 매핑", False, "name_mapped")
print("===GO2_STAND_END===", flush=True)
app.close()
