"""책 6.9 그리퍼 로드 + 개폐 제어 — 동작 검증본.

원문과의 차이:
  [ERR-022] 경로 404.
      /Isaac/Robots/Robotiq/2F85/2f85_instanceable.usd
    → /Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd
  [ERR-009] isaacsim.core.api.articulations.Articulation 은 삭제됨
    → isaacsim.core.prims.Articulation (prim_paths_expr)
  [ERR-035] "액티브 DOF 1개" 는 사실과 다르다 — 실제로 6개가 노출된다.
            다만 구동은 finger_joint 하나로 충분하다 (mimic 연동 확인).
            단수 ArticulationAction 이 아니라 복수 ArticulationActions 를 써야 한다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python gripper_control.py
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
from isaacsim.core.prims import Articulation  # noqa: E402  [ERR-009]
from isaacsim.core.utils.types import ArticulationActions  # noqa: E402  [ERR-035]
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "")
if OUT:
    os.makedirs(OUT, exist_ok=True)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
nucleus = nucleus_utils.get_assets_root_path()

# [ERR-022] 올바른 그리퍼 경로
stage_utils.add_reference_to_stage(
    usd_path=nucleus + "/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd",
    prim_path="/World/Gripper")
UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/Gripper")).SetTranslate(
    Gf.Vec3d(0.0, 0.0, 0.25))

key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(16000.0)
key.CreateWidthAttr().Set(2.0)
key.CreateHeightAttr().Set(2.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.25, -0.45, 0.9))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(1000.0)

world.reset()
gripper = Articulation(prim_paths_expr="/World/Gripper", name="gripper")
world.scene.add(gripper)
world.reset()

names = list(gripper.dof_names)
DRIVE = "finger_joint"
DRIVE_IDX = np.array([names.index(DRIVE)])
MAX_CLOSE = 0.8      # rad — 2F-85 완전 폐쇄

print("\n===GRIPPER===", flush=True)
print(f"노출된 DOF {len(names)}개", flush=True)
for i, nm in enumerate(names):
    print(f"  [{i}] {nm}{'   ← 구동 조인트' if nm == DRIVE else '   (mimic)'}", flush=True)
print(f"\n[ERR-035] 원고는 '액티브 DOF 1개' 라 하지만 실제로는 {len(names)}개가 노출된다.",
      flush=True)
print(f"          다만 구동은 {DRIVE} 하나로 충분하다 — 아래에서 확인한다.", flush=True)


def set_gripper(gripper, open_ratio: float):
    """open_ratio: 0.0(완전 개방) ~ 1.0(완전 폐쇄).

    구동 조인트 하나만 지정한다. 나머지 5개는 mimic 으로 따라온다.
    [ERR-035] 뷰 클래스에는 복수형 ArticulationActions 를 넘겨야 한다.
    """
    gripper.apply_action(ArticulationActions(
        joint_positions=np.array([[open_ratio * MAX_CLOSE]]),
        joint_indices=DRIVE_IDX,
    ))


def settle(ratio, steps=240, render=False):
    for _ in range(steps):
        set_gripper(gripper, ratio)
        world.step(render=render)
    return np.asarray(gripper.get_joint_positions())[0].copy()


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
    e, t = np.asarray(eye, float), np.asarray(target, float)
    fv = t - e
    fv /= np.linalg.norm(fv)
    r = np.cross(fv, np.asarray(up, float))
    r /= np.linalg.norm(r)
    u = np.cross(r, fv)
    m = np.stack([r, u, -fv], axis=1)
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


cam = None
if OUT:
    TGT = (0.0, 0.0, 0.33)
    EYE = (TGT[0] + 0.02, TGT[1] - 0.42, TGT[2] + 0.10)
    rtx = RtxCamera("/World/GripCam", tick_rate=30.0,
                    translations=[list(EYE)], orientations=[look_at_quat(EYE, TGT)])
    cam = CameraSensor(rtx, resolution=(800, 640), annotators=["rgb"])
    app_utils.play(commit=True)
    for _ in range(40):
        world.step(render=True)

q_open = settle(0.0, render=bool(OUT))
if OUT and cam is not None:
    d = cam.get_data("rgb")[0]
    if d is not None:
        save_png(f"{OUT}/ch06_gripper_open.png", d.numpy())

q_close = settle(1.0, render=bool(OUT))
if OUT and cam is not None:
    d = cam.get_data("rgb")[0]
    if d is not None:
        save_png(f"{OUT}/ch06_gripper_closed.png", d.numpy())

print(f"\n{'조인트':<36}{'개방':>9}{'폐쇄':>9}{'변화':>9}", flush=True)
print("-" * 64, flush=True)
followers = 0
for i, nm in enumerate(names):
    d = q_close[i] - q_open[i]
    if i != DRIVE_IDX[0] and abs(d) > 0.02:
        followers += 1
    print(f"{nm:<36}{q_open[i]:>9.3f}{q_close[i]:>9.3f}{d:>9.3f}", flush=True)

print(f"\n구동 조인트 1개만 제어 → 나머지 {followers}/{len(names)-1}개가 따라 움직였다.",
      flush=True)
print(f"판정: {'✅ mimic 연동 확인 — 원고 설명대로다' if followers >= 3 else '❌ 연동 없음'}",
      flush=True)
if OUT:
    print(f"\n저장: ch06_gripper_open.png, ch06_gripper_closed.png", flush=True)
print("===GRIPPER_END===", flush=True)
app.close()
