"""책 5.3 Franka Pick & Place — 동작 검증본.

원문과의 차이:
  [ERR-015] `isaacsim.robot.manipulators.examples` 는 기본 비활성 확장이다.
            확장을 켜지 않으면 ModuleNotFoundError 가 난다.
  [추가]    원문은 집을 물체를 만들지 않는다. pick_position 에 큐브를 놓아야
            실제로 집는 동작을 확인할 수 있다.
  [추가]    조명·카메라가 없어 결과를 눈으로 볼 수 없다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python franka_pick_place.py
"""
import math
import os
import struct
import zlib

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "1") == "1"})

import omni.kit.app  # noqa: E402

# [ERR-015] 확장을 먼저 켜야 Franka 예제 모듈을 import 할 수 있다
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.robot.manipulators.examples", True
)

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid  # noqa: E402
from isaacsim.robot.manipulators.examples.franka import Franka  # noqa: E402
from isaacsim.robot.manipulators.examples.franka.controllers import (  # noqa: E402
    PickPlaceController,
)
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "")
if OUT:
    os.makedirs(OUT, exist_ok=True)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

# ── Franka 로드 ───────────────────────────────────────────────────
franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka"))

# ── [추가] 집을 물체 — 원문에는 없다 ──────────────────────────────
PICK = np.array([0.32, 0.00, 0.03])
PLACE = np.array([0.32, -0.30, 0.03])
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/TargetCube",
        name="target_cube",
        position=PICK.copy(),
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([0.9, 0.2, 0.2]),
        size=1.0,
        mass=0.03,
    )
)

# ── [추가] 조명 ───────────────────────────────────────────────────
key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(9000.0)
key.CreateWidthAttr().Set(4.0)
key.CreateHeightAttr().Set(4.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.4, -0.6, 2.2))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(600.0)

world.reset()

controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka.gripper,
    robot_articulation=franka,
)

print("\n===PICKPLACE===", flush=True)
print(f"Franka DOF {franka.num_dof} / 집을 위치 {PICK.tolist()} → 놓을 위치 {PLACE.tolist()}",
      flush=True)


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
    EYE, TGT = (1.2, -1.1, 0.75), (0.25, -0.12, 0.2)
    rtx = RtxCamera("/World/Cam", tick_rate=30.0,
                    translations=[list(EYE)], orientations=[look_at_quat(EYE, TGT)])
    cam = CameraSensor(rtx, resolution=(700, 900), annotators=["rgb"])
    app_utils.play(commit=True)
    # 렌더 프로덕트가 준비될 때까지 워밍업 — 너무 이르면 get_data 가 None 을 준다
    for _ in range(30):
        world.step(render=True)

shots = {}
print(f"{'step':>6}{'큐브 z':>9}{'큐브 y':>9}  상태", flush=True)

for step in range(1200):
    action = controller.forward(
        picking_position=PICK,
        placing_position=PLACE,
        current_joint_positions=franka.get_joint_positions(),
    )
    franka.apply_action(action)
    world.step(render=bool(OUT))

    if step % 200 == 0 or controller.is_done():
        cp = np.asarray(cube.get_world_pose()[0]).ravel()
        print(f"{step:>6}{cp[2]:>9.3f}{cp[1]:>9.3f}  "
              f"{'집어 올림' if cp[2] > 0.06 else '바닥'}", flush=True)
        if OUT and step in (0, 400, 800):
            d = cam.get_data("rgb")[0]
            if d is not None:
                shots[step] = d.numpy().copy()

    if controller.is_done():
        print(f"\n✅ Pick & Place 완료! (step {step})", flush=True)
        break

final = np.asarray(cube.get_world_pose()[0]).ravel()
moved = float(np.linalg.norm(final[:2] - PICK[:2]))
err = float(np.linalg.norm(final[:2] - PLACE[:2]))
print(f"\n  큐브 최종 위치 : {np.round(final, 3).tolist()}", flush=True)
print(f"  목표까지 오차  : {err:.3f} m   (이동 거리 {moved:.3f} m)", flush=True)
print(f"  판정           : {'✅ 성공' if err < 0.08 else '❌ 실패'}", flush=True)

if OUT:
    d = cam.get_data("rgb")[0]
    if d is not None:
        shots[9999] = d.numpy().copy()
    for k, img in shots.items():
        tag = {0: "start", 400: "grasp", 800: "move", 9999: "done"}.get(k, str(k))
        save_png(f"{OUT}/ch05_pickplace_{tag}.png", img)
        print(f"  저장: ch05_pickplace_{tag}.png", flush=True)

print("===PICKPLACE_END===", flush=True)
app.close()
