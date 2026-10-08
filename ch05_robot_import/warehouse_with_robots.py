"""책 5.8 [실습] 창고 씬에 로봇 배치하기 — 동작 검증본.

원문과의 차이:
  [ERR-022] 에셋 경로 2곳이 404다. 제조사별 디렉터리로 재편되었다.
      Franka : /Isaac/Robots/Franka/franka.usd
             → /Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd
      Carter : /Isaac/Robots/Carter/carter_v1.usd
             → /Isaac/Robots/NVIDIA/Carter/carter_v1.usd
  [ERR-028] WheeledRobot / DifferentialController 를 신규 경로로 import 해놓고
            구형 인자(prim_path, name, create_robot, position)를 넘긴다.
  [ERR-027] 참조 에셋에 AddTranslateOp()를 다시 부르면 예외가 난다.
            franka_xform.AddTranslateOp() → ClearXformOpOrder() 후 호출로 수정.
  [추가]    원문에는 카메라·조명 배치가 없어 화면에 아무것도 안 보인다.
            결과를 눈으로 확인할 수 있도록 추가했다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python warehouse_with_robots.py            # GUI
    HEADLESS=1 OUT_DIR=./out python warehouse_with_robots.py   # 이미지 저장
"""
import os
import struct
import zlib

import numpy as np
from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
app = SimulationApp({"headless": HEADLESS})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import FixedCuboid  # noqa: E402
from isaacsim.robot.experimental.wheeled_robots.controllers import (  # noqa: E402
    DifferentialController,
)
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "")
if OUT:
    os.makedirs(OUT, exist_ok=True)

world = World(stage_units_in_meters=1.0)
nucleus = nucleus_utils.get_assets_root_path()
stage = omni.usd.get_context().get_stage()

# ── 1. 바닥 ────────────────────────────────────────────────────────
world.scene.add_default_ground_plane()

# ── 2. 작업 테이블 (Franka 앞) ─────────────────────────────────────
world.scene.add(
    FixedCuboid(
        prim_path="/World/Table",
        name="work_table",
        position=np.array([0.5, 0.0, 0.4]),
        scale=np.array([0.8, 1.2, 0.8]),
        color=np.array([0.6, 0.5, 0.4]),
    )
)

# ── 3. Franka 팔로봇 배치 ─────────────────────────────────────────
# [ERR-022] 원문 경로는 404
franka_usd = nucleus + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
stage_utils.add_reference_to_stage(usd_path=franka_usd, prim_path="/World/Robots/Franka")
# [ERR-027] 참조해온 에셋에는 이미 xformOp:translate 가 있어서
#           AddTranslateOp()를 다시 부르면 예외가 난다.
#           XformCommonAPI 를 쓰거나, ClearXformOpOrder() 후 추가해야 한다.
_fx = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Robots/Franka"))
_fx.ClearXformOpOrder()
_fx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.8))

# ── 4. Carter AMR 배치 ────────────────────────────────────────────
# [ERR-022] 원문 경로는 404
carter_usd = nucleus + "/Isaac/Robots/NVIDIA/Carter/carter_v1.usd"
# [ERR-028] 원문은 **신규 import 경로**에 **구형 API 인자**를 넘긴다.
#   구형 isaacsim.robot.wheeled_robots        : prim_path=, name=, create_robot=, position=
#   신규 isaacsim.robot.experimental...       : paths(위치인자), positions=, orientations=
#                                               (name / create_robot 인자 없음)
carter = WheeledRobot(
    "/World/Robots/Carter",
    wheel_dof_names=["left_wheel", "right_wheel"],
    usd_path=carter_usd,
    positions=np.array([[-3.0, 0.0, 0.1]]),
)

# ── [추가] 조명 — 원문에 없어 화면이 어둡다 ───────────────────────
key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(9000.0)
key.CreateWidthAttr().Set(8.0)
key.CreateHeightAttr().Set(8.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(-1.0, 0.0, 5.0))
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
dome.CreateIntensityAttr().Set(500.0)

# ── 5. 씬 초기화 ──────────────────────────────────────────────────
world.reset()

n_franka = sum(1 for _ in Usd.PrimRange(stage.GetPrimAtPath("/World/Robots/Franka")))
n_carter = sum(1 for _ in Usd.PrimRange(stage.GetPrimAtPath("/World/Robots/Carter")))
print("\n✅ 창고 + 로봇 2종 씬 완성!", flush=True)
print(f"   Franka prim {n_franka}개 / Carter prim {n_carter}개", flush=True)
print(f"   Carter 관절 이름 = {list(carter.dof_names)[:6]} ...", flush=True)
print(f"   Carter 전체 DOF  = {len(carter.dof_names)}", flush=True)

# ── [추가] 결과 확인용 카메라 ─────────────────────────────────────
cam_np = None
if OUT:
    import math

    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

    def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)):
        """eye에서 target을 바라보는 USD 카메라 자세를 (w,x,y,z)로 돌려준다.

        USD 카메라는 로컬 -Z 를 바라보고 +Y 가 위쪽이다.
        """
        e = np.asarray(eye, float)
        t = np.asarray(target, float)
        f = t - e
        f /= np.linalg.norm(f)          # forward  → 카메라의 -Z
        u0 = np.asarray(up, float)
        r = np.cross(f, u0)             # right    → 카메라의 +X
        r /= np.linalg.norm(r)
        u = np.cross(r, f)              # up       → 카메라의 +Y
        m = np.stack([r, u, -f], axis=1)   # 열이 각각 카메라의 X, Y, Z 축
        w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
        if w < 1e-8:
            return [1.0, 0.0, 0.0, 0.0]
        return [w,
                (m[2, 1] - m[1, 2]) / (4 * w),
                (m[0, 2] - m[2, 0]) / (4 * w),
                (m[1, 0] - m[0, 1]) / (4 * w)]

    EYE, TGT = (-2.6, -4.6, 2.2), (-0.6, 0.0, 0.5)
    rtx = RtxCamera(
        "/World/RigCam",
        tick_rate=30.0,
        translations=[list(EYE)],
        orientations=[look_at_quat(EYE, TGT)],
    )
    cam = CameraSensor(rtx, resolution=(540, 960), annotators=["rgb"])
    app_utils.play(commit=True)

# ── 6. AMR 전진 주행 ──────────────────────────────────────────────
# [ERR-028] 신규 DifferentialController 는 name 인자가 없고 전부 키워드 전용이다.
diff_controller = DifferentialController(wheel_radius=0.125, wheel_base=0.46)

start = np.asarray(carter.get_world_poses()[0], dtype=float).ravel().copy()
shots = {}

for step in range(600):
    action = diff_controller.forward(command=np.array([0.3, 0.0]))
    carter.apply_wheel_actions(action)
    world.step(render=True)

    if step == 300:
        print("  → AMR 이동 중... (Step 300)", flush=True)

    if OUT and step in (60, 350, 599):
        shots[step] = cam.get_data("rgb")[0].numpy().copy()

end = np.asarray(carter.get_world_poses()[0], dtype=float).ravel()
print(f"\n  AMR 이동 거리 = {np.linalg.norm(end - start):.2f} m "
      f"(시작 x={start[0]:.2f} → 끝 x={end[0]:.2f})", flush=True)


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
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


for step, img in shots.items():
    tag = {60: "start", 350: "mid", 599: "end"}[step]
    p = f"{OUT}/ch05_warehouse_robots_{tag}.png"
    save_png(p, img)
    print(f"  저장: {os.path.basename(p)}  (평균 밝기 {img.mean():.0f})", flush=True)

# GUI로 볼 때 창을 유지한다 (HOLD_SECONDS 초). AMR은 제자리에서 계속 돈다.
_hold = float(os.environ.get("HOLD_SECONDS", "0"))
if _hold > 0:
    import time
    print(f"===HOLD_START=== {_hold:.0f}초 유지", flush=True)
    _t = time.time() + _hold
    while time.time() < _t:
        carter.apply_wheel_actions(diff_controller.forward(command=np.array([0.0, 0.6])))
        world.step(render=True)
    print("===HOLD_END===", flush=True)

print("\n✅ 실습 완료!", flush=True)
app.close()
