"""책 5.4~5.6 검증 — AMR / 4족보행 / 휴머노이드를 실제로 불러와 나란히 세운다.

책이 "예상 출력"으로 적어 둔 관절 이름·개수가 실제와 맞는지 대조하고,
결과를 이미지로 남긴다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    HEADLESS=1 OUT_DIR=./out python robot_gallery.py
"""
import math
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
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "")
if OUT:
    os.makedirs(OUT, exist_ok=True)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
nucleus = nucleus_utils.get_assets_root_path()
stage = omni.usd.get_context().get_stage()

# [ERR-022] 경로는 전부 6.1 기준으로 교정한 것이다.
ROBOTS = [
    # (표시이름, USD 경로, prim, x위치, 서있는 높이, 책이 적은 DOF)
    ("Nova Carter (AMR)", "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
     "/World/R_NovaCarter", -3.6, 0.31, None),
    ("Unitree Go2 (4족)", "/Isaac/Robots/Unitree/Go2/go2.usd",
     "/World/R_Go2", -1.2, 0.45, 12),
    ("Boston Dynamics Spot (4족)", "/Isaac/Robots/BostonDynamics/spot/spot.usd",
     "/World/R_Spot", 1.2, 0.70, 12),
    ("Unitree H1 (휴머노이드)", "/Isaac/Robots/Unitree/H1/h1.usd",
     "/World/R_H1", 3.8, 1.05, 19),
]

print("\n===GALLERY===", flush=True)

loaded = []
for label, rel, prim_path, x, z, book_dof in ROBOTS:
    try:
        stage_utils.add_reference_to_stage(usd_path=nucleus + rel, prim_path=prim_path)
        prim = stage.GetPrimAtPath(prim_path)
        n_prim = sum(1 for _ in Usd.PrimRange(prim))
        if n_prim <= 1:
            print(f"❌ {label:<28} 빈 참조 — 에셋 경로 확인 필요", flush=True)
            continue
        # [ERR-027] 참조 에셋은 AddTranslateOp 재호출이 불가 → XformCommonAPI 사용
        UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(x, 0.0, z))
        loaded.append((label, prim_path, book_dof, n_prim))
        print(f"✅ {label:<28} prim {n_prim:>5}개  배치 x={x}", flush=True)
    except Exception as e:
        print(f"❌ {label:<28} {type(e).__name__}: {str(e)[:70]}", flush=True)

# 조명
key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(12000.0)
key.CreateWidthAttr().Set(12.0)
key.CreateHeightAttr().Set(8.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.0, -1.0, 6.0))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(600.0)

world.reset()

# ── 관절 이름·개수 대조 ───────────────────────────────────────────
print("\n[관절 구성 — 책의 서술과 대조]", flush=True)
for label, prim_path, book_dof, _ in loaded:
    try:
        r = Robot(prim_path=prim_path, name=prim_path.split("/")[-1].lower())
        world.scene.add(r)
        world.reset()
        names = list(r.dof_names)
        mark = "" if book_dof is None else (" ✅ 일치" if len(names) == book_dof
                                            else f" ❌ 책은 {book_dof}개라고 적음")
        print(f"  {label}", flush=True)
        print(f"    DOF {len(names)}개{mark}", flush=True)
        print(f"    {names[:6]}", flush=True)
        if len(names) > 6:
            print(f"    {names[6:12]}{' ...' if len(names) > 12 else ''}", flush=True)
    except Exception as e:
        print(f"  {label}  ❌ {type(e).__name__}: {str(e)[:80]}", flush=True)

# ── 결과 이미지 ───────────────────────────────────────────────────
if OUT:
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

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

    EYE, TGT = (0.2, -9.0, 2.6), (0.2, 0.0, 0.5)
    rtx = RtxCamera("/World/GalleryCam", tick_rate=30.0,
                    translations=[list(EYE)], orientations=[look_at_quat(EYE, TGT)])
    cam = CameraSensor(rtx, resolution=(540, 1280), annotators=["rgb"])
    app_utils.play(commit=True)
    # 관절 목표를 주지 않으면 로봇이 중력에 무너진다.
    # 불러온 직후 형상을 보여주는 것이 목적이므로 최소 스텝만 돌린다.
    for _ in range(12):
        world.step(render=True)

    img = cam.get_data("rgb")[0].numpy()
    h, w = img.shape[:2]
    a = np.ascontiguousarray(img[:, :, :3].astype(np.uint8))
    raw = b"".join(b"\x00" + a[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    open(f"{OUT}/ch05_robot_gallery.png", "wb").write(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    print(f"\n  저장: ch05_robot_gallery.png (평균 밝기 {img.mean():.0f})", flush=True)

print("===GALLERY_END===", flush=True)
app.close()
