"""Chapter 6 실습 — 센서가 실제로 본 것을 이미지로 저장한다.

RGB / 깊이 / 세그멘테이션 / LiDAR 포인트클라우드를 PNG로 남긴다.
책 6.2~6.3의 결과물을 눈으로 확인하는 용도다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python sensor_capture.py
"""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid  # noqa: E402
from isaacsim.core.utils.semantics import add_labels  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "./out")
os.makedirs(OUT, exist_ok=True)
root = nucleus_utils.get_assets_root_path()

# ── 씬 구성 ───────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

# 바닥 위에 색이 다른 상자 몇 개 (깊이·세그멘테이션이 잘 보이도록 거리를 다르게)
boxes = [
    ("Red", [0.0, 0.0, 0.15], [0.3, 0.3, 0.3], [0.9, 0.2, 0.2]),
    ("Green", [0.6, 0.5, 0.25], [0.5, 0.5, 0.5], [0.2, 0.8, 0.3]),
    ("Blue", [-0.7, 0.9, 0.2], [0.4, 0.4, 0.4], [0.2, 0.4, 0.9]),
    ("Yellow", [0.2, 1.6, 0.35], [0.7, 0.7, 0.7], [0.9, 0.8, 0.2]),
]
for name, pos, scale, color in boxes:
    p = world.scene.add(
        FixedCuboid(
            prim_path=f"/World/Props/Box_{name}",
            name=f"box_{name.lower()}",
            position=np.array(pos),
            scale=np.array(scale),
            color=np.array(color),
        )
    )
    add_labels(stage.GetPrimAtPath(f"/World/Props/Box_{name}"),
               labels=["box"], instance_name="class")

# 로봇 하나 (센서가 볼 대상)
stage_utils.add_reference_to_stage(
    usd_path=root + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    prim_path="/World/UR5e",
)
UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/UR5e")).SetTranslate(
    Gf.Vec3d(-0.2, 2.6, 0.0)
)
add_labels(stage.GetPrimAtPath("/World/UR5e"), labels=["robot"], instance_name="class")

# 조명 — RectLight는 회전 없이도 -Z(아래)를 비춘다 (→ ERR-021)
light = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
light.CreateIntensityAttr().Set(8000.0)
light.CreateWidthAttr().Set(6.0)
light.CreateHeightAttr().Set(6.0)
UsdGeom.XformCommonAPI(light.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 1.0, 5.0))
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
dome.CreateIntensityAttr().Set(400.0)

world.reset()

# ── 6.2 카메라 ────────────────────────────────────────────────────
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402

# USD 카메라는 로컬 -Z 방향을 본다. 기본 상태는 수직 아래를 보므로,
# X축으로 기울여 앞을 보게 한다. XformCommonAPI로 나중에 덮어쓰면 적용되지 않으므로
# 생성자에 orientations(쿼터니언 w,x,y,z)로 직접 준다.
import math
_theta = math.radians(72.0)
_q = [math.cos(_theta / 2), math.sin(_theta / 2), 0.0, 0.0]   # (w, x, y, z)
rtx_camera = RtxCamera(
    "/World/Cam",
    tick_rate=30.0,
    translations=[[0.0, -2.2, 1.3]],
    orientations=[_q],
)

camera = CameraSensor(
    rtx_camera,
    resolution=(480, 640),  # (height, width)
    annotators=["rgb", "distance_to_image_plane", "semantic_segmentation"],
)
app_utils.play(commit=True)
for _ in range(90):
    world.step(render=True)

print("\n===CAPTURE===", flush=True)


def save_png(path, arr):
    """외부 라이브러리 없이 PNG를 쓴다 (zlib + struct 만 사용)."""
    import struct
    import zlib

    h, w = arr.shape[:2]
    if arr.ndim == 2:
        arr = np.repeat(arr[:, :, None], 3, axis=2)
    arr = np.ascontiguousarray(arr[:, :, :3].astype(np.uint8))
    raw = b"".join(b"\x00" + arr[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    open(path, "wb").write(png)
    return path


# RGB — get_data()는 warp 배열이므로 .numpy() 필요 (→ ERR-025)
rgb, _ = camera.get_data("rgb")
rgb_np = rgb.numpy()
save_png(f"{OUT}/ch06_camera_rgb.png", rgb_np)
print(f"  RGB    shape={rgb_np.shape} 평균={rgb_np.mean():.1f}  → ch06_camera_rgb.png", flush=True)

# 깊이 — 컬러맵 없이 명암으로 (가까울수록 밝게)
depth, _ = camera.get_data("distance_to_image_plane")
d = np.asarray(depth.numpy()).squeeze()
finite = np.isfinite(d) & (d < 50)
lo, hi = (d[finite].min(), d[finite].max()) if finite.any() else (0.0, 1.0)
dn = np.zeros_like(d)
dn[finite] = 255.0 * (1.0 - (d[finite] - lo) / max(hi - lo, 1e-6))
save_png(f"{OUT}/ch06_camera_depth.png", dn.astype(np.uint8))
print(f"  Depth  범위={lo:.2f}~{hi:.2f}m  → ch06_camera_depth.png", flush=True)

# 세그멘테이션 — 라벨 ID를 색으로
try:
    seg, seg_info = camera.get_data("semantic_segmentation")
    s = np.asarray(seg.numpy()).squeeze().astype(np.uint32)
    rng = np.random.default_rng(7)
    lut = rng.integers(40, 255, size=(int(s.max()) + 2, 3), dtype=np.uint8)
    lut[0] = [25, 25, 30]
    save_png(f"{OUT}/ch06_camera_seg.png", lut[np.clip(s, 0, len(lut) - 1)])
    print(f"  Seg    고유 라벨={np.unique(s).tolist()[:8]}  → ch06_camera_seg.png", flush=True)
except Exception as e:
    print(f"  Seg    건너뜀: {type(e).__name__}: {str(e)[:70]}", flush=True)

# ── 6.3 LiDAR — 포인트클라우드를 위에서 본 그림으로 ───────────────
from isaacsim.sensors.experimental.rtx import (  # noqa: E402
    Lidar,
    LidarSensor,
    parse_generic_model_output_data,
)

lidar = Lidar.create("/World/Lidar", config="Example_Rotary", translations=[[0.0, 0.0, 0.4]])
lsensor = LidarSensor(lidar, annotators=["generic-model-output"])
app_utils.play(commit=True)
for _ in range(60):
    world.step(render=True)

data, _ = lsensor.get_data("generic-model-output")
gmo = parse_generic_model_output_data(data)
# ⚠️ 6.1 GenericModelOutput의 x / y / z 는 **직교좌표가 아니다**.
#    elementsCoordsType == CoordsType.SPHERICAL 이며 의미는 다음과 같다:
#       x = 방위각 azimuth (도),  y = 고도각 elevation (도),  z = 거리 range (m)
#    좌표계는 frameOfReference == SENSOR (센서 기준).
#    (N,3) 형태의 pointCloudData 필드는 존재하지 않는다 → ERR-026 참조
pts = None
if all(hasattr(gmo, a) for a in ("x", "y", "z")):
    n = int(getattr(gmo, "numElements", 0) or 0)
    az_deg, el_deg, rng = (np.asarray(getattr(gmo, a)).ravel() for a in ("x", "y", "z"))
    if not n:
        n = min(len(az_deg), len(el_deg), len(rng))
    az, el, r = np.radians(az_deg[:n]), np.radians(el_deg[:n]), rng[:n]

    # 구면 → 직교 변환
    X = r * np.cos(el) * np.cos(az)
    Y = r * np.cos(el) * np.sin(az)
    Z = r * np.sin(el)
    pts = np.stack([X, Y, Z], axis=1)
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[r[: len(pts)] > 1e-3]   # 미명중(거리 0) 제거

if pts is not None and len(pts):
    S = 512
    img = np.full((S, S, 3), 18, np.uint8)
    r = 6.0
    m = (np.abs(pts[:, 0]) < r) & (np.abs(pts[:, 1]) < r)
    p = pts[m]
    px = ((p[:, 0] + r) / (2 * r) * (S - 1)).astype(int)
    py = ((r - p[:, 1]) / (2 * r) * (S - 1)).astype(int)
    z = p[:, 2]
    zc = np.clip((z - z.min()) / max(np.ptp(z), 1e-6), 0, 1)
    img[py, px] = np.stack([(60 + 195 * zc), (120 + 100 * zc), (255 - 120 * zc)], -1).astype(np.uint8)
    img[S // 2 - 2:S // 2 + 3, S // 2 - 2:S // 2 + 3] = [255, 60, 60]  # 센서 위치
    save_png(f"{OUT}/ch06_lidar_topdown.png", img)
    print(f"  LiDAR  포인트 {len(pts):,}개 (표시 {m.sum():,})  → ch06_lidar_topdown.png", flush=True)
else:
    fields = [f for f in dir(gmo) if not f.startswith("_")]
    print(f"  LiDAR  포인트 배열을 못 찾음. 사용 가능 필드: {fields[:12]}", flush=True)

print("===CAPTURE_END===", flush=True)
app.close()
