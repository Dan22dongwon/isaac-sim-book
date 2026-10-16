"""22.x 1단계 — 미로를 주행하며 LiDAR 데이터와 주행 라벨을 모은다.

JetBot + 회전형 RTX LiDAR 로 상자 미로를 돌며
    (거리 벡터 N개, 라벨 forward/left/right)
를 CSV 로 저장한다. 이 CSV 가 2단계 분류기 학습의 입력이다.

라벨은 '규칙 기반 주행기'가 만든다. 영상에서는 사람이 키보드로 몰았지만,
책에서는 누가 돌려도 같은 결과가 나와야 하므로 규칙으로 대체했다.
분류기는 이 규칙을 LiDAR 입력만 보고 흉내 내도록 학습된다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python lidar_maze_collect.py
"""
import csv
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
from isaacsim.core.api.objects import FixedCuboid  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.sensors.experimental.rtx import (  # noqa: E402
    CameraSensor, Lidar, LidarSensor, RtxCamera, parse_generic_model_output_data,
)
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "./out")
os.makedirs(OUT, exist_ok=True)
root = nucleus_utils.get_assets_root_path()

N_BINS = 36          # LiDAR 한 바퀴를 36칸(10도)으로 요약한다
MAX_RANGE = 4.0      # 이보다 먼 것은 '열려 있다'로 본다

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=4.0 / 120.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()


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


def place_cam(prim_path, eye, target, up=(0.0, 0.0, 1.0)):
    e, t = np.asarray(eye, float), np.asarray(target, float)
    f = t - e
    f /= np.linalg.norm(f)
    rr = np.cross(f, np.asarray(up, float))
    rr /= np.linalg.norm(rr)
    u = np.cross(rr, f)
    m = Gf.Matrix4d(1.0)
    m.SetRow(0, Gf.Vec4d(*rr, 0.0))
    m.SetRow(1, Gf.Vec4d(*u, 0.0))
    m.SetRow(2, Gf.Vec4d(*(-f), 0.0))
    m.SetRow(3, Gf.Vec4d(*e, 1.0))
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


# ── 조명 ──────────────────────────────────────────────────────────
key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(9000.0)
key.CreateWidthAttr().Set(24.0)
key.CreateHeightAttr().Set(24.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 0.0, 9.0))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(1200.0)

# ── 미로 ──────────────────────────────────────────────────────────
# ⚠️ 장애물에 충돌체가 없으면 LiDAR 가 통과해 버린다.
#    FixedCuboid 는 충돌체를 기본 포함하므로 그대로 감지된다.
WALLS = [
    # (중심 x, 중심 y, 가로, 세로)  — 바깥 울타리
    (0.0, 3.2, 6.6, 0.2), (0.0, -3.2, 6.6, 0.2),
    (3.2, 0.0, 0.2, 6.6), (-3.2, 0.0, 0.2, 6.6),
    # 안쪽 칸막이 — 통로가 꺾이도록
    (-1.0, 1.4, 0.2, 3.4), (1.0, -1.4, 0.2, 3.4),
    (1.4, 1.6, 3.4, 0.2), (-1.4, -1.6, 3.4, 0.2),
]
for i, (cx, cy, sx, sy) in enumerate(WALLS):
    FixedCuboid(prim_path=f"/World/Maze/Wall_{i}", name=f"wall_{i}",
                position=np.array([cx, cy, 0.25]),
                scale=np.array([sx, sy, 0.5]),
                color=np.array([0.80, 0.42, 0.20]))

# ── JetBot ────────────────────────────────────────────────────────
JETBOT = "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
stage_utils.add_reference_to_stage(usd_path=root + JETBOT, prim_path="/World/Jetbot")
jetbot = world.scene.add(Robot(prim_path="/World/Jetbot", name="jetbot"))

# ── 센서 ──────────────────────────────────────────────────────────
# ⚠️ 센서 선택이 결과를 좌우한다.
#    Example_Rotary 는 차량용 장거리 LiDAR 라 '최소 거리 1.45 m' 다.
#    통로 폭 1.6 m 미로에서는 벽이 최소 거리 안쪽으로 들어와 아무것도 안 보인다.
#    실내 소형 로봇에는 RPLIDAR S2E(최소 0.40 m)가 맞다.
lidar = Lidar.create("/World/Jetbot/chassis/Lidar", config="Slamtec_RPLIDAR_S2E",
                     translations=[[0.0, 0.0, 0.16]])
lidar_sensor = LidarSensor(lidar, annotators=["generic-model-output"])

rtx = RtxCamera("/World/CaptureCam", tick_rate=30.0)
cam = CameraSensor(rtx, resolution=(800, 1280), annotators=["rgb"])
_c = UsdGeom.Camera(stage.GetPrimAtPath("/World/CaptureCam"))
_c.CreateFocalLengthAttr().Set(24.0)
_c.CreateHorizontalApertureAttr().Set(20.955)

world.reset()
app_utils.play(commit=True)

names = list(jetbot.dof_names)
print(f"\n===LIDAR_COLLECT===", flush=True)
print(f"  JetBot 관절: {names}", flush=True)
LEFT = next(i for i, n in enumerate(names) if "left" in n.lower())
RIGHT = next(i for i, n in enumerate(names) if "right" in n.lower())
print(f"  좌/우 바퀴 인덱스: {LEFT}, {RIGHT}", flush=True)

# ⚠️ 속도 제어에서는 강성(kp)=0, 감쇠(kd)>0 이어야 한다.
#    에셋 기본 게인 그대로면 바퀴가 전혀 돌지 않는다.
n_dof = len(names)
jetbot.get_articulation_controller().set_gains(
    kps=np.zeros(n_dof), kds=np.ones(n_dof) * 600.0
)


_fields_shown = [False]


def scan():
    """LiDAR 한 프레임을 N_BINS 칸의 거리 벡터로 요약한다."""
    data, _ = lidar_sensor.get_data("generic-model-output")
    if data is None:
        return None
    gmo = parse_generic_model_output_data(data)
    if not _fields_shown[0]:
        _fields_shown[0] = True
        print(f"  gmo 필드: {[f for f in dir(gmo) if not f.startswith('_')]}", flush=True)
    # ⚠️ 핵심: Example_Rotary 의 출력은 직교좌표가 아니라 '구면좌표'다.
    #    gmo.elementsCoordsType == CoordsType.SPHERICAL 이고
    #      x = 방위각(도, -180~180) · y = 고도각(도) · z = 거리(m)
    #    이걸 x,y 를 평면 좌표로 착각하고 hypot 을 쓰면 값이 전부 엉킨다.
    az = np.asarray(gmo.x, float)
    el = np.asarray(gmo.y, float)
    dist = np.asarray(gmo.z, float)
    if az.size == 0:
        return None
    # 수평에 가까운 링만 쓴다 — 바닥·천장 반사를 걸러낸다
    ok = np.isfinite(dist) & (dist > 0.05) & (np.abs(el) < 3.0)
    az, dist = az[ok], dist[ok]
    if az.size == 0:
        return None
    idx = (np.mod(az, 360.0) / (360.0 / N_BINS)).astype(int) % N_BINS
    out = np.full(N_BINS, MAX_RANGE, dtype=np.float32)
    np.minimum.at(out, idx, np.clip(dist, 0.0, MAX_RANGE))
    return out


# 회전 임계값은 LiDAR 최소 거리(0.40 m)보다 넉넉히 커야 한다.
# 그렇지 않으면 벽이 최소 거리 안으로 들어와 '반환 없음'이 되고,
# 그걸 '멀다'로 오해해 그대로 박는다.
TURN_AT = 0.95


def sectors(bins):
    n = len(bins)
    front = np.concatenate([bins[: n // 12], bins[-(n // 12):]]).min()
    left = bins[n // 12: n // 3].min()
    right = bins[-(n // 3): -(n // 12)].min()
    return front, left, right


def rule_driver(bins, stuck=False):
    """규칙 기반 주행기 — 이 판단이 학습 라벨이 된다."""
    front, left, right = sectors(bins)
    if front > TURN_AT and not stuck:
        return "forward"
    return "left" if left > right else "right"


WHEEL = {"forward": (7.0, 7.0), "left": (-3.0, 6.0), "right": (6.0, -3.0)}


def drive(label):
    cmd = np.zeros(len(names))
    l, r = WHEEL[label]
    cmd[LEFT], cmd[RIGHT] = l, r
    jetbot.apply_action(ArticulationAction(joint_velocities=cmd))


def shot(name, eye, target, frames=30, label=""):
    place_cam("/World/CaptureCam", eye, target)
    img = None
    for i in range(frames + 60):
        world.step(render=True)
        if i >= frames:
            d = cam.get_data("rgb")
            if d is not None:
                img = d[0].numpy()
                break
    if img is not None:
        save_png(f"{OUT}/{name}.png", img)
        print(f"  [OK] {name}.png  {label}", flush=True)


jetbot.set_world_pose(position=np.array([-2.2, -2.3, 0.05]))
for _ in range(60):
    world.step(render=False)

shot("ml_01_maze_overview", (0.0, -7.6, 7.2), (0.0, 0.0, 0.0),
     label="상자 미로 + JetBot 전경")

# ── 수집 ──────────────────────────────────────────────────────────
# ⚠️ RTX LiDAR 는 world.step(render=True) 로만 갱신된다.
#    render=False 로 돌리면 첫 프레임 값이 그대로 고정돼, 로봇이 제자리에서 맴돈다.
rows = []
STEPS = 2600
warm = 0
# 벽에 끼면 센서만으로는 빠져나오지 못한다 — 위치 정체를 감시해 강제로 회전시킨다
last_pos = np.asarray(jetbot.get_world_pose()[0]).ravel()[:2].copy()
stall = 0
stuck = False
for step in range(STEPS):
    if step % 40 == 0:
        now = np.asarray(jetbot.get_world_pose()[0]).ravel()[:2]
        stall = stall + 1 if np.linalg.norm(now - last_pos) < 0.03 else 0
        last_pos = now.copy()
        stuck = stall >= 2
    b = scan()
    if b is None:
        warm += 1
        world.step(render=True)
        continue
    lab = rule_driver(b, stuck=stuck)
    if step in (0, 400, 1200, 2000):
        nb = len(b)
        fr = np.concatenate([b[: nb // 12], b[-(nb // 12):]]).min()
        lf = b[nb // 12: nb // 3].min()
        rt = b[-(nb // 3): -(nb // 12)].min()
        pos_ = np.asarray(jetbot.get_world_pose()[0]).ravel()
        vel_ = np.asarray(jetbot.get_joint_velocities()).ravel()
        q = np.asarray(jetbot.get_world_pose()[1]).ravel()
        yaw = np.degrees(np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2)))
        print(f"    step{step:>5} front={fr:.2f} L={lf:.2f} R={rt:.2f} -> {lab:<7} "
              f"pos={pos_[:2].round(2)} yaw={yaw:6.1f}", flush=True)
        print(f"      bins={np.round(b,2).tolist()}", flush=True)
    drive(lab)
    rows.append(list(np.round(b, 4)) + [lab])
    world.step(render=True)

pos = np.asarray(jetbot.get_world_pose()[0]).ravel()
print(f"  LiDAR 예열 {warm}스텝 · 수집 {len(rows)}행 · 최종 위치 {pos[:2].round(2)}", flush=True)

csv_path = f"{OUT}/lidar_maze.csv"
with open(csv_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow([f"d{i:02d}" for i in range(N_BINS)] + ["label"])
    w.writerows(rows)

from collections import Counter  # noqa: E402
cnt = Counter(r[-1] for r in rows)
print(f"  라벨 분포: {dict(cnt)}", flush=True)
print(f"  저장: {csv_path}", flush=True)

shot("ml_02_driving", (pos[0] + 1.6, pos[1] - 1.9, 1.3), (pos[0], pos[1], 0.1),
     label="주행 중 — LiDAR 로 벽을 피한다")
print("===LIDAR_COLLECT_END===", flush=True)
app.close()
