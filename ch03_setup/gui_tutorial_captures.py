"""Ch3.5 GUI 따라하기 — 단계별 캡처 생성 (책 삽입용).

GUI로 손으로 하는 각 단계를 코드로 재현하고, 매 단계의 결과를 렌더링해
"이 버튼을 누르면 이런 화면이 된다"를 그림으로 보여 줄 수 있게 한다.

산출물 (OUT_DIR):
    gui_step01_cube_created.png      Create > Mesh > Cube
    gui_step02_cube_material.png     Create > Material > OmniPBR 바인딩
    gui_step03_physics_before.png     + Add > Physics > Rigid Body (Play 전)
    gui_step04_physics_after.png      Play — 큐브 낙하 후
    gui_step05_warehouse.png          Content Browser > warehouse.usd 더블클릭
    gui_step06_franka_placed.png      Franka 드래그 & 드롭
    gui_step07_robots_placed.png      Spot + Go2 추가 배치
    gui_step08_robots_collapsed.png   Play — 토크 없이 주저앉음
    gui_step09_challenge_before.png   5분 챌린지 씬 (Play 전)
    gui_step10_challenge_after.png    5분 챌린지 (Play 후)

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python gui_tutorial_captures.py
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
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "./out")
os.makedirs(OUT, exist_ok=True)
root = nucleus_utils.get_assets_root_path()

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=8.0 / 200.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()


# ── PNG 저장 (외부 의존 없이) ──────────────────────────────────────
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
    """카메라 prim 을 eye 에 두고 target 을 보게 한다.

    [ERR-027] 참조 에셋은 이미 xformOp 이 있을 수 있어 ClearXformOpOrder 로 먼저 비운다.
    """
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
key.CreateIntensityAttr().Set(5000.0)
key.CreateWidthAttr().Set(18.0)
key.CreateHeightAttr().Set(12.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.0, -4.0, 8.0))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(300.0)

# ── 캡처 카메라 ───────────────────────────────────────────────────
rtx = RtxCamera("/World/CaptureCam", tick_rate=30.0)
# 주의: resolution 은 (height, width) 순서다. (1280, 800) 을 주면 세로로 길쭉한 그림이 나온다.
cam = CameraSensor(rtx, resolution=(800, 1280), annotators=["rgb"])

# USD 카메라 기본 초점거리는 50mm — 화각이 24° 라 로봇 여러 대가 프레임에 안 들어온다.
# 24mm 로 낮추면 수평 화각이 약 47° 가 되어 씬 전체가 담긴다.
_cam_usd = UsdGeom.Camera(stage.GetPrimAtPath("/World/CaptureCam"))
_cam_usd.CreateFocalLengthAttr().Set(24.0)
_cam_usd.CreateHorizontalApertureAttr().Set(20.955)

world.reset()
app_utils.play(commit=True)


def shot(name, eye, target, frames=40, freeze=None, label=""):
    """카메라를 배치하고 안정화 후 저장한다.

    RTX 센서는 world.step(render=True) 로만 틱된다 — world.render() 로는 데이터가 안 온다.
    그래서 'Play 전' 상태는 매 스텝 포즈를 다시 고정(freeze)해서 정지 화면을 만든다.
    """
    place_cam("/World/CaptureCam", eye, target)
    img = None
    for i in range(frames + 60):
        if freeze is not None:
            freeze()
        world.step(render=True)
        if i >= frames:
            data = cam.get_data("rgb")
            if data is not None:
                img = data[0].numpy()
                break
    if img is None:
        print(f"  [FAIL] {name}.png — RGB 데이터 없음", flush=True)
        return
    path = f"{OUT}/{name}.png"
    save_png(path, img)
    print(f"  [OK] {name}.png  {img.shape[1]}x{img.shape[0]}  {label}", flush=True)


print("\n===GUI_TUTORIAL_CAPTURE===", flush=True)

CAM_CUBE = ((1.4, -1.8, 1.2), (0.0, 0.0, 0.2))

# ── 1. Create > Mesh > Cube ───────────────────────────────────────
VisualCuboid(prim_path="/World/Cube", name="cube_visual",
             position=np.array([0.0, 0.0, 0.15]), size=0.3,
             color=np.array([0.55, 0.55, 0.58]))
world.reset()
shot("gui_step01_cube_created", *CAM_CUBE, label="Create > Mesh > Cube")

# ── 2. 재질 입히기 (빨간 큐브로 교체) ──────────────────────────────
stage.RemovePrim("/World/Cube")   # VisualCuboid 는 scene 에 등록하지 않았으므로 prim 만 지운다
VisualCuboid(prim_path="/World/Cube", name="cube_red",
             position=np.array([0.0, 0.0, 0.15]), size=0.3,
             color=np.array([0.85, 0.15, 0.12]))
world.reset()
shot("gui_step02_cube_material", *CAM_CUBE, label="Create > Material > OmniPBR")

# ── 3. 물리 부여 — Play 전 (공중에 띄운 상태) ──────────────────────
stage.RemovePrim("/World/Cube")
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/Cube", name="cube_rb",
                  position=np.array([0.0, 0.0, 1.20]), size=0.3,
                  color=np.array([0.85, 0.15, 0.12]))
)
world.reset()


def freeze_cube():
    cube.set_world_pose(position=np.array([0.0, 0.0, 1.20]))
    cube.set_linear_velocity(np.zeros(3))
    cube.set_angular_velocity(np.zeros(3))


shot("gui_step03_physics_before", (1.8, -2.4, 1.8), (0.0, 0.0, 0.7),
     freeze=freeze_cube, label="+ Add > Physics > Rigid Body (Play 전)")

# ── 4. Play — 낙하 후 ─────────────────────────────────────────────
for _ in range(400):
    world.step(render=False)
shot("gui_step04_physics_after", (1.8, -2.4, 1.8), (0.0, 0.0, 0.7),
     label="Play — 큐브 낙하 착지")
z = float(np.asarray(cube.get_world_pose()[0]).ravel()[2])
print(f"       큐브 최종 높이 z = {z:.3f} m (반변 0.15 → 착지 성공)", flush=True)

# ── 5. Warehouse 환경 로딩 ────────────────────────────────────────
world.scene.remove_object("cube_rb")   # scene 등록 객체는 remove_object 가 prim 까지 정리한다
WAREHOUSE = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
stage_utils.add_reference_to_stage(usd_path=root + WAREHOUSE, prim_path="/World/Warehouse")
world.reset()
shot("gui_step05_warehouse", (6.0, -8.0, 2.6), (0.0, 0.0, 1.0),
     label="Content Browser > warehouse.usd")

# ── 6. Franka 드래그 & 드롭 ───────────────────────────────────────
FRANKA = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
stage_utils.add_reference_to_stage(usd_path=root + FRANKA, prim_path="/World/Franka")
franka = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))
world.reset()
franka.set_world_pose(position=np.array([0.0, 0.0, 0.0]))
FRANKA_HOME = np.array([0.0, -0.6, 0.0, -2.2, 0.0, 2.0, 0.78, 0.04, 0.04])


def freeze_franka():
    """토크가 없으면 팔이 처진다. 드래그&드롭 직후 모습은 포즈를 고정해서 보여 준다."""
    franka.set_world_pose(position=np.array([0.0, 0.0, 0.0]))
    n = len(franka.dof_names)
    pose = FRANKA_HOME[:n] if n <= len(FRANKA_HOME) else np.zeros(n)
    franka.set_joint_positions(pose)
    franka.set_joint_velocities(np.zeros(n))


shot("gui_step06_franka_placed", (1.5, -2.0, 1.0), (0.0, 0.0, 0.45),
     freeze=freeze_franka, label="Franka 드래그 & 드롭")

# ── 7. Spot + Go2 추가 배치 ───────────────────────────────────────
SPOT = "/Isaac/Robots/BostonDynamics/spot/spot.usd"
GO2 = "/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd"
stage_utils.add_reference_to_stage(usd_path=root + SPOT, prim_path="/World/Spot")
stage_utils.add_reference_to_stage(usd_path=root + GO2, prim_path="/World/Go2")
spot = world.scene.add(Robot(prim_path="/World/Spot", name="spot"))
go2 = world.scene.add(Robot(prim_path="/World/Go2", name="go2"))
world.reset()
# 에셋 기본 관절값(= 서 있는 자세)을 기억해 둔다
SPOT_HOME = np.asarray(spot.get_joint_positions()).ravel().copy()
GO2_HOME = np.array([0.0 if "hip" in n else 0.35 if "thigh" in n else -0.70
                     for n in go2.dof_names])


def freeze_all():
    """드롭 직후(Play 전) 모습 — 세 로봇의 포즈를 모두 고정한다."""
    freeze_franka()
    spot.set_world_pose(position=np.array([-1.3, 0.0, 0.52]))
    spot.set_joint_positions(SPOT_HOME)
    spot.set_joint_velocities(np.zeros(len(spot.dof_names)))
    go2.set_world_pose(position=np.array([1.3, 0.0, 0.45]))
    go2.set_joint_positions(GO2_HOME)
    go2.set_joint_velocities(np.zeros(len(go2.dof_names)))


shot("gui_step07_robots_placed", (0.0, -4.8, 1.5), (0.0, 0.0, 0.50),
     freeze=freeze_all, label="Spot + Go2 추가 — 토크 없는 초기 상태")
print(f"       Franka DOF {len(franka.dof_names)} / Spot DOF {len(spot.dof_names)} / "
      f"Go2 DOF {len(go2.dof_names)}", flush=True)

# ── 8. Play — 토크 없이 주저앉음 ──────────────────────────────────
for _ in range(500):
    world.step(render=False)
shot("gui_step08_robots_collapsed", (0.0, -4.8, 1.5), (0.0, 0.0, 0.35),
     label="Play — 토크 0 이면 주저앉는다 (정상)")
for nm, r in (("Franka", franka), ("Spot", spot), ("Go2", go2)):
    zz = float(np.asarray(r.get_world_pose()[0]).ravel()[2])
    print(f"       {nm:<7} Play 후 z = {zz:.3f} m", flush=True)

# ── 9~10. 5분 챌린지 — Warehouse + Franka + Spot + 큐브 3개 ───────
world.scene.remove_object("go2")
cubes = []
for i, x in enumerate((-0.35, 0.0, 0.35)):
    c = world.scene.add(
        DynamicCuboid(prim_path=f"/World/Box{i}", name=f"box{i}",
                      position=np.array([x, 0.65, 0.9]), size=0.12,
                      color=np.array([0.95, 0.70, 0.15]))
    )
    cubes.append(c)
world.reset()


def freeze_challenge():
    freeze_franka()
    spot.set_world_pose(position=np.array([-1.3, 0.0, 0.52]))
    spot.set_joint_positions(SPOT_HOME)
    spot.set_joint_velocities(np.zeros(len(spot.dof_names)))
    for j, xx in enumerate((-0.35, 0.0, 0.35)):
        cubes[j].set_world_pose(position=np.array([xx, 0.65, 0.9]))
        cubes[j].set_linear_velocity(np.zeros(3))
        cubes[j].set_angular_velocity(np.zeros(3))


shot("gui_step09_challenge_before", (0.3, -4.2, 1.5), (-0.5, 0.2, 0.45),
     freeze=freeze_challenge, label="5분 챌린지 — Play 전")

for _ in range(500):
    world.step(render=False)
shot("gui_step10_challenge_after", (0.3, -4.2, 1.5), (-0.5, 0.2, 0.35),
     label="5분 챌린지 — Play 후")
for i, c in enumerate(cubes):
    zz = float(np.asarray(c.get_world_pose()[0]).ravel()[2])
    print(f"       Box{i} 최종 z = {zz:.3f} m", flush=True)

print("===GUI_TUTORIAL_CAPTURE_END===", flush=True)
app.close()
