"""Ch5.X Physics Inspector / Ch7.X OmniGraph — 단계별 캡처 생성 (책 삽입용).

Physics Inspector 의 슬라이더 조작은 GUI 동작이지만, 결과(관절이 어떻게 움직이는가)는
코드로 동일하게 재현할 수 있다. 슬라이더를 끝까지 민 모습을 그림으로 보여 주면
독자가 자기 화면과 비교할 수 있다.

산출물 (OUT_DIR):
    pi_01_franka_home.png        전 관절 0 — 기준 자세
    pi_02_joint1_min.png         joint1 하한 (-166°)
    pi_03_joint1_max.png         joint1 상한 (+166°)
    pi_04_joint4_bend.png        joint4 팔꿈치 굽힘
    pi_05_gripper_open.png       집게 열림 (Prismatic 상한)
    pi_06_gripper_closed.png     집게 닫힘 (Prismatic 하한)
    pi_07_go2_joints.png         Go2 12관절 — 다리 굽힘 자세
    og_01_carter_placed.png      Nova Carter 배치
    og_02_carter_driven.png      Differential Controller 주행 후

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python physics_inspector_captures.py
"""
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

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=8.0 / 200.0)
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
key.CreateIntensityAttr().Set(15000.0)
key.CreateWidthAttr().Set(14.0)
key.CreateHeightAttr().Set(10.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.0, -3.0, 6.0))
fill = UsdLux.RectLight.Define(stage, "/World/Lights/Fill")
fill.CreateIntensityAttr().Set(14000.0)
fill.CreateWidthAttr().Set(8.0)
fill.CreateHeightAttr().Set(6.0)
UsdGeom.XformCommonAPI(fill.GetPrim()).SetTranslate(Gf.Vec3d(2.5, -2.5, 2.0))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(3000.0)

# ── 로봇 ──────────────────────────────────────────────────────────
FRANKA = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
GO2 = "/Isaac/Robots/Unitree/Go2/configuration/go2_description_physics.usd"
stage_utils.add_reference_to_stage(usd_path=root + FRANKA, prim_path="/World/Franka")
stage_utils.add_reference_to_stage(usd_path=root + GO2, prim_path="/World/Go2")

franka = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))
go2 = world.scene.add(Robot(prim_path="/World/Go2", name="go2"))

# ── 카메라 ────────────────────────────────────────────────────────
rtx = RtxCamera("/World/CaptureCam", tick_rate=30.0)
cam = CameraSensor(rtx, resolution=(800, 1280), annotators=["rgb"])
_c = UsdGeom.Camera(stage.GetPrimAtPath("/World/CaptureCam"))
_c.CreateFocalLengthAttr().Set(24.0)
_c.CreateHorizontalApertureAttr().Set(20.955)

world.reset()
franka.set_world_pose(position=np.array([0.0, 0.0, 0.0]))
go2.set_world_pose(position=np.array([0.0, 6.0, 0.45]))   # 프레임 밖에 대기
world.reset()
app_utils.play(commit=True)


def shot(name, eye, target, freeze, frames=35, label=""):
    place_cam("/World/CaptureCam", eye, target)
    img = None
    for i in range(frames + 60):
        freeze()
        world.step(render=True)
        if i >= frames:
            data = cam.get_data("rgb")
            if data is not None:
                img = data[0].numpy()
                break
    if img is None:
        print(f"  [FAIL] {name}.png", flush=True)
        return
    save_png(f"{OUT}/{name}.png", img)
    print(f"  [OK] {name}.png  {img.shape[1]}x{img.shape[0]}  {label}", flush=True)


print("\n===PHYSICS_INSPECTOR_CAPTURE===", flush=True)

# ── 관절 스펙 출력 — 책의 관절 표를 실측으로 검증한다 ──────────────
names = list(franka.dof_names)
props = franka.dof_properties
print(f"\n  Franka 관절 수: {len(names)}", flush=True)
print(f"  {'관절':<22}{'하한':>9}{'상한':>9}{'강성':>9}{'감쇠':>8}", flush=True)
LIMITS = {}
for i, n in enumerate(names):
    lo, hi = float(props["lower"][i]), float(props["upper"][i])
    LIMITS[n] = (lo, hi)
    unit = "m" if "finger" in n else "deg"
    lo_d, hi_d = (lo, hi) if unit == "m" else (np.degrees(lo), np.degrees(hi))
    print(f"  {n:<22}{lo_d:>9.2f}{hi_d:>9.2f}"
          f"{float(props['stiffness'][i]):>9.0f}{float(props['damping'][i]):>8.0f}  {unit}",
          flush=True)

N = len(names)
HOME = np.zeros(N)
KP = np.ones(N) * 1200.0
KD = np.ones(N) * 60.0
franka.get_articulation_controller().set_gains(kps=KP, kds=KD)

CAM_F = ((1.9, -1.9, 1.1), (0.0, 0.0, 0.42))


def pin(pose):
    """목표 포즈를 유지한다 — Physics Inspector 슬라이더를 고정한 것과 같다."""
    def _f():
        franka.set_world_pose(position=np.array([0.0, 0.0, 0.0]))
        franka.set_joint_positions(pose)
        franka.set_joint_velocities(np.zeros(N))
        franka.apply_action(ArticulationAction(joint_positions=pose))
        go2.set_world_pose(position=np.array([0.0, 8.0, 0.45]))   # 프레임 밖으로
    return _f


# ── 1. 기준 자세 ──────────────────────────────────────────────────
shot("pi_01_franka_home", *CAM_F, freeze=pin(HOME), label="전 관절 0 — 기준 자세")

# ── 2~3. joint1 하한 / 상한 ───────────────────────────────────────
j1_lo, j1_hi = LIMITS[names[0]]
p = HOME.copy(); p[0] = j1_lo * 0.95
shot("pi_02_joint1_min", *CAM_F, freeze=pin(p),
     label=f"{names[0]} 하한 {np.degrees(j1_lo):.0f}deg")
p = HOME.copy(); p[0] = j1_hi * 0.95
shot("pi_03_joint1_max", *CAM_F, freeze=pin(p),
     label=f"{names[0]} 상한 {np.degrees(j1_hi):.0f}deg")

# ── 4. joint4 팔꿈치 굽힘 ─────────────────────────────────────────
p = HOME.copy()
if N > 3:
    j4_lo, j4_hi = LIMITS[names[3]]
    p[1] = -0.5
    p[3] = j4_lo * 0.8
shot("pi_04_joint4_bend", *CAM_F, freeze=pin(p), label="joint2/4 — 팔꿈치 접기")

# ── 5~6. 집게 열림 / 닫힘 ─────────────────────────────────────────
fingers = [i for i, n in enumerate(names) if "finger" in n]
base = HOME.copy()
base[1], base[3], base[5] = -0.3, -2.0, 1.9   # 집게를 앞으로 내민 자세


def hand_xyz():
    """집게(hand) prim 의 월드 좌표 — 카메라를 여기에 맞춘다."""
    for cand in ("/World/Franka/panda_hand", "/World/Franka/panda_link7"):
        prim = stage.GetPrimAtPath(cand)
        if prim and prim.IsValid():
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            t = m.ExtractTranslation()
            return np.array([t[0], t[1], t[2]])
    return np.array([0.42, 0.0, 0.45])


# 자세를 먼저 잡아 손 위치를 확정한 뒤 카메라를 건다
p_open = base.copy()
for i in fingers:
    p_open[i] = LIMITS[names[i]][1]
f_open = pin(p_open)
for _ in range(80):
    f_open()
    world.step(render=False)
hx = hand_xyz()
print(f"  집게 위치: {hx.round(3)}", flush=True)
CAM_G = ((hx[0] + 0.78, hx[1] - 1.00, hx[2] + 0.30), tuple(hx))

shot("pi_05_gripper_open", *CAM_G, freeze=f_open, label="집게 열림 (Prismatic 상한)")

p_close = base.copy()
for i in fingers:
    p_close[i] = LIMITS[names[i]][0]
shot("pi_06_gripper_closed", *CAM_G, freeze=pin(p_close), label="집게 닫힘 (Prismatic 하한)")

# ── 7. Go2 — 12관절 ───────────────────────────────────────────────
g_names = list(go2.dof_names)
GN = len(g_names)
print(f"\n  Go2 관절 수: {GN}  →  {', '.join(g_names[:4])} ...", flush=True)
go2.get_articulation_controller().set_gains(kps=np.ones(GN) * 60.0, kds=np.ones(GN) * 3.0)
g_pose = np.array([0.0 if "hip" in n else 0.35 if "thigh" in n else -0.70 for n in g_names])


def pin_go2():
    go2.set_world_pose(position=np.array([0.0, 0.0, 0.42]))
    go2.set_joint_positions(g_pose)
    go2.set_joint_velocities(np.zeros(GN))
    go2.apply_action(ArticulationAction(joint_positions=g_pose))
    franka.set_world_pose(position=np.array([0.0, 6.0, 0.0]))   # Franka 는 치운다


shot("pi_07_go2_joints", (1.4, -1.4, 0.75), (0.0, 0.0, 0.28),
     freeze=pin_go2, label=f"Go2 {GN}관절 — 다리 굽힘")

# ══════════════════════════════════════════════════════════════════
# Ch7.X OmniGraph — Nova Carter + Differential Controller
# ══════════════════════════════════════════════════════════════════
print("\n===OMNIGRAPH_CAPTURE===", flush=True)

go2.set_world_pose(position=np.array([0.0, 9.0, 0.45]))

CARTER = "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
stage_utils.add_reference_to_stage(usd_path=root + CARTER, prim_path="/World/Carter")
carter = world.scene.add(Robot(prim_path="/World/Carter", name="carter"))
world.reset()
app_utils.play(commit=True)

c_names = list(carter.dof_names)
CN = len(c_names)
print(f"  Nova Carter 관절 수: {CN}", flush=True)
print(f"  관절: {', '.join(c_names)}", flush=True)

wheels = [i for i, n in enumerate(c_names) if "wheel" in n.lower() and "caster" not in n.lower()]
print(f"  구동 바퀴 인덱스: {wheels} → {[c_names[i] for i in wheels]}", flush=True)

carter.set_world_pose(position=np.array([0.0, 0.0, 0.3]))
for _ in range(60):
    world.step(render=False)
start = np.asarray(carter.get_world_pose()[0]).ravel().copy()


def pin_carter_still():
    carter.set_world_pose(position=np.array([0.0, 0.0, 0.3]))
    carter.set_joint_velocities(np.zeros(CN))


shot("og_01_carter_placed", (1.7, -1.7, 0.95), (0.0, 0.0, 0.25),
     freeze=pin_carter_still, label="Nova Carter 배치 (Play 전)")

# Differential Controller 와 동일한 변환: v, w → 좌우 바퀴 각속도
WHEEL_R, WHEEL_D = 0.14, 0.41
v, w = 0.6, 0.35
left = (v - w * WHEEL_D / 2.0) / WHEEL_R
right = (v + w * WHEEL_D / 2.0) / WHEEL_R
print(f"  v={v} m/s, w={w} rad/s  →  좌 {left:.2f} rad/s, 우 {right:.2f} rad/s", flush=True)

cmd = np.zeros(CN)
if len(wheels) >= 2:
    cmd[wheels[0]] = left
    cmd[wheels[1]] = right
drive = ArticulationAction(joint_velocities=cmd)

for _ in range(700):
    carter.apply_action(drive)
    world.step(render=False)

end = np.asarray(carter.get_world_pose()[0]).ravel()
moved = float(np.linalg.norm(end[:2] - start[:2]))
print(f"  주행 거리: {moved:.2f} m  (시작 {start[:2].round(2)} → 끝 {end[:2].round(2)})", flush=True)


def follow_carter():
    carter.apply_action(drive)


place_cam("/World/CaptureCam",
          (end[0] + 1.7, end[1] - 1.7, 0.95), (end[0], end[1], 0.25))
img = None
for i in range(95):
    follow_carter()
    world.step(render=True)
    if i >= 35:
        d = cam.get_data("rgb")
        if d is not None:
            img = d[0].numpy()
            break
if img is not None:
    save_png(f"{OUT}/og_02_carter_driven.png", img)
    print(f"  [OK] og_02_carter_driven.png  {img.shape[1]}x{img.shape[0]}  "
          f"Differential Controller 주행 후", flush=True)

# ── OmniGraph 노드 이름 검증 (책 7.X.7 코드) ──────────────────────
print("\n  OmniGraph 노드 이름 검증 (책 7.X.7 코드를 실제로 실행):", flush=True)
import omni.graph.core as og  # noqa: E402

CANDIDATES = {
    "OnPlaybackTick": ["omni.graph.action.OnPlaybackTick"],
    "DifferentialController": [
        "isaacsim.robot.wheeled_robots.DifferentialController",
        "omni.isaac.wheeled_robots.DifferentialController",
    ],
    "ArticulationController": [
        "isaacsim.core.nodes.IsaacArticulationController",
        "omni.isaac.core_nodes.IsaacArticulationController",
    ],
}

resolved = {}
for label, cands in CANDIDATES.items():
    hit = None
    for c in cands:
        try:
            og.Controller.edit(
                {"graph_path": f"/World/_probe_{label}", "evaluator_name": "execution"},
                {og.Controller.Keys.CREATE_NODES: [("n", c)]},
            )
            hit = c
            break
        except Exception:
            continue
    resolved[label] = hit
    print(f"    [{'OK  ' if hit else '없음'}] {label:<24} -> {hit or ' / '.join(cands)}", flush=True)

if all(resolved.values()):
    try:
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("tick", resolved["OnPlaybackTick"]),
                    ("diff_ctrl", resolved["DifferentialController"]),
                    ("art_ctrl", resolved["ArticulationController"]),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("diff_ctrl.inputs:maxLinearSpeed", 1.0),
                    ("diff_ctrl.inputs:wheelDistance", WHEEL_D),
                    ("diff_ctrl.inputs:wheelRadius", WHEEL_R),
                    ("art_ctrl.inputs:robotPath", "/World/Carter"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("tick.outputs:tick", "diff_ctrl.inputs:execIn"),
                    ("diff_ctrl.outputs:execOut", "art_ctrl.inputs:execIn"),
                    ("diff_ctrl.outputs:velocityCommand", "art_ctrl.inputs:velocityCommand"),
                ],
            },
        )
        print(f"    [OK  ] Action Graph 생성 성공 — 노드 {len(nodes)}개", flush=True)
    except Exception as e:
        print(f"    [실패] Action Graph 생성: {type(e).__name__}: {e}", flush=True)

print("===OMNIGRAPH_CAPTURE_END===", flush=True)
app.close()
