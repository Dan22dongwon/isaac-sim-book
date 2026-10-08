"""책 6.9.6 [실습] 실제 하드웨어 구성 재현 — 동작 검증본.

UR5e + Robotiq 2F-85 그리퍼 + RealSense 깊이 카메라를 한 리그로 조립한다.

원문과의 차이:
  [ERR-036] ★가장 중요★ 원고는 그리퍼·카메라를 팔의 링크 **아래**에 참조한다.
            USD 계층의 부모-자식은 물리 부착이 아니다. 게다가 관절체 안에
            관절체를 중첩하면 PhysX 가 발산한다 (Invalid PhysX transform 441개,
            z=nan). → 최상위에 **형제**로 두고 FixedJoint 로 연결한다.
  [ERR-022] 그리퍼 경로 404.
      /Isaac/Robots/Robotiq/2F85/2f85_instanceable.usd
    → /Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd
  [ERR-023] RealSense **D435 에셋이 없다.** D455 / D457 만 제공된다.
      /Isaac/Sensors/Intel/RealSense/rsd435.usd
    → /Isaac/Sensors/RealSense/D455/rsd455.usd
  [ERR-027] 참조 에셋에 AddTranslateOp() 재호출 → 예외.
            XformCommonAPI.SetTranslate 로 교체.
  [추가]    조명·카메라가 없어 결과를 볼 수 없다. 팔이 자세를 못 잡고 주저앉는다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    HEADLESS=0 python hardware_rig.py          # 화면으로 보기
    OUT_DIR=./out python hardware_rig.py       # 이미지로 저장
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
from isaacsim.core.prims import Articulation, RigidPrim  # noqa: E402
from isaacsim.core.utils.types import ArticulationActions  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "")
if OUT:
    os.makedirs(OUT, exist_ok=True)
GUI = os.environ.get("HEADLESS", "1") == "0"

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
nucleus = nucleus_utils.get_assets_root_path()
stage = omni.usd.get_context().get_stage()

# ── 부품 — 전부 **최상위 형제**로 둔다 [ERR-036] ──────────────────
PARTS = [
    ("UR5e 로봇 팔", "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd", "/World/UR5e"),
    # [ERR-022] 2F85/2f85_instanceable.usd → 2F-85/Robotiq_2F_85_edit.usd
    ("Robotiq 2F-85 그리퍼", "/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd",
     "/World/Gripper"),
    # [ERR-023] rsd435.usd 는 존재하지 않는다 → D455
    ("RealSense D455", "/Isaac/Sensors/RealSense/D455/rsd455.usd", "/World/RealSense"),
]

print("\n===RIG===", flush=True)
for label, rel, prim_path in PARTS:
    stage_utils.add_reference_to_stage(usd_path=nucleus + rel, prim_path=prim_path)
    prim = stage.GetPrimAtPath(prim_path)
    n = sum(1 for _ in Usd.PrimRange(prim)) if prim.IsValid() else 0
    print(f"  {'✅' if n > 1 else '❌'} {label:<22} prim {n:>4}개  {rel}", flush=True)

# 팔 바닥을 월드에 고정 — 없으면 팔 전체가 넘어진다
base_fix = UsdPhysics.FixedJoint.Define(stage, "/World/base_fix")
base_fix.CreateBody1Rel().SetTargets(["/World/UR5e/base_link"])
base_fix.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

# ── [ERR-036] 관절체는 하나여야 한다 ──────────────────────────────
# UR5e 와 2F-85 는 각자 ArticulationRoot 를 가진 **별개의 관절체**다.
# PhysX 는 두 관절체를 조인트로 잇지 못한다. FixedJoint 를 걸면
# 관절체 자체가 깨져 Articulation 뷰를 만들 수 없게 된다
#   (AttributeError: 'NoneType' object has no attribute 'is_homogeneous').
# 그리퍼의 ArticulationRoot 를 떼어 내 팔의 관절체에 편입시킨다.
roots = [p for base in ("/World/Gripper", "/World/UR5e")
         for p in Usd.PrimRange(stage.GetPrimAtPath(base))
         if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print(f"\n  ArticulationRoot {len(roots)}개 발견:", flush=True)
for p in roots:
    print(f"    {p.GetPath()}", flush=True)

grip_root = stage.GetPrimAtPath("/World/Gripper/Robotiq_2F_85")
grip_root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
if grip_root.HasAPI(PhysxSchema.PhysxArticulationAPI):
    grip_root.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
print(f"  → 그리퍼의 ArticulationRoot 제거. 남은 루트는 UR5e 하나뿐이다.", flush=True)

# 조명
key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(13000.0)
key.CreateWidthAttr().Set(5.0)
key.CreateHeightAttr().Set(5.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(1.2, -1.4, 2.6))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(900.0)

world.reset()

POSE = np.array([0.0, -1.2, 1.4, -1.75, -1.57, 0.0])   # 팔꿈치를 세운 자세

# ── 1단계: 플랜지가 어디 있는지 알아낸다 ──────────────────────────
# 뷰를 world.scene 에 **등록하지 않는다**. FixedJoint 를 추가하면 관절체
# 토폴로지가 바뀌어 기존 뷰가 무효가 되고, 다음 world.reset() 에서
# AttributeError: 'NoneType' object has no attribute 'is_homogeneous' 가 난다.
arm0 = Articulation(prim_paths_expr="/World/UR5e", name="ur5e_tmp")
arm0.initialize()
arm0.set_joint_positions(POSE[None, :])
arm0.set_joint_velocities(np.zeros((1, 6)))
for _ in range(5):
    world.step(render=False)

flange0 = RigidPrim(prim_paths_expr="/World/UR5e/wrist_3_link", name="flange_tmp")
flange0.initialize()
fpos, fquat = flange0.get_world_poses()
fpos = np.asarray(fpos)[0].astype(float)
fquat = np.asarray(fquat)[0].astype(float)
print(f"\n  플랜지(wrist_3_link) 물리 위치 = {np.round(fpos, 3).tolist()}", flush=True)

# ── 2단계: 부품을 플랜지 자리로 옮긴다 ────────────────────────────
grip0 = RigidPrim(prim_paths_expr="/World/Gripper/Robotiq_2F_85/base_link",
                  name="grip_tmp")
grip0.initialize()
grip0.set_world_poses(positions=np.array([fpos + np.array([0.0, 0.0, 0.02])]),
                      orientations=np.array([fquat]))

# D455 는 관절체가 아니므로 USD 트랜스폼으로 배치한다 [ERR-027]
UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/RealSense")).SetTranslate(
    Gf.Vec3d(*(fpos + np.array([0.07, 0.0, 0.03]))))
world.step(render=False)

# ── 3단계: [ERR-036] 진짜 부착은 FixedJoint 가 만든다 ─────────────
fj = UsdPhysics.FixedJoint.Define(stage, "/World/gripper_attach")
fj.CreateBody0Rel().SetTargets(["/World/UR5e/wrist_3_link"])
fj.CreateBody1Rel().SetTargets(["/World/Gripper/Robotiq_2F_85/base_link"])
fj.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.02))
fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

# ── 4단계: 토폴로지가 바뀌었으니 뷰를 **새로** 만든다 ─────────────
world.reset()
arm = Articulation(prim_paths_expr="/World/UR5e", name="ur5e")
arm.initialize()
flange = RigidPrim(prim_paths_expr="/World/UR5e/wrist_3_link", name="flange")
flange.initialize()
grip = RigidPrim(prim_paths_expr="/World/Gripper/Robotiq_2F_85/base_link", name="grip")
grip.initialize()

all_names = list(arm.dof_names)
arm.set_joint_positions(
    np.concatenate([POSE, np.zeros(len(all_names) - 6)])[None, :])
arm.set_joint_velocities(np.zeros((1, len(all_names))))
for _ in range(10):
    world.step(render=False)

print("\n✅ 실제 하드웨어 구성 리그 완성", flush=True)
print("   - UR5e 로봇 팔 (바닥 고정)", flush=True)
print("   - Robotiq 2F-85 그리퍼  ※ FixedJoint 로 플랜지에 부착 [ERR-036]", flush=True)
print("   - RealSense D455  ※ 원고의 D435 는 에셋이 없다", flush=True)


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


# 카메라는 **물리 위치**를 기준으로 잡는다. XformCache 는 물리를 반영하지 않는다.
g_now = np.asarray(grip.get_world_poses()[0])[0].astype(float)
TGT = tuple(float(v) for v in (g_now + np.array([0.0, 0.0, -0.05])))
EYE = (TGT[0] + 0.55, TGT[1] - 0.75, TGT[2] + 0.35)
print(f"  그리퍼 물리 위치 {np.round(g_now, 3).tolist()} → 카메라가 이 지점을 본다",
      flush=True)

cam = None
if OUT:
    rtx = RtxCamera("/World/RigCam", tick_rate=30.0,
                    translations=[list(EYE)], orientations=[look_at_quat(EYE, TGT)])
    cam = CameraSensor(rtx, resolution=(1000, 800), annotators=["rgb"])
    app_utils.play(commit=True)
    for _ in range(40):
        world.step(render=True)

if GUI:
    # 화면에서 리그가 보이도록 뷰포트 카메라를 옮긴다
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=list(EYE), target=list(TGT))
    except Exception as e:
        print(f"  (뷰포트 카메라 이동 실패: {e})", flush=True)

# ── FixedJoint 로 붙이면 그리퍼는 **팔의 관절체에 흡수된다** ──────
# 그래서 /World/Gripper 를 별도 Articulation 으로 잡으려 하면 실패한다
# (AttributeError: 'NoneType' object has no attribute 'is_homogeneous').
# 팔의 뷰 하나로 팔 관절 + 그리퍼 관절을 함께 제어해야 한다.
all_names = list(arm.dof_names)
print(f"\n  부착 후 팔 관절체의 DOF {len(all_names)}개:", flush=True)
for i, nm in enumerate(all_names):
    tag = "  ← 그리퍼" if ("finger" in nm or "knuckle" in nm) else ""
    print(f"    [{i:>2}] {nm}{tag}", flush=True)

ARM_IDX = np.array([i for i, n in enumerate(all_names)
                    if not ("finger" in n or "knuckle" in n)])
DRIVE = np.array([all_names.index("finger_joint")])
arm_pose = POSE[:len(ARM_IDX)][None, :]

print(f"\n  {'step':>6}{'그리퍼 z':>11}{'플랜지 z':>11}{'상대거리':>11}", flush=True)
steps = 2400 if GUI else 400
d_list = []
for s in range(steps):
    arm.apply_action(ArticulationActions(joint_positions=arm_pose,
                                         joint_indices=ARM_IDX))
    ratio = 0.5 - 0.5 * math.cos(s / 90.0)          # 0 ↔ 1 왕복
    arm.apply_action(ArticulationActions(
        joint_positions=np.array([[ratio * 0.8]]), joint_indices=DRIVE))
    world.step(render=GUI or bool(OUT))
    if s % 100 == 0:
        g = np.asarray(grip.get_world_poses()[0])[0].astype(float)
        f = np.asarray(flange.get_world_poses()[0])[0].astype(float)
        d = float(np.linalg.norm(g - f))
        d_list.append(d)
        if s % 400 == 0:
            print(f"  {s:>6}{g[2]:>11.4f}{f[2]:>11.4f}{d:>11.4f}", flush=True)

drift = (max(d_list) - min(d_list)) * 1000 if d_list else float("nan")
print(f"\n  그리퍼-플랜지 상대거리 변동 {drift:.2f} mm "
      f"(유한값 {all(np.isfinite(d) for d in d_list)})", flush=True)
print(f"  판정: {'✅ 여닫는 동안에도 단단히 붙어 있다' if drift < 5 else '❌ 부착 실패'}",
      flush=True)

if OUT and cam is not None:
    d = cam.get_data("rgb")[0]
    if d is not None:
        img = d.numpy()
        save_png(f"{OUT}/ch06_hardware_rig.png", img)
        print(f"\n  저장: ch06_hardware_rig.png (평균 밝기 {img.mean():.0f})", flush=True)

print("===RIG_END===", flush=True)
app.close()
