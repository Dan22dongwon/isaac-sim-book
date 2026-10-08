"""6장 실습 결과를 **화면으로 직접 보는** 데모.

한 씬에 6장에서 검증한 것을 모아 놓고 계속 움직인다.
  왼쪽  — UR5e + Robotiq 2F-85 리그 (ERR-036 수정 적용). 그리퍼가 여닫힌다.
  오른쪽 — 케이블 체인 (ERR-033 수정 적용). 옆으로 밀면 흔들린다.

마우스로 시점을 돌려 보면 된다.
  좌클릭 드래그 = 회전 | 가운데 드래그 = 이동 | 휠 = 확대·축소

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    python demo_ch06_gui.py            # 기본 5분
    MINUTES=10 python demo_ch06_gui.py
"""
import math
import os
import struct
import zlib

import numpy as np
from isaacsim import SimulationApp

# OUT_DIR 을 주면 헤드리스로 돌며 책에 넣을 그림만 저장한다
OUT = os.environ.get("OUT_DIR", "")
app = SimulationApp({"headless": bool(OUT)})

import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation, RigidPrim  # noqa: E402
from isaacsim.core.utils.types import ArticulationActions  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import omni.usd  # noqa: E402

MINUTES = float(os.environ.get("MINUTES", "5"))

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
nucleus = nucleus_utils.get_assets_root_path()

# ── 왼쪽: UR5e + 그리퍼 리그 ──────────────────────────────────────
stage_utils.add_reference_to_stage(
    usd_path=nucleus + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    prim_path="/World/UR5e")
stage_utils.add_reference_to_stage(
    usd_path=nucleus + "/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd",
    prim_path="/World/Gripper")

base_fix = UsdPhysics.FixedJoint.Define(stage, "/World/base_fix")
base_fix.CreateBody1Rel().SetTargets(["/World/UR5e/base_link"])
base_fix.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

# [ERR-036] 관절체는 하나여야 한다 — 그리퍼의 ArticulationRoot 를 뗀다
grip_root = stage.GetPrimAtPath("/World/Gripper/Robotiq_2F_85")
grip_root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
if grip_root.HasAPI(PhysxSchema.PhysxArticulationAPI):
    grip_root.RemoveAPI(PhysxSchema.PhysxArticulationAPI)


# ── 오른쪽: 케이블 체인 [ERR-033] ─────────────────────────────────
def create_cable_chain(stage, base_path, n=15, seg_len=0.03, radius=0.006,
                       start_pos=(0.0, 0.9, 0.95)):
    stage.DefinePrim(base_path, "Xform")
    prev = None
    for i in range(n):
        seg = f"{base_path}/segment_{i:02d}"
        cap = UsdGeom.Capsule.Define(stage, seg)
        cap.GetRadiusAttr().Set(radius)
        cap.GetHeightAttr().Set(seg_len)
        cap.GetAxisAttr().Set("Z")
        UsdGeom.Xformable(cap.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(start_pos[0], start_pos[1], start_pos[2] - i * seg_len))
        UsdPhysics.RigidBodyAPI.Apply(cap.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
        UsdPhysics.MassAPI.Apply(cap.GetPrim()).CreateMassAttr().Set(0.02)
        if prev is not None:
            j = UsdPhysics.Joint.Define(stage, f"{base_path}/j_{i:02d}")
            j.CreateBody0Rel().SetTargets([prev])
            j.CreateBody1Rel().SetTargets([seg])
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, -seg_len / 2))
            j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, seg_len / 2))
            # [ERR-033-b] 병진축을 잠가야 늘어나지 않는다 (low > high = 잠금)
            for ax in ("transX", "transY", "transZ"):
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), ax)
                lim.CreateLowAttr().Set(1.0)
                lim.CreateHighAttr().Set(-1.0)
        prev = seg
    fj = UsdPhysics.FixedJoint.Define(stage, f"{base_path}/anchor_joint")
    fj.CreateBody1Rel().SetTargets([f"{base_path}/segment_00"])
    # [ERR-033-a] 이 줄이 없으면 케이블이 월드 원점으로 끌려간다
    fj.CreateLocalPos0Attr().Set(Gf.Vec3f(*start_pos))
    fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    return n


CABLE_XY = (0.0, 0.90)          # 케이블이 매달리는 자리
POST_XY = (0.0, 1.10)           # 기둥은 20 cm 옆에 세운다

N_CABLE = create_cable_chain(stage, "/World/Cable",
                             start_pos=(CABLE_XY[0], CABLE_XY[1], 0.95))

# ── 케이블을 매다는 거치대 (기둥 + 가로대) ───────────────────────
# 주의: 기둥과 케이블을 **같은 좌표에 두면 안 된다.**
# 겹치면 물리 엔진이 매 스텝 서로 밀어내서, 케이블이 흔들리는 게 아니라
# 충돌로 튀어 다니는 것처럼 보인다.
post = UsdGeom.Cylinder.Define(stage, "/World/Rig/Post")
post.GetRadiusAttr().Set(0.02)
post.GetHeightAttr().Set(1.0)
post.GetAxisAttr().Set("Z")
UsdGeom.XformCommonAPI(post.GetPrim()).SetTranslate(
    Gf.Vec3d(POST_XY[0], POST_XY[1], 0.5))
UsdPhysics.CollisionAPI.Apply(post.GetPrim())

# 기둥 꼭대기에서 케이블 자리까지 뻗는 가로대
beam = UsdGeom.Cylinder.Define(stage, "/World/Rig/Beam")
beam.GetRadiusAttr().Set(0.015)
beam.GetHeightAttr().Set(abs(POST_XY[1] - CABLE_XY[1]))
beam.GetAxisAttr().Set("Y")
UsdGeom.XformCommonAPI(beam.GetPrim()).SetTranslate(
    Gf.Vec3d(0.0, (POST_XY[1] + CABLE_XY[1]) / 2.0, 0.98))

gap = abs(POST_XY[1] - CABLE_XY[1]) - 0.02 - 0.006
print(f"\n  기둥-케이블 간격 {gap*100:.1f} cm (겹치지 않는다)", flush=True)

# ── 조명 ──────────────────────────────────────────────────────────
key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(14000.0)
key.CreateWidthAttr().Set(6.0)
key.CreateHeightAttr().Set(6.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(1.2, -1.6, 2.8))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(1000.0)

world.reset()

POSE = np.array([0.4, -1.1, 1.3, -1.75, -1.57, 0.0])

# 1단계 — 플랜지 위치를 읽는다 (뷰를 scene 에 등록하지 않는다)
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

# 2단계 — 그리퍼를 플랜지 자리로
grip0 = RigidPrim(prim_paths_expr="/World/Gripper/Robotiq_2F_85/base_link",
                  name="grip_tmp")
grip0.initialize()
grip0.set_world_poses(positions=np.array([fpos + np.array([0.0, 0.0, 0.02])]),
                      orientations=np.array([fquat]))
world.step(render=False)

# 3단계 — FixedJoint 로 진짜 부착 [ERR-036]
fj = UsdPhysics.FixedJoint.Define(stage, "/World/gripper_attach")
fj.CreateBody0Rel().SetTargets(["/World/UR5e/wrist_3_link"])
fj.CreateBody1Rel().SetTargets(["/World/Gripper/Robotiq_2F_85/base_link"])
fj.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.02))
fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

# 4단계 — 토폴로지가 바뀌었으니 뷰를 새로 만든다
world.reset()
arm = Articulation(prim_paths_expr="/World/UR5e", name="ur5e")
arm.initialize()
names = list(arm.dof_names)
ARM_IDX = np.array([i for i, n in enumerate(names)
                    if not ("finger" in n or "knuckle" in n)])
DRIVE = np.array([names.index("finger_joint")])
arm.set_joint_positions(np.concatenate([POSE, np.zeros(len(names) - 6)])[None, :])
arm.set_joint_velocities(np.zeros((1, len(names))))
for _ in range(10):
    world.step(render=False)

# 케이블 꼬리를 흔들 수 있게 잡아 둔다
tail = RigidPrim(prim_paths_expr=f"/World/Cable/segment_{N_CABLE-1:02d}", name="tail")
tail.initialize()

EYE, TGT = [1.9, -1.5, 1.35], [0.25, 0.45, 0.55]

# 보기 좋은 시점으로 뷰포트 카메라를 옮긴다
if not OUT:
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=EYE, target=TGT)
    except Exception as e:
        print(f"(뷰포트 이동 실패: {e})", flush=True)


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
    os.makedirs(OUT, exist_ok=True)
    import isaacsim.core.experimental.utils.app as app_utils
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
    rtx = RtxCamera("/World/BookCam", tick_rate=30.0,
                    translations=[EYE], orientations=[look_at_quat(EYE, TGT)])
    cam = CameraSensor(rtx, resolution=(1400, 900), annotators=["rgb"])
    app_utils.play(commit=True)
    for _ in range(40):
        world.step(render=True)

print("\n" + "=" * 62, flush=True)
print("  6장 데모 — 창을 보세요", flush=True)
print("=" * 62, flush=True)
print(f"  왼쪽  UR5e + Robotiq 2F-85   그리퍼가 천천히 여닫힙니다", flush=True)
print(f"        팔 관절 {len(ARM_IDX)}개 + 그리퍼 관절 {len(names)-len(ARM_IDX)}개 "
      f"= 한 관절체 {len(names)}개 [ERR-036]", flush=True)
print(f"  오른쪽 케이블 {N_CABLE}마디   주기적으로 옆에서 밀어 흔듭니다 [ERR-033]", flush=True)
print(f"\n  마우스 좌클릭 드래그=회전  휠=확대  가운데드래그=이동", flush=True)
print(f"  {MINUTES:.0f}분 뒤 자동 종료합니다. 먼저 닫으려면 이 터미널에서 Ctrl+C.",
      flush=True)
print("=" * 62 + "\n", flush=True)

arm_pose = POSE[None, :]
steps = 420 if OUT else int(MINUTES * 60 * 60)
SHOTS = {150: "ch06_demo_open", 260: "ch06_demo_closed", 400: "ch06_demo_swing"}
try:
    for s in range(steps):
        arm.apply_action(ArticulationActions(joint_positions=arm_pose,
                                             joint_indices=ARM_IDX))
        ratio = 0.5 - 0.5 * math.cos(s / 110.0)       # 그리퍼 여닫기
        arm.apply_action(ArticulationActions(
            joint_positions=np.array([[ratio * 0.8]]), joint_indices=DRIVE))
        if (s % 420 == 0) or (OUT and s == 360):      # 7초마다 케이블을 툭 친다
            # 기둥 반대쪽(-y)으로 민다. +y 로 밀면 기둥에 부딪힌다.
            tail.set_velocities(np.array([[1.4, -0.6, 0.0, 0.0, 0.0, 0.0]]))
        world.step(render=True)

        if OUT and s in SHOTS and cam is not None:
            d = cam.get_data("rgb")[0]
            if d is not None:
                img = d.numpy()
                save_png(f"{OUT}/{SHOTS[s]}.png", img)
                print(f"  저장: {SHOTS[s]}.png (평균 밝기 {img.mean():.0f})", flush=True)

        if (not OUT) and s % 1800 == 0:
            top = float(np.asarray(
                RigidPrim(prim_paths_expr="/World/Cable/segment_00",
                          name=f"t{s}").get_world_poses()[0])[0][2]) \
                if s == 0 else None
            span = float(np.linalg.norm(
                np.asarray(tail.get_world_poses()[0])[0]
                - np.array([CABLE_XY[0], CABLE_XY[1], 0.95])))
            print(f"  ... {s/3600:.1f}분 경과   케이블 길이 {span:.3f} m "
                  f"(이상값 {(N_CABLE-1)*0.03:.3f} m)", flush=True)
except KeyboardInterrupt:
    print("\n  사용자가 중단했습니다.", flush=True)

print("\n  데모 종료", flush=True)
app.close()
