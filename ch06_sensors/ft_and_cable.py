"""책 6.10 [실습] F/T 근사 센서 + 케이블 파지 통합 씬 — 동작 검증본.

원문과의 차이:
  [ERR-009] ArticulationView 는 Isaac Sim 에서 삭제됨
            → isaacsim.core.prims.Articulation (prim_paths_expr 인자 유지)
  [ERR-022] UR5e 경로는 원문 그대로 유효하다 (수정 불필요)

검증 내용
  ① 강체 캡슐 체인으로 케이블 근사 — D6 조인트로 연결
  ② 관절 반력(get_measured_joint_forces)으로 F/T 근사
  ③ 케이블이 중력에 늘어지는지 실측

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python ft_and_cable.py
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
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "")
if OUT:
    os.makedirs(OUT, exist_ok=True)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
nucleus = nucleus_utils.get_assets_root_path()
stage = omni.usd.get_context().get_stage()


# ── ① 케이블 = 캡슐 체인 ──────────────────────────────────────────
def create_cable_chain(stage, base_path, n_segments=15, segment_length=0.03,
                       radius=0.005, start_pos=(0.0, 0.0, 1.0)):
    """짧은 캡슐을 D6 조인트로 이어 케이블을 근사한다."""
    stage.DefinePrim(base_path, "Xform")
    prev = None
    for i in range(n_segments):
        seg = f"{base_path}/segment_{i:02d}"
        capsule = UsdGeom.Capsule.Define(stage, seg)
        capsule.GetRadiusAttr().Set(radius)
        capsule.GetHeightAttr().Set(segment_length)
        z = start_pos[2] - i * segment_length
        # 새로 만든 prim 이므로 AddTranslateOp 사용 가능 (참조 에셋이었다면 불가)
        UsdGeom.Xformable(capsule.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(start_pos[0], start_pos[1], z))
        UsdPhysics.RigidBodyAPI.Apply(capsule.GetPrim())
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
        UsdPhysics.MassAPI.Apply(capsule.GetPrim()).CreateMassAttr().Set(0.02)

        if prev is not None:
            jp = f"{base_path}/joint_{i:02d}"
            j = UsdPhysics.Joint.Define(stage, jp)
            j.CreateBody0Rel().SetTargets([prev])
            j.CreateBody1Rel().SetTargets([seg])
            # 두 캡슐이 맞닿는 지점에서 연결한다. 생략하면 몸체 원점끼리 붙어
            # 세그먼트가 겹친 채 시작한다.
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, -segment_length / 2))
            j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, segment_length / 2))
            # [ERR-033-b] 일반 UsdPhysics.Joint 는 6축이 전부 자유다.
            # 병진축을 잠그지 않으면 케이블이 중력에 끝없이 늘어난다
            # (실측: 0.42 m 체인이 0.888 m 로 신장).
            # low > high 로 두면 해당 축이 완전히 잠긴다.
            for ax in ("transX", "transY", "transZ"):
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), ax)
                lim.CreateLowAttr().Set(1.0)
                lim.CreateHighAttr().Set(-1.0)
        prev = seg

    # 맨 위 세그먼트를 공중에 고정
    fj = UsdPhysics.FixedJoint.Define(stage, f"{base_path}/anchor_joint")
    fj.CreateBody1Rel().SetTargets([f"{base_path}/segment_00"])
    # [ERR-033-a] body0 를 비우면 '월드'에 붙는데, localPos0 기본값이 (0,0,0)
    # 이므로 맨 위 세그먼트가 **월드 원점으로 끌려간다**.
    # 매달고 싶은 좌표를 localPos0 에 직접 지정해야 한다.
    fj.CreateLocalPos0Attr().Set(Gf.Vec3f(*start_pos))
    fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    return n_segments


N = create_cable_chain(stage, "/World/Cable", n_segments=15,
                       start_pos=(0.35, 0.0, 0.9))

# ── ② UR5e (F/T 근사 대상) ────────────────────────────────────────
stage_utils.add_reference_to_stage(
    usd_path=nucleus + "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    prim_path="/World/UR5e")

key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr().Set(14000.0)
key.CreateWidthAttr().Set(6.0)
key.CreateHeightAttr().Set(6.0)
UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(0.2, -1.0, 2.5))
UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(800.0)

world.reset()

# [ERR-009] ArticulationView → Articulation (prim_paths_expr 유지)
view = Articulation(prim_paths_expr="/World/UR5e", name="ur5e_view")
world.scene.add(view)
world.reset()

print("\n===FT_CABLE===", flush=True)
print(f"케이블 세그먼트 {N}개 (캡슐 + D6 조인트)", flush=True)
print(f"UR5e 관절 {len(view.dof_names)}개", flush=True)


def seg_z(i):
    p = stage.GetPrimAtPath(f"/World/Cable/segment_{i:02d}")
    c = UsdGeom.XformCache()
    return float(c.GetLocalToWorldTransform(p).ExtractTranslation()[2])


print(f"\n{'step':>6}{'케이블 최하단 z':>16}{'F/T shape':>14}{'최대 |F|':>11}", flush=True)
for step in range(301):
    world.step(render=False)
    if step % 100 == 0:
        f = np.asarray(view.get_measured_joint_forces())
        bottom = seg_z(N - 1)
        print(f"{step:>6}{bottom:>16.3f}{str(f.shape):>14}{np.abs(f).max():>11.2f}", flush=True)

f = np.asarray(view.get_measured_joint_forces())
print(f"\n  get_measured_joint_forces() shape = {f.shape}", flush=True)
print(f"    (로봇 수, 관절 수, 6) — 6은 힘 3축 + 토크 3축", flush=True)
for i, name in enumerate(view.dof_names):
    print(f"    [{i}] {name:<24} |F| {np.abs(f[0, i, :3]).max():7.2f} N"
          f"   |T| {np.abs(f[0, i, 3:]).max():7.2f} Nm", flush=True)

# 앵커가 제자리를 지키는지 / 케이블이 늘어나지 않았는지 검증
top, bot = seg_z(0), seg_z(N - 1)
span = top - bot
ideal = (N - 1) * 0.03
print(f"\n  앵커 z = {top:.3f} (목표 0.900)   최하단 z = {bot:.3f}", flush=True)
print(f"  체인 길이 = {span:.3f} m (이상값 {ideal:.3f} m, "
      f"오차 {abs(span - ideal) * 1000:.0f} mm)", flush=True)


# ── ③ 스윙 시험 — 병진축을 잠갔을 때 회전축은 살아 있는가 ──────────
# 케이블이 굽지 못하면 그건 케이블이 아니라 막대다.
def seg_x(i):
    p = stage.GetPrimAtPath(f"/World/Cable/segment_{i:02d}")
    c = UsdGeom.XformCache()
    return float(c.GetLocalToWorldTransform(p).ExtractTranslation()[0])


from isaacsim.core.prims import RigidPrim  # noqa: E402

x0 = seg_x(N - 1)
tail = RigidPrim(prim_paths_expr=f"/World/Cable/segment_{N-1:02d}", name="cable_tail")
tail.initialize()
tail.set_velocities(np.array([[1.2, 0.0, 0.0, 0.0, 0.0, 0.0]]))  # 옆으로 툭 친다

swing = []
for step in range(120):
    world.step(render=False)
    swing.append(seg_x(N - 1))

amp = max(swing) - min(swing)
print(f"\n  [스윙 시험] 꼬리를 옆으로 1.2 m/s 로 친 뒤 120 스텝", flush=True)
print(f"    x 변화폭 {amp * 1000:.0f} mm   (최종 x {swing[-1]:.3f}, 시작 {x0:.3f})", flush=True)
print(f"    → {'회전축이 살아 있다 (케이블처럼 굽는다)' if amp > 0.01 else '❌ 굽지 않는다 = 강체 막대'}",
      flush=True)

if OUT:
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

    TGT = (0.35, 0.0, 0.62)
    EYE = (TGT[0] + 0.55, TGT[1] - 0.95, TGT[2] + 0.30)
    rtx = RtxCamera("/World/Cam", tick_rate=30.0,
                    translations=[list(EYE)], orientations=[look_at_quat(EYE, TGT)])
    cam = CameraSensor(rtx, resolution=(900, 700), annotators=["rgb"])
    app_utils.play(commit=True)
    for _ in range(40):
        world.step(render=True)
    d = cam.get_data("rgb")[0]
    if d is not None:
        img = d.numpy()
        h, w = img.shape[:2]
        a = np.ascontiguousarray(img[:, :, :3].astype(np.uint8))
        raw = b"".join(b"\x00" + a[y].tobytes() for y in range(h))

        def chunk(tag, data):
            return (struct.pack(">I", len(data)) + tag + data
                    + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

        open(f"{OUT}/ch06_ft_cable.png", "wb").write(
            b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
        print(f"  저장: ch06_ft_cable.png (평균 밝기 {img.mean():.0f})", flush=True)

print("===FT_CABLE_END===", flush=True)
app.close()
