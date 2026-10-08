"""책 7.1 [실습] 솔버 반복 횟수가 시뮬레이션 품질을 어떻게 바꾸는가.

원문과의 차이:
  [ERR-037] 원문의 솔버 속성 이름이 존재하지 않는다.
      physx_api.CreateMinIterationCountAttr().Set(8)     ← 그런 속성 없음
      physx_api.CreateMaxIterationCountAttr().Set(64)    ← 그런 속성 없음
    실제로는 위치(Position)와 속도(Velocity)가 분리돼 있다.
      MinPositionIterationCount / MaxPositionIterationCount
      MinVelocityIterationCount / MaxVelocityIterationCount
  [ERR-038] PhysicsScene 경로가 '/World/physicsScene' 이 아니라 '/physicsScene' 이다.
            World() 가 만드는 위치가 다르다. 경로를 하드코딩하지 말고 찾아야 한다.

무엇을 보여 주는가
  상자 6개를 쌓고 반복 횟수만 바꾼다. 반복이 모자라면 상자들이 서로 파고들어
  **탑이 통째로 가라앉는다.** 설정이 말뿐이 아니라는 것을 숫자와 그림으로 본다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python solver_iteration_demo.py
"""
import math
import os
import struct
import zlib

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": os.environ.get("HEADLESS", "1") == "1"})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import RigidPrim  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "")
if OUT:
    os.makedirs(OUT, exist_ok=True)

N_BOX = 6
SIZE = 0.20          # 상자 한 변 (m)
MASS = 5.0           # 상자 하나 무게 (kg)
STEPS = 200
IDEAL = np.array([SIZE / 2 + i * SIZE for i in range(N_BOX)])

# (설명, 위치반복, 속도반복)
CASES = [("반복 1회", 1, 1),
         ("반복 4회", 4, 1),
         ("반복 32회", 32, 4)]


def find_physics_scene(stage):
    """[ERR-038] 경로를 하드코딩하지 않는다. World 가 만든 씬을 찾아 쓴다."""
    for p in stage.Traverse():
        if p.GetTypeName() == "PhysicsScene":
            return p
    raise RuntimeError("PhysicsScene 을 찾지 못했다")


def set_solver(stage, pos_iter, vel_iter):
    api = PhysxSchema.PhysxSceneAPI.Apply(find_physics_scene(stage))
    api.CreateSolverTypeAttr().Set("TGS")
    # [ERR-037] Position / Velocity 로 나뉜 올바른 이름
    api.CreateMinPositionIterationCountAttr().Set(pos_iter)
    api.CreateMaxPositionIterationCountAttr().Set(pos_iter)
    api.CreateMinVelocityIterationCountAttr().Set(vel_iter)
    api.CreateMaxVelocityIterationCountAttr().Set(vel_iter)
    # 반복 횟수의 효과만 보려고 접촉 안정화는 끈다.
    # (켜 두면 PhysX 가 부족한 반복을 보정해 차이가 가려진다)
    api.CreateEnableStabilizationAttr().Set(False)


def build_tower(stage, base, x):
    stage.DefinePrim(base, "Xform")
    for i in range(N_BOX):
        p = f"{base}/box_{i:02d}"
        c = UsdGeom.Cube.Define(stage, p)
        c.GetSizeAttr().Set(SIZE)
        UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(x, 0.0, SIZE / 2 + i * SIZE))
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr().Set(MASS)


def add_lights(stage):
    key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr().Set(15000.0)
    key.CreateWidthAttr().Set(8.0)
    key.CreateHeightAttr().Set(8.0)
    UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(1.5, -2.5, 4.0))
    UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(1000.0)


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


print("\n===SOLVERDEMO===", flush=True)
print(f"상자 {N_BOX}개 (각 {MASS:.0f}kg, {SIZE*100:.0f}cm) 를 쌓고 {STEPS}스텝.", flush=True)
print("반복 횟수만 바꾼다. 나머지 조건은 전부 같다.\n", flush=True)
print(f"{'설정':<14}{'최대 침하':>12}{'꼭대기 높이':>13}{'이상값':>10}{'판정':>8}", flush=True)
print("-" * 60, flush=True)

rows = []
for label, pi, vi in CASES:
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    set_solver(stage, pi, vi)
    build_tower(stage, "/World/Tower", 0.0)
    add_lights(stage)
    world.reset()

    views = []
    for i in range(N_BOX):
        v = RigidPrim(prim_paths_expr=f"/World/Tower/box_{i:02d}",
                      name=f"{label}_{i}".replace(" ", ""))
        v.initialize()
        views.append(v)

    cam = None
    if OUT:
        # 탑 전체(1.2 m)가 프레임에 들어오도록 넉넉히 물러난다.
        # 가까이 붙으면 반복 32회 그림에서 탑이 잘려 비교가 안 된다.
        TGT = (0.0, 0.0, N_BOX * SIZE / 2)
        EYE = (3.6, -4.4, N_BOX * SIZE * 1.1)
        rtx = RtxCamera(f"/World/Cam_{pi}", tick_rate=30.0,
                        translations=[list(EYE)],
                        orientations=[look_at_quat(EYE, TGT)])
        cam = CameraSensor(rtx, resolution=(700, 800), annotators=["rgb"])
        app_utils.play(commit=True)
        for _ in range(40):
            world.step(render=True)

    for _ in range(STEPS):
        world.step(render=bool(OUT))

    h = np.array([float(np.asarray(v.get_world_poses()[0])[0][2]) for v in views])
    sink = float(np.abs(IDEAL - h).max())
    ok = sink < 0.01
    print(f"{label:<14}{sink*1000:>10.2f}mm{h[-1]:>12.3f}m{IDEAL[-1]:>9.3f}m"
          f"{'✅' if ok else '❌':>8}", flush=True)
    rows.append((label, sink, h[-1]))

    if OUT and cam is not None:
        d = cam.get_data("rgb")[0]
        if d is not None:
            save_png(f"{OUT}/ch07_solver_iter{pi:02d}.png", d.numpy())

    world.clear()

worst, best = rows[0], rows[-1]
print(f"\n  반복 {CASES[0][1]}회 → {CASES[-1][1]}회", flush=True)
print(f"    최대 침하 {worst[1]*1000:.2f} mm → {best[1]*1000:.2f} mm", flush=True)
print(f"    꼭대기   {worst[2]:.3f} m → {best[2]:.3f} m (이상값 {IDEAL[-1]:.3f} m)", flush=True)
print(f"\n  {worst[0]} 의 침하 {worst[1]*1000:.0f} mm 는 탑 전체 높이"
      f" {N_BOX*SIZE*1000:.0f} mm 와 맞먹는다 — 탑이 통째로 무너졌다는 뜻이다.", flush=True)
if OUT:
    print(f"\n  저장: ch07_solver_iter01/04/32.png", flush=True)
print("===SOLVERDEMO_END===", flush=True)
app.close()
