"""책 7.1 솔버 반복 효과 — **동영상**으로 기록한다.

정지 사진으로는 '무너지는 과정'이 안 보인다. 탑이 어떻게 파고들어 주저앉는지는
움직임이라 영상이어야 한다.

두 설정을 각각 렌더해 MP4 로 만든다.
  반복  1회 → 상자들이 서로 파고들며 무너진다
  반복 32회 → 곧게 선다

[ERR-037] 솔버 반복 속성은 Position/Velocity 로 나뉜다 (원고의 이름은 없다).
[ERR-038] PhysicsScene 경로를 하드코딩하지 않는다.
[주의]    씬 속성을 바꾼 뒤에는 **world.reset()** 을 거쳐야 PhysX 가 읽는다.
          속성만 바꾸면 반영되지 않는다 (실측: 0.100m vs 1.100m).

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python render_solver_video.py
"""
import math
import os
import struct
import subprocess
import zlib

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import RigidPrim  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", "./out")
os.makedirs(OUT, exist_ok=True)
FRAMES = os.path.join(OUT, "_frames")

DT = 1.0 / 60          # 차이가 가장 뚜렷한 타임스텝
N_BOX, SIZE, MASS = 6, 0.20, 5.0
IDEAL_TOP = SIZE / 2 + (N_BOX - 1) * SIZE
SECONDS = 4.0
STEPS = int(SECONDS / DT)
FPS = 30
EVERY = max(1, int(round((1 / DT) / FPS)))   # 몇 스텝마다 한 프레임을 뽑나
W, H = 900, 760


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


def render_case(pos_iter, tag):
    World.clear_instance()
    world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=DT)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()

    # [ERR-038] 경로를 찾아서 쓴다
    scene = next(p for p in stage.Traverse() if p.GetTypeName() == "PhysicsScene")
    api = PhysxSchema.PhysxSceneAPI.Apply(scene)
    api.CreateSolverTypeAttr().Set("TGS")
    # [ERR-037] Position / Velocity 로 나뉜 올바른 이름
    api.CreateMinPositionIterationCountAttr().Set(pos_iter)
    api.CreateMaxPositionIterationCountAttr().Set(pos_iter)
    api.CreateMinVelocityIterationCountAttr().Set(1)
    api.CreateMaxVelocityIterationCountAttr().Set(1)
    api.CreateEnableStabilizationAttr().Set(False)

    stage.DefinePrim("/World/Tower", "Xform")
    for i in range(N_BOX):
        c = UsdGeom.Cube.Define(stage, f"/World/Tower/box_{i:02d}")
        c.GetSizeAttr().Set(SIZE)
        UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, SIZE / 2 + i * SIZE))
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr().Set(MASS)

    key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr().Set(16000.0)
    key.CreateWidthAttr().Set(10.0)
    key.CreateHeightAttr().Set(10.0)
    UsdGeom.XformCommonAPI(key.GetPrim()).SetTranslate(Gf.Vec3d(1.5, -3.0, 4.5))
    UsdLux.DomeLight.Define(stage, "/World/Lights/Dome").CreateIntensityAttr().Set(1100.0)

    TGT = (0.0, 0.0, IDEAL_TOP / 2)
    EYE = (2.6, -3.4, IDEAL_TOP * 0.95)
    rtx = RtxCamera(f"/World/Cam_{pos_iter}", tick_rate=1.0 / DT,
                    translations=[list(EYE)], orientations=[look_at_quat(EYE, TGT)])
    cam = CameraSensor(rtx, resolution=(W, H), annotators=["rgb"])

    world.reset()
    top = RigidPrim(prim_paths_expr=f"/World/Tower/box_{N_BOX-1:02d}", name=f"top{tag}")
    top.initialize()

    app_utils.play(commit=True)
    for _ in range(40):            # 렌더 프로덕트 워밍업
        world.step(render=True)

    d = os.path.join(FRAMES, tag)
    os.makedirs(d, exist_ok=True)
    for f in os.listdir(d):
        os.remove(os.path.join(d, f))

    n = 0
    for s in range(STEPS):
        world.step(render=True)
        if s % EVERY == 0:
            img = cam.get_data("rgb")[0]
            if img is not None:
                save_png(os.path.join(d, f"f{n:04d}.png"), img.numpy())
                n += 1

    z = float(np.asarray(top.get_world_poses()[0])[0][2])
    world.clear()
    return n, z, d


def encode(src_dir, out_mp4):
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
           "-i", os.path.join(src_dir, "f%04d.png"),
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out_mp4]
    subprocess.run(cmd, check=True)
    return os.path.getsize(out_mp4)


print("\n===VIDEO===", flush=True)
print(f"상자 {N_BOX}개 (각 {MASS:.0f}kg), dt=1/{1/DT:.0f}s, {SECONDS:.0f}초 × {FPS}fps",
      flush=True)
print(f"정상 높이 {IDEAL_TOP:.2f}m\n", flush=True)

results = []
for pos_iter, tag in ((1, "iter01"), (32, "iter32")):
    n, z, d = render_case(pos_iter, tag)
    mp4 = os.path.join(OUT, f"ch07_solver_{tag}.mp4")
    size = encode(d, mp4)
    ok = abs(z - IDEAL_TOP) < 0.05
    results.append((pos_iter, z, ok, mp4, size, n))
    print(f"  반복 {pos_iter:>2}회  프레임 {n}장 → {os.path.basename(mp4)} "
          f"({size/1024:.0f} KB)   꼭대기 {z:.3f}m  {'✅ 서 있음' if ok else '❌ 무너짐'}",
          flush=True)

print(f"\n  반복 1회는 {results[0][1]:.3f}m, 32회는 {results[1][1]:.3f}m.", flush=True)
print(f"  같은 코드, 같은 무게, 같은 타임스텝 — 반복 횟수만 다르다.", flush=True)
print("===VIDEO_END===", flush=True)
app.close()
