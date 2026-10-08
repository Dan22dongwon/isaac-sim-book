"""책 9.1 (b02) Occupancy Map 생성 — 동작 검증본.

Nav2 는 지도가 있어야 경로를 계획한다. 이 절이 그 지도를 만든다.

원문과의 차이 — [ERR-056] **이 블록의 API 가 통째로 다르다.**

  원고                                    실제
  ────────────────────────────────────────────────────────────────
  acquire_occupancy_map_interface()   →   _omap.Generator(physx, stage_id)
  set_cell_size(0.05)                 →   update_settings(cell_size, ...)
  set_occupied_below_threshold(0.1)   →   ❌ 없음
  set_occupied_above_threshold(3.0)   →   ❌ 없음
  set_min_bound((...))                →   set_transform(origin, min, max) 하나로
  set_max_bound((...))                →   (위와 동일)
  generate2d()                        →   generate2d()  ← 이건 맞다

  게다가 확장을 **먼저 켜지 않으면 모듈 자체가 안 보인다.**
    ModuleNotFoundError: No module named 'isaacsim.asset.gen'

올바른 사용법은 확장의 자체 테스트(`isaacsim/asset/gen/omap/tests/test_occupancy.py`)에
있다. 문서를 찾기보다 그쪽을 보는 편이 빠르고 정확하다.

검증 방법: 파일이 생긴 것으로 끝내지 않는다. **정답을 아는 방**(10m×8m, 기둥 2개)을
만들고, 지도의 점유 비율이 기하 계산과 맞는지 본다. 전부 빈 칸이어도 파일은 생긴다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python occupancy_map.py
"""
import os

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

# [ERR-056] 확장을 켜야 바인딩 모듈이 보인다
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402

app_utils.enable_extension("isaacsim.asset.gen.omap")
app.update()

import omni.physx  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

import omni.usd  # noqa: E402

OUT = os.environ.get("OUT_DIR", ".")
os.makedirs(OUT, exist_ok=True)

World.clear_instance()
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

print("\n===OMAP===", flush=True)

# ── 정답을 아는 방 ───────────────────────────────────────────────
ROOM_X, ROOM_Y, WALL_T, WALL_H = 10.0, 8.0, 0.2, 2.0
PILLAR = 0.6
stage.DefinePrim("/World/Room", "Xform")


def box(name, cx, cy, sx, sy):
    c = UsdGeom.Cube.Define(stage, f"/World/Room/{name}")
    c.GetSizeAttr().Set(1.0)
    api = UsdGeom.XformCommonAPI(c.GetPrim())
    api.SetTranslate(Gf.Vec3d(cx, cy, WALL_H / 2))
    api.SetScale(Gf.Vec3f(sx, sy, WALL_H))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())


box("N", 0, ROOM_Y / 2, ROOM_X, WALL_T)
box("S", 0, -ROOM_Y / 2, ROOM_X, WALL_T)
box("E", ROOM_X / 2, 0, WALL_T, ROOM_Y)
box("W", -ROOM_X / 2, 0, WALL_T, ROOM_Y)
box("P1", -2.0, 1.0, PILLAR, PILLAR)
box("P2", 2.5, -1.5, PILLAR, PILLAR)

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr().Set(1000.0)
world.reset()
for _ in range(10):
    world.step(render=False)

print(f"방 {ROOM_X}×{ROOM_Y} m, 벽 두께 {WALL_T} m, 기둥 {PILLAR} m 짜리 2개", flush=True)

# ── [ERR-056] 올바른 API: Generator ──────────────────────────────
from isaacsim.asset.gen.omap.bindings import _omap  # noqa: E402

CELL = 0.05
MARGIN = 1.0
MINB = (-ROOM_X / 2 - MARGIN, -ROOM_Y / 2 - MARGIN, 0.1)
MAXB = (ROOM_X / 2 + MARGIN, ROOM_Y / 2 + MARGIN, WALL_H - 0.1)

gen = _omap.Generator(omni.physx.get_physx_interface(),
                      omni.usd.get_context().get_stage_id())
# update_settings(cell_size, 점유값, 비점유값, 미탐색값)
gen.update_settings(CELL, 0, 255, 128)
# set_transform(원점, 최소경계, 최대경계) — 원고의 set_min/max_bound 를 대신한다
gen.set_transform((0, 0, 0), MINB, MAXB)
gen.generate2d()

occupied_pos = gen.get_occupied_positions()
free_pos = gen.get_free_positions()
dims = gen.get_dimensions()
buf = gen.get_buffer()

print(f"\n지도 {dims}  셀 {CELL} m", flush=True)
print(f"점유 셀 {len(occupied_pos):,}개 / 빈 셀 {len(free_pos):,}개", flush=True)

# ── 검증: 기하 계산과 맞는가 ─────────────────────────────────────
area_map = (MAXB[0] - MINB[0]) * (MAXB[1] - MINB[1])
area_wall = 2 * (ROOM_X + ROOM_Y) * WALL_T + 2 * PILLAR ** 2
ratio = len(occupied_pos) * CELL ** 2 / area_map * 100
expect = area_wall / area_map * 100

print(f"\n점유 면적 {len(occupied_pos) * CELL**2:.2f} m²  "
      f"(지도 전체 {area_map:.1f} m²) → {ratio:.1f}%", flush=True)
print(f"기하 계산상 벽+기둥 면적 {area_wall:.2f} m² → {expect:.1f}%", flush=True)

ok = len(occupied_pos) > 0 and abs(ratio - expect) < 3.0
print(f"판정: {'✅ 벽이 지도에 제대로 찍혔다' if ok else '❌ 지도가 비었거나 어긋난다'}",
      flush=True)

# ── 저장 (ROS2 map_server 규격) ──────────────────────────────────
img = np.array(buf, dtype=np.uint8).reshape(dims[1], dims[0])

# 원고는 cv2 를 쓰지만 PGM 은 규격이 단순해 표준 라이브러리로 충분하다.
pgm = os.path.join(OUT, "warehouse_map.pgm")
with open(pgm, "wb") as f:
    f.write(b"P5\n%d %d\n255\n" % (dims[0], dims[1]))
    f.write(np.ascontiguousarray(np.flipud(img)).tobytes())   # PGM 은 위→아래

yml = os.path.join(OUT, "warehouse_map.yaml")
with open(yml, "w") as f:
    f.write(f"""image: warehouse_map.pgm
resolution: {CELL}
origin: [{MINB[0]}, {MINB[1]}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")

print(f"\n저장: warehouse_map.pgm ({os.path.getsize(pgm)/1024:.0f} KB) + .yaml", flush=True)
print("===OMAP_END===", flush=True)
app.close()
