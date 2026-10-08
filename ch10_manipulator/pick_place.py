"""10.5 b09 — Franka Pick & Place 종단간 (원고 수정본)

원고 대비 고친 곳
------------------------------------------------------------------
  [ERR-072] `isaacsim.robot.manipulators.examples` 확장이 **기본으로 꺼져 있다.**
            import 하기 **전에** enable_extension 을 불러야 한다.
            안 그러면 ModuleNotFoundError 로 첫 줄부터 막힌다.
            (Ch7 ERR-056 의 Occupancy Map 과 같은 함정)

  [ERR-074] 원고는 `world.get_observations()` 를 부르고 결과를 쓰지 않는다.
            World 에 Task 를 등록하지 않으면 이 호출은 의미가 없다.
            지웠다.

실행:
    conda activate isaac_env
    python pick_place.py            # GUI
    HEADLESS=1 python pick_place.py # 화면 없이
"""
import os

from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
app = SimulationApp({"headless": HEADLESS})

# ── [ERR-072] import 보다 먼저 확장을 켠다 ───────────────────────
from isaacsim.core.utils.extensions import enable_extension   # noqa: E402

enable_extension("isaacsim.robot.manipulators")
enable_extension("isaacsim.robot.manipulators.examples")

import numpy as np                                            # noqa: E402
from isaacsim.core.api import World                           # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid  # noqa: E402
from isaacsim.robot.manipulators.examples.franka import Franka    # noqa: E402
from isaacsim.robot.manipulators.examples.franka.controllers import (  # noqa: E402
    PickPlaceController)

# ── 씬 ───────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

TABLE_TOP = 0.8                       # 테이블 윗면 높이

world.scene.add(FixedCuboid(
    prim_path="/World/Table", name="table",
    position=np.array([0.5, 0.0, TABLE_TOP / 2]),
    scale=np.array([0.6, 1.0, TABLE_TOP]),
    color=np.array([0.6, 0.5, 0.4])))

BOX = 0.05
box = world.scene.add(DynamicCuboid(
    prim_path="/World/PickBox", name="pick_box",
    position=np.array([0.4, 0.1, TABLE_TOP + BOX / 2]),
    scale=np.array([BOX, BOX, BOX]),
    color=np.array([0.9, 0.3, 0.1]),
    mass=0.1))

franka = world.scene.add(Franka(
    prim_path="/World/Franka", name="franka",
    position=np.array([0.0, 0.0, TABLE_TOP])))

# ── [ERR-075] end_effector_initial_height 는 **월드 절대 높이**다 ──
# 기본값은 pick_place_controller.py 72행에서 0.3 m 로 박혀 있다.
#     self._h1 = 0.3 / get_stage_units()
# 원고는 로봇을 z=0.8 인 테이블 위에 올려 놓고 이 값을 주지 않는다.
# 그러면 0단계에서 손끝을 z=0.3 으로 내리려 하는데, 그곳은 테이블
# 윗면보다 0.5 m **아래**다. 팔이 테이블을 훑으며 박스를 쳐낸다.
# 실측: 박스가 (-0.106, -2.091, 0.025) 까지 날아가 바닥에 떨어졌다.
controller = PickPlaceController(
    name="pick_place_controller",
    gripper=franka.gripper,
    robot_articulation=franka,
    end_effector_initial_height=TABLE_TOP + 0.3)

world.reset()

# ── [ERR-076] world.reset() 직후 집게는 **닫혀 있다** ──────────────
# 원고는 이 줄이 없다. 그러면 닫힌 집게로 내려가 박스를 감싸지 못한다.
# 실측(계측 스크립트): 집게폭이 0단계부터 0.0000 이고, 박스는 단 1 mm 도
# 움직이지 않은 채 팔만 왕복한다. 이 줄을 넣으면 0.0852 로 벌어진다.
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

if not HEADLESS:
    from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
    set_camera_view(eye=np.array([1.8, 1.6, 1.6]),
                    target=np.array([0.4, 0.0, 0.85]))

PICK = np.array([0.4, 0.10, TABLE_TOP + BOX / 2])
PLACE = np.array([0.4, -0.25, TABLE_TOP + BOX / 2])

print(f"\nPick & Place 시작  집기 {PICK}  놓기 {PLACE}", flush=True)

for step in range(1500):
    action = controller.forward(
        picking_position=PICK,
        placing_position=PLACE,
        current_joint_positions=franka.get_joint_positions(),
        end_effector_offset=np.array([0.0, 0.0, 0.02]))
    franka.apply_action(action)
    world.step(render=True)

    if step % 150 == 0:
        p, _ = box.get_world_pose()
        print(f"  step {step:4d}  단계 {controller.get_current_event():2d}  "
              f"박스 ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})", flush=True)

    if controller.is_done():
        print(f"  컨트롤러 완료 — step {step}", flush=True)
        break

# ── 검증 ─────────────────────────────────────────────────────────
final, _ = box.get_world_pose()
err_xy = float(np.linalg.norm(final[:2] - PLACE[:2]))
fell = final[2] < TABLE_TOP - 0.05

print(f"\n최종 박스 위치 ({final[0]:+.3f}, {final[1]:+.3f}, {final[2]:+.3f})")
print(f"목표까지 거리   {err_xy*100:.1f} cm")
print(f"테이블에서 떨어졌나: {'예 ❌' if fell else '아니오 ✅'}")
print("✅ 성공" if (err_xy < 0.05 and not fell) else "❌ 실패", flush=True)

if not HEADLESS:
    hold = int(os.environ.get("HOLD", "1800"))     # 결과를 눈으로 볼 시간
    print(f"\n(창을 유지합니다 — {hold} 스텝)", flush=True)
    for _ in range(hold):
        world.step(render=True)

app.close()
