"""10.3 b02 — IK 솔버로 목표 자세의 관절각 구하기 (원고 수정본)

원고는 `PinkIKSolver` 를 쓰는데 **Isaac Sim 6.1 에 그 클래스가 없다.**
있는 것은 셋뿐이다.

    LulaKinematicsSolver          ← 실제 IK 를 푸는 엔진
    ArticulationKinematicsSolver  ← 위를 특정 로봇/프레임에 붙이는 껍데기
    KinematicsSolver              ← 추상 기반 클래스

원고 대비 고친 곳
------------------------------------------------------------------
  [ERR-071] `PinkIKSolver` 없음 → `LulaKinematicsSolver` +
            `ArticulationKinematicsSolver` 조합으로 교체.
            `pink` 파이썬 패키지도 설치돼 있지 않다.

  [ERR-072] `isaacsim.robot.manipulators.examples` 확장을 import 앞에 켠다.

  [ERR-084] 원고는 `urdf_path="franka/franka.urdf"` 같은 **상대경로 문자열**을
            준다. 그런 파일명은 설치본에 없다(실제는 `lula_franka_gen.urdf`).
            설정 로더를 쓰면 절대경로가 자동으로 채워진다.

  [ERR-085] 원고의 `solve(...)` 는 `warm_start=` 인자를 받는다고 쓰는데,
            Lula 계열은 `warm_start` 라는 이름을 쓰지 않는다. 실제 API 는
            `compute_inverse_kinematics(target_position, target_orientation,
            position_tolerance, orientation_tolerance)` 이고, 시작 자세는
            **로봇의 현재 관절각에서 자동으로** 잡는다.

오류가 아니었던 것 — 정정
------------------------------------------------------------------
  처음에 원고의 `end_effector_frame_name="panda_hand"` 도 틀렸다고 적었다.
  **아니다.** 이 스크립트가 직접 찍어 보니 `panda_hand` 는 Lula 가 지원하는
  프레임 목록에 **있다**. ERR-063(`right_gripper`) 은 **RMPflow 설정 로더**가
  주는 값에 대한 이야기이고, 운동학 솔버에는 해당하지 않는다.
  두 프레임은 약 99 mm 떨어져 있으므로 **무엇을 제어하는지만 알고 쓰면 된다.**

실행:
    conda activate isaac_env
    python ik_solver.py
"""
import inspect
import os

import numpy as np
from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
app = SimulationApp({"headless": HEADLESS})

from isaacsim.core.utils.extensions import enable_extension   # noqa: E402

enable_extension("isaacsim.robot.manipulators")
enable_extension("isaacsim.robot.manipulators.examples")

import isaacsim.robot_motion.motion_generation as mg          # noqa: E402
from isaacsim.core.api import World                           # noqa: E402
from isaacsim.core.api.objects import VisualSphere            # noqa: E402
from isaacsim.robot.manipulators.examples.franka import Franka  # noqa: E402

# ── [ERR-071] 원고가 부르는 클래스가 있는지 먼저 본다 ────────────
print(f"\nPinkIKSolver 존재: "
      f"{'있음' if hasattr(mg, 'PinkIKSolver') else '❌ 없다 (원고 오류)'}")
print(f"실제 있는 IK 클래스: "
      f"{[n for n in dir(mg) if 'Kinematics' in n]}")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka"))

# ── [ERR-084] 상대경로 문자열 대신 공식 설정 로더 ────────────────
icl = mg.interface_config_loader
cfg = icl.load_supported_lula_kinematics_solver_config("Franka")
print(f"\n설정 로더가 주는 값:")
for k, v in cfg.items():
    print(f"   {k} = {v.split('/')[-1] if isinstance(v, str) else v}")

lula = mg.LulaKinematicsSolver(**cfg)
FRAME = "right_gripper"          # [ERR-063] 원고의 'panda_hand' 가 아니다
print(f"\n지원 프레임: {lula.get_all_frame_names()}")
print(f"원고의 'panda_hand' 가 목록에 있나: "
      f"{'예' if 'panda_hand' in lula.get_all_frame_names() else '❌ 없다'}")

art_ik = mg.ArticulationKinematicsSolver(franka, lula, FRAME)

# ── [ERR-085] 실제 메서드 이름과 인자 ────────────────────────────
sig = inspect.signature(art_ik.compute_inverse_kinematics)
print(f"\n실제 API: compute_inverse_kinematics{sig}")
print(f"   원고의 'warm_start=' 인자: "
      f"{'있음' if 'warm_start' in sig.parameters else '❌ 없다 (원고 오류)'}")

# 목표 표시용 구슬
marker = world.scene.add(VisualSphere(
    prim_path="/World/Target", name="target", radius=0.015,
    color=np.array([0.9, 0.2, 0.2])))

world.reset()
if not HEADLESS:
    from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
    set_camera_view(eye=np.array([1.6, 1.4, 1.1]),
                    target=np.array([0.35, 0.0, 0.35]))

# ── 여러 목표로 실제 IK 를 풀어 본다 ─────────────────────────────
DOWN = np.array([0.0, 1.0, 0.0, 0.0])          # (w,x,y,z) — 공구가 아래
TARGETS = [
    ("원고 b02 의 목표", np.array([0.40, 0.20, 0.30])),
    ("정면 가까이",       np.array([0.35, 0.00, 0.45])),
    ("좌측 멀리",         np.array([0.45, -0.35, 0.25])),
    ("높은 곳",           np.array([0.30, 0.10, 0.60])),
    ("팔 길이 밖 (실패해야 정상)", np.array([1.20, 0.00, 0.35])),
]

print(f"\n{'목표':>28} {'IK':>5} {'실제 오차':>10}")
for name, tgt in TARGETS:
    marker.set_world_pose(position=tgt)
    action, ok = art_ik.compute_inverse_kinematics(
        target_position=tgt, target_orientation=DOWN)

    err = None
    if ok:
        franka.apply_action(action)            # 관절각을 실제로 먹인다
        for _ in range(90):
            world.step(render=not HEADLESS)
        q = franka.get_joint_positions()[:7]
        pos, _ = lula.compute_forward_kinematics(FRAME, q)
        err = float(np.linalg.norm(np.asarray(pos).ravel()[:3] - tgt))
    print(f"{name:>28} {'✅' if ok else '❌':>5} "
          f"{(f'{err*1000:7.1f} mm' if err is not None else '      —'):>10}")

print("\n요약")
print("   원고의 PinkIKSolver 는 없다. LulaKinematicsSolver 로 바꾸면")
print("   작업영역 안의 목표는 IK 가 풀리고, 밖의 목표는 정직하게 실패한다.")

if not HEADLESS:
    for _ in range(int(os.environ.get("HOLD", "1200"))):
        world.step(render=True)
app.close()
