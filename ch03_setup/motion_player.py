"""3.10.3 — 모션 파일(텍스트)로 로봇 자세를 반복 시험한다.

자세를 파이썬 코드에 박아 두면 하나 고칠 때마다 Isaac Sim 을 다시 띄워야 한다.
텍스트 파일로 분리하면 시뮬레이터를 켜 둔 채 저장만 하면 바로 반영된다.

목표 자세와 실제 도달 자세의 차이를 출력하므로,
'관절 한계에 걸렸는지 / 게인이 부족한지' 를 바로 알 수 있다.

실행:
    conda activate isaac_env
    export OMNI_KIT_ACCEPT_EULA=YES
    OUT_DIR=./out python motion_player.py
    # 반복 감시 모드:  WATCH=1 OUT_DIR=./out python motion_player.py
"""
import os
import time

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.storage import native as nucleus_utils  # noqa: E402

OUT = os.environ.get("OUT_DIR", "./out")
os.makedirs(OUT, exist_ok=True)
MOTION_FILE = os.environ.get("MOTION_FILE", os.path.join(OUT, "motions.txt"))
WATCH = os.environ.get("WATCH", "0") == "1"
HOLD = 90                       # 자세당 유지 스텝

root = nucleus_utils.get_assets_root_path()
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=8.0 / 200.0)
world.scene.add_default_ground_plane()

FRANKA = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
stage_utils.add_reference_to_stage(usd_path=root + FRANKA, prim_path="/World/Franka")
robot = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))

world.reset()
app_utils.play(commit=True)

names = list(robot.dof_names)
N = len(names)
robot.get_articulation_controller().set_gains(
    kps=np.ones(N) * 1200.0, kds=np.ones(N) * 60.0
)

# 예시 모션 파일이 없으면 하나 만들어 둔다
if not os.path.exists(MOTION_FILE):
    with open(MOTION_FILE, "w", encoding="utf-8") as f:
        f.write(
            "# motions.txt — Franka 9관절 (라디안)\n"
            "# joint1, joint2, ..., joint7, finger1, finger2\n"
            "0.0,  -0.6,  0.0, -2.2, 0.0, 2.0, 0.78, 0.04, 0.04\n"
            "0.5,  -0.6,  0.0, -2.2, 0.0, 2.0, 0.78, 0.04, 0.04\n"
            "0.5,  -0.3,  0.0, -1.8, 0.0, 2.0, 0.78, 0.00, 0.00\n"
            "0.0,  -0.3,  0.0, -1.8, 0.0, 2.0, 0.78, 0.00, 0.00\n"
            "# 아래는 일부러 관절 한계를 넘긴 자세 — 오차가 크게 나와야 정상이다\n"
            "0.0,  -3.0,  0.0, -2.2, 0.0, 2.0, 0.78, 0.04, 0.04\n"
        )


def load_motions(path, n_dof):
    """주석과 빈 줄을 건너뛰고 자세 목록을 읽는다."""
    poses = []
    for line in open(path, encoding="utf-8"):
        line = line.split("#")[0].strip()
        if not line:
            continue
        vals = [float(v) for v in line.replace(",", " ").split()]
        if len(vals) != n_dof:
            print(f"  [건너뜀] 관절 수 불일치 {len(vals)} != {n_dof}", flush=True)
            continue
        poses.append(np.array(vals))
    return poses


def play_once(poses):
    props = robot.dof_properties
    for k, pose in enumerate(poses, 1):
        for _ in range(HOLD):
            robot.apply_action(ArticulationAction(joint_positions=pose))
            world.step(render=False)
        actual = np.asarray(robot.get_joint_positions()).ravel()
        diff = np.abs(actual - pose)
        err = float(np.degrees(diff).max())
        worst = int(np.argmax(diff))
        # 한계를 넘겨 요청한 관절을 짚어 준다
        lo, hi = float(props["lower"][worst]), float(props["upper"][worst])
        over = "" if lo <= pose[worst] <= hi else "  ← 관절 한계 초과 요청"
        print(f"  자세 {k}/{len(poses)}  최대 오차 {err:5.1f}도  "
              f"({names[worst]}){over}", flush=True)


print("\n===MOTION_PLAYER===", flush=True)
print(f"  로봇 관절 {N}개: {names[:3]} ...", flush=True)
print(f"  모션 파일: {MOTION_FILE}", flush=True)

poses = load_motions(MOTION_FILE, N)
print(f"  자세 {len(poses)}개 로드\n", flush=True)
play_once(poses)

if WATCH:
    # 파일이 바뀌면 다시 읽어 재생한다 — 시뮬을 끄지 않고 자세를 다듬을 수 있다
    print("\n  감시 모드: motions.txt 를 저장하면 즉시 반영된다 (Ctrl+C 로 종료)", flush=True)
    last = os.path.getmtime(MOTION_FILE)
    while app.is_running():
        time.sleep(0.5)
        m = os.path.getmtime(MOTION_FILE)
        if m != last:
            last = m
            poses = load_motions(MOTION_FILE, N)
            print(f"\n  모션 파일 갱신 — 자세 {len(poses)}개", flush=True)
            play_once(poses)

print("===MOTION_PLAYER_END===", flush=True)
app.close()
