"""10.6 b03~b10 — G코드 → 로봇 궤적 종단간 (원고 수정본)

CNC 가공 파일을 읽어 로봇이 그 구멍 자리를 순서대로 찍는다.
원고 b03·b04(파싱) · b05(좌표변환) · b06(궤적) · b07(검증) · b09(내보내기)
· b10(종단간) 을 한 파일로 합치고 고쳤다.

원고 대비 고친 곳
------------------------------------------------------------------
  [ERR-063] RMPflow 설정을 상대경로 문자열로 주면 파일을 못 찾는다.
            `end_effector_frame_name` 도 'panda_hand' 가 아니라
            **'right_gripper'** 다. 공식 설정 로더를 쓴다.

  [ERR-063] `ArticulationMotionPolicy(rmpflow=...)` 는 인자 이름이 없다.
            실제 시그니처는 (robot_articulation, motion_policy, ...) 다.

  [ERR-072] `isaacsim.robot.manipulators.examples` 확장이 기본으로 꺼져 있다.
            import 앞에 enable_extension 을 부른다.

  [ERR-077] 원고 b07 `validate_trajectory` 는 `target_sequence` 를 순회하면서
            **`target_position` 을 한 번도 쓰지 않는다.** 로봇을 움직이지도
            않으므로 같은 관절값을 N 번 읽고 같은 경고를 N 번 낸다.
            궤적 실행 **중에** 표본을 모으도록 고쳤다.

  [ERR-078] 원고 b10 의 작업물 블록이 드릴 구멍 3개 중 2개를 벗어난다.
            블록은 x 0.375~0.525 인데 구멍은 x 0.5 / 0.55 / 0.6 이다.
            구멍을 덮도록 블록 크기와 위치를 다시 잡았다.

  [ERR-081] 원고는 **RMPflow 가 모는 프레임과 다른 프레임**으로 도달을 잰다.
            RMPflow: right_gripper / 원고의 측정: franka.end_effector
            (= panda_rightfinger). 둘은 40.7 mm 떨어진 **상수** 차이다.
            그런데 원고의 판정은 `< 0.005` 다. 영원히 참이 될 수 없다.
            로봇이 1.1 mm 로 도달해도 전부 "미도달", 성공률 0% 가 찍힌다.

  [ERR-082] 원고에는 **공구 길이(TCP 오프셋)** 가 없다.
            G코드의 Z 는 드릴 **끝** 기준인데 로봇 목표는 **손목** 기준이다.
            빼 주지 않으면 손목을 구멍 속으로 밀어 넣으라는 말이 된다.

  [ERR-080] 원고 b04 는 `re` 를 import 하지 않는다(b03 에 의존).
            `prev_z` 는 대입만 하고 쓰지 않는 죽은 코드다.

확인했으나 오류가 아니었던 것
------------------------------------------------------------------
  작업물을 `add_obstacle()` 로 등록하면 RMPflow 가 표면 가공을 방해할까?
  → **아니다.** TCP 오프셋(ERR-082)을 넣으면 손목이 블록 위에 머무르므로
    장애물 등록 여부와 무관하게 도달률 100% 다. 실측으로 둘 다 확인했다.

실행:
    conda activate isaac_env
    python gcode_to_robot.py                  # GUI
    HEADLESS=1 python gcode_to_robot.py       # 화면 없이
    OBSTACLE=1 python gcode_to_robot.py       # 원고처럼 작업물을 장애물로
"""
import csv
import os
import re

import numpy as np
from isaacsim import SimulationApp

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
OBSTACLE = os.environ.get("OBSTACLE", "0") == "1"
app = SimulationApp({"headless": HEADLESS})

# ── [ERR-072] import 보다 먼저 확장을 켠다 ───────────────────────
from isaacsim.core.utils.extensions import enable_extension   # noqa: E402

enable_extension("isaacsim.robot.manipulators")
enable_extension("isaacsim.robot.manipulators.examples")

from isaacsim.core.api import World                           # noqa: E402
from isaacsim.core.api.objects import FixedCuboid             # noqa: E402
from isaacsim.robot.manipulators.examples.franka import Franka  # noqa: E402
from isaacsim.robot_motion.motion_generation import (         # noqa: E402
    RmpFlow, ArticulationMotionPolicy, interface_config_loader as icl)

HERE = os.path.dirname(os.path.abspath(__file__))


# ── 1단계 : G코드 파싱 (원고 b03·b04) ────────────────────────────
def parse_gcode_work_points(path, z_threshold=0.0):
    """실제 가공이 일어나는 지점만 뽑는다.

    G1(가공 이동)이면서 Z 가 임계값 아래로 내려간 순간 = 재료에 닿는 순간.
    G0(빠른 이동)은 공중 이동이라 제외한다.
    """
    pts = []
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    move_re = re.compile(r"^(G0|G1)\s+(.*)$")
    coord_re = re.compile(r"([XYZ])(-?\d+\.?\d*)")
    with open(path) as f:
        for line in f:
            line = line.split(";")[0].strip()          # 주석 제거
            m = move_re.match(line)
            if not m:
                continue
            for ax, val in coord_re.findall(m.group(2)):
                pos[ax] = float(val)
            if m.group(1) == "G1" and pos["Z"] < z_threshold:
                pts.append([pos["X"], pos["Y"], pos["Z"]])
    return np.array(pts)


gcode_mm = parse_gcode_work_points(os.path.join(HERE, "drilling.nc"))
gcode_m = gcode_mm / 1000.0                            # mm → m (Isaac 은 m)
print(f"\n① G코드에서 작업 지점 {len(gcode_m)}개 추출 (mm)")
for i, p in enumerate(gcode_mm):
    print(f"    구멍 {i+1}: X={p[0]:.0f} Y={p[1]:.0f} Z={p[2]:.0f}")


# ── 2단계 : CAD → 로봇 좌표계 (원고 b05) ─────────────────────────
def cad_to_robot_frame(pts, origin, rot=None):
    """작업물 원점이 로봇 기준 어디에 놓였는지를 더해 준다."""
    if rot is None:
        rot = np.eye(3)
    return (rot @ pts.T).T + origin


WORKPIECE_ORIGIN = np.array([0.4, -0.15, 0.1])         # 원고 b10 값
targets = cad_to_robot_frame(gcode_m, WORKPIECE_ORIGIN)
print(f"\n② 로봇 좌표계로 변환 (작업물 원점 {WORKPIECE_ORIGIN})")
for i, p in enumerate(targets):
    print(f"    구멍 {i+1}: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m  "
          f"베이스에서 {np.linalg.norm(p):.3f} m")

# ── [ERR-078] 블록이 구멍을 덮도록 다시 잡는다 ───────────────────
lo, hi = targets.min(axis=0), targets.max(axis=0)
MARGIN = 0.04
wp_center = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.05])
wp_scale = np.array([hi[0] - lo[0] + 2 * MARGIN,
                     hi[1] - lo[1] + 2 * MARGIN, 0.10])
print(f"\n   작업물 블록 중심 {np.round(wp_center, 3)}  크기 {np.round(wp_scale, 3)}")
print(f"   (원고 값은 중심 [0.45 -0.10 0.05] 크기 [0.15 0.15 0.1] 로 "
      f"구멍 3개 중 2개를 벗어난다)")

# ── 3단계 : 씬 ───────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
workpiece = world.scene.add(FixedCuboid(
    prim_path="/World/Workpiece", name="workpiece",
    position=wp_center, scale=wp_scale,
    color=np.array([0.5, 0.5, 0.55])))
franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka"))

# ── [ERR-063] 공식 설정 로더 — 상대경로 문자열을 쓰지 않는다 ─────
cfg = icl.load_supported_motion_policy_config("Franka", "RMPflow")
print(f"\n③ RMPflow end_effector_frame_name = {cfg['end_effector_frame_name']}"
      f"   (원고는 'panda_hand')")
rmpflow = RmpFlow(**cfg)

if OBSTACLE:
    rmpflow.add_obstacle(workpiece)        # 원고 b10 이 하는 것
    print("   작업물을 **장애물로 등록**했다 (원고 방식)")

# ── [ERR-063] 인자 이름은 motion_policy 다 ───────────────────────
motion_policy = ArticulationMotionPolicy(
    robot_articulation=franka, motion_policy=rmpflow)

world.reset()
if not HEADLESS:
    from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
    set_camera_view(eye=np.array([1.7, -1.3, 1.2]),
                    target=np.array([0.45, -0.07, 0.30]))

# ── 4단계 : 접근 → 작업 → 후퇴 시퀀스 ────────────────────────────
APPROACH_H = 0.05
TOOL_DOWN = np.array([0.0, 1.0, 0.0, 0.0])             # 공구가 아래를 본다

# ── [ERR-082] 원고에는 **공구 길이(TCP 오프셋)** 가 없다 ──────────
# G코드의 Z 는 드릴 **끝** 기준이다. 그런데 로봇에 주는 목표는
# **손목 프레임**(right_gripper) 기준이다. 공구 길이만큼 빼 주지 않으면
# 손목을 구멍 속으로 밀어 넣으라는 말이 된다.
# 원고 그대로 돌리면 작업 지점 3개가 17.5 mm 에서 멈춘다 — 집게가
# 블록 윗면에 닿아 더 못 내려가기 때문이다. 실물 드릴은 재료를 깎지만
# 시뮬레이션의 블록은 깎이지 않는다.
TOOL_LENGTH = float(os.environ.get("TOOL_LENGTH", "0.05"))   # 드릴 길이
TCP = np.array([0.0, 0.0, TOOL_LENGTH])
print(f"\n   공구 길이(TCP) {TOOL_LENGTH*1000:.0f} mm — 손목 목표 = 구멍 + 이만큼 위")

sequence, labels = [], []
for i, p in enumerate(targets):
    work = p + TCP                                     # 손목이 가야 할 곳
    up = work + np.array([0, 0, APPROACH_H])
    sequence += [up, work, up]
    labels += [f"구멍{i+1} 접근", f"구멍{i+1} 작업", f"구멍{i+1} 후퇴"]

print(f"\n④ {len(sequence)}개 세부 지점 순회 "
      f"(장애물 등록: {'예' if OBSTACLE else '아니오'})")

# ── [ERR-081] 원고는 **RMPflow 가 모는 프레임과 다른 프레임**으로 잰다 ──
#   RMPflow 가 모는 것        : right_gripper
#   원고가 재는 것            : franka.end_effector = panda_rightfinger
#   둘은 약 40 mm 떨어져 있다 (실측 40.7 mm, 자세와 무관한 상수).
#   그런데 원고의 판정은 `< 0.005` (5 mm) 다. **영원히 참이 될 수 없다.**
#   로봇이 1.1 mm 로 정확히 도달해도 전부 "미도달", 성공률 0% 가 찍힌다.
#   RMPflow 가 모는 그 프레임으로 재야 한다.
def ee_pos_of(q):
    """RMPflow 가 실제로 제어하는 손끝(right_gripper) 위치."""
    p = rmpflow.get_end_effector_pose(q[:7])
    p = p[0] if isinstance(p, tuple) else p
    return np.asarray(p).ravel()[:3]


TOL = 0.005
reached, closest, samples = [], [], []
for idx, tgt in enumerate(sequence):
    ok, best = False, 9.9
    for _ in range(200):
        rmpflow.set_end_effector_target(tgt, TOOL_DOWN)
        rmpflow.update_world()
        franka.apply_action(motion_policy.get_next_articulation_action(1 / 60))
        world.step(render=True)
        d = float(np.linalg.norm(ee_pos_of(franka.get_joint_positions()) - tgt))
        best = min(best, d)
        # [ERR-077] 검증 표본은 **움직이는 동안** 모은다
        samples.append((franka.get_joint_positions().copy(),
                        franka.get_joint_velocities().copy()))
        if d < TOL:
            ok = True
            break
    reached.append(ok)
    closest.append(best)
    print(f"    {labels[idx]:12s} {'✅ 도달' if ok else '⚠️ 미도달'}  "
          f"최근접 {best*1000:6.1f} mm")

rate = sum(reached) / len(reached) * 100
print(f"\n   도달 성공률 {rate:.0f}%   최악 오차 {max(closest)*1000:.1f} mm")


# ── 5단계 : 궤적 검증 (원고 b07 수정) ────────────────────────────
def validate(samples, limits):
    """[ERR-077] 원고는 target_sequence 를 돌면서 그 값을 쓰지 않는다.

    로봇을 움직이지도 않으니 같은 관절값을 N 번 읽는다. 여기서는
    실행 중에 모아 둔 표본을 본다.
    """
    # 팔 관절 7개만 본다. 8·9번은 집게 손가락이라 닫힌 상태에서
    # **항상** 한계에 붙어 있고, 넣으면 경고가 표본 수만큼 찍힌다.
    ARM = 7
    low, high = limits[:ARM, 0], limits[:ARM, 1]
    margin, warn = 0.05, []
    near = np.zeros(ARM, dtype=int)
    fast = 0
    for q, dq in samples:
        near += ((q[:ARM] < low + margin) | (q[:ARM] > high - margin))
        if np.max(np.abs(dq[:ARM])) > 3.0:
            fast += 1
    for j, c in enumerate(near):
        if c:
            warn.append(f"관절 {j} 가 한계 ±{margin} rad 안에 들어온 표본 "
                        f"{c}/{len(samples)}개")
    if fast:
        warn.append(f"관절 속도 3.0 rad/s 초과 표본 {fast}/{len(samples)}개 "
                    f"— 특이점 근접 가능성")
    return warn


limits = franka.get_articulation_controller().get_joint_limits()
warnings = validate(samples, np.asarray(limits))
print(f"\n⑤ 궤적 검증 — 표본 {len(samples)}개")
if warnings:
    print(f"   ⚠️ 경고 {len(warnings)}건")
    for w in warnings:
        print(f"      {w}")
else:
    print("   ✅ 관절 한계·속도 이상 없음")

# ── 6단계 : RoboDK 배포용 CSV (원고 b09) ─────────────────────────
# ── [ERR-083] 쿼터니언 순서를 바꿔 주지 않는다 ───────────────────
# Isaac Sim 은 **스칼라 앞** (w, x, y, z), RoboDK·ROS 는 **스칼라 뒤**
# (qx, qy, qz, qw) 다. 원고 b09 는 헤더만 qx,qy,qz,qw 로 쓰고 값은
# Isaac 순서 그대로 내보낸다.
#   실측: [0,1,0,0] 을 Isaac 이 해석한 회전행렬은 diag(1,-1,-1) = X축 180°.
#   같은 네 숫자를 스칼라 뒤로 읽으면 Y축 180° 다.
#   공구는 둘 다 아래를 보지만 **수직축 기준 180° 돌아간다.**
#   드릴처럼 회전대칭인 공구면 무해하고, 방향성 있는 공구면 틀린 자세가
#   실물 로봇으로 나간다.
w_, x_, y_, z_ = TOOL_DOWN                       # Isaac 순서 (w,x,y,z)
quat_xyzw = (x_, y_, z_, w_)                     # RoboDK 순서로 바꾼다

out = os.path.join(HERE, "validated_trajectory.csv")
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["label", "x_mm", "y_mm", "z_mm", "qx", "qy", "qz", "qw"])
    for lab, p in zip(labels, sequence):
        w.writerow([lab, *np.round(p * 1000.0, 3), *quat_xyzw])
print(f"\n⑥ RoboDK 배포용 궤적 저장: {os.path.basename(out)} "
      f"({len(sequence)}행)")

print(f"\n{'✅ 성공' if rate == 100 else '❌ 일부 미도달'}", flush=True)

if not HEADLESS:
    for _ in range(int(os.environ.get("HOLD", "1800"))):
        world.step(render=True)
app.close()
