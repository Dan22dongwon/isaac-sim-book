"""
Ch5.X 실습 — Physics Inspector로 관절 디버깅

Franka Panda를 로드하고 관절 속성을 코드로 확인한다.
GUI에서 Tools > Physics > Physics Inspector로 동일한 결과를 시각적으로 확인할 수 있다.
"""

from isaacsim.simulation_app import SimulationApp

app = SimulationApp({"headless": False, "width": 1280, "height": 720})

import numpy as np
from isaacsim.core.api.world import World
from isaacsim.core.api.robots import Robot

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

NUCLEUS = "omniverse://localhost/NVIDIA/Assets/Isaac/6.1"
FRANKA_USD = f"{NUCLEUS}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

franka = world.scene.add(
    Robot(prim_path="/World/Franka", name="franka", usd_path=FRANKA_USD)
)
world.reset()

print("=" * 70)
print(f"로봇: Franka Panda")
print(f"관절 수: {franka.num_dof}")
print(f"관절 이름: {franka.dof_names}")
print("=" * 70)

dof_props = franka.dof_properties
print(f"\n{'관절 이름':25s}  {'타입':10s}  {'제한(도)':20s}  {'강성':>8s}  {'감쇠':>8s}")
print("-" * 80)
for i, name in enumerate(franka.dof_names):
    lower = np.degrees(dof_props["lower"][i])
    upper = np.degrees(dof_props["upper"][i])
    stiff = dof_props["stiffness"][i]
    damp = dof_props["damping"][i]
    jtype = "Prismatic" if abs(upper - lower) < 10 else "Revolute"
    print(f"  {name:25s}  {jtype:10s}  [{lower:7.1f}°, {upper:7.1f}°]  {stiff:8.0f}  {damp:8.0f}")

print("\n" + "=" * 70)
print("Physics Inspector 확인 방법:")
print("  1. Tools > Physics > Physics Inspector 열기")
print("  2. Stage에서 /World/Franka 선택 → Refresh")
print("  3. 슬라이더로 각 관절을 움직여 회전 방향과 제한을 확인")
print("  4. 위 코드 출력과 GUI 결과가 일치하는지 비교")
print("=" * 70)

print("\n관절 순차 동작 시연 (5초 간격)...")
world.reset()

target_positions = np.zeros(franka.num_dof)

for i in range(min(7, franka.num_dof)):
    lower = dof_props["lower"][i]
    upper = dof_props["upper"][i]
    mid = (lower + upper) / 2.0
    target_positions[i] = mid
    franka.set_joint_positions(target_positions)

    print(f"  joint {i} ({franka.dof_names[i]}): {np.degrees(mid):.1f}°")
    for _ in range(300):
        world.step(render=True)

    target_positions[i] = 0.0
    franka.set_joint_positions(target_positions)
    for _ in range(150):
        world.step(render=True)

print("\n시연 완료. GUI에서 Physics Inspector를 직접 사용해 보세요.")

while app.is_running():
    world.step(render=True)

app.close()
