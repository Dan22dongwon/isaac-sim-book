from isaacsim import SimulationApp  # Isaac Sim 앱 런타임
# 시뮬레이션 앱 초기화
app = SimulationApp({"headless": False})  # GUI 모드

from isaacsim.core.api import World  # 6.1 호환 고수준 API
from isaacsim.core.api.objects import DynamicSphere
import numpy as np

# World 생성 (물리 시뮬레이션 컨테이너)
world = World(stage_units_in_meters=1.0)

# 지면(Ground Plane) 추가
world.scene.add_default_ground_plane()

# 구체(Sphere) 추가 — 물리 적용됨
sphere = world.scene.add(
    DynamicSphere(
        prim_path="/World/FallingSphere",
        name="falling_sphere",
        position=np.array([0.0, 0.0, 5.0]),  # 5m 높이에서 낙하
        radius=0.1,
        color=np.array([1.0, 0.0, 0.0]),     # 빨간색
    )
)

# 시뮬레이션 초기화
world.reset()

# 메인 루프: 500 스텝 실행
for i in range(500):
    world.step(render=True)

    # 50 스텝마다 구체 위치 출력
    if i % 50 == 0:
        pos = sphere.get_world_pose()[0]
        print(f"Step {i:3d} | 구체 위치: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.3f}m")

# 시뮬레이터 종료
app.close()
print("\n✅ Hello Isaac Sim 완료!")
