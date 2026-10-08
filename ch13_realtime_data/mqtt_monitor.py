"""책 13장 [실습] 실시간 모니터링 통합 (b07) — 동작 검증본.

원문과의 차이:
  [ERR-094] 원문은 `equipment.set_color(np.array([...]))` 를 호출한다.
            DynamicCuboid 에 set_color() 메서드가 없다 (6.1 기준).
          → `apply_visual_material(PreviewSurface(color=...))` 로 교체.
  [검증 편의] 원문은 MQTT 브로커("localhost:1883")에서 진동값을 받는다.
            브로커 없이 검증할 수 있도록 MQTT 대신 **합성 진동 신호**를 주입했다.
            MQTT 수신부(on_message/loop_start)는 opcua_bridge/mqtt_subscriber 참고.

검증 결과:
  정상 구간 → 큐브 초록, 이상값(z>3) 주입 → 큐브 빨강 전환 확인.
  캡처: ../_docs/images/book/ch13/ch13_monitor_normal.png (초록),
        ../_docs/images/book/ch13/ch13_monitor_anomaly.png (빨강)

실행:  python mqtt_monitor.py          # 합성 데이터로 자동 검증 + 캡처
"""
import os
import numpy as np

import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp

app = SimulationApp({"headless": True})

from collections import deque
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.materials import PreviewSurface
import omni.replicator.core as rep

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

equipment = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Conveyor01_Indicator",
        name="conveyor01",
        position=np.array([0.0, 0.0, 0.5]),
        scale=np.array([2.0, 0.5, 0.3]),
        color=np.array([0.2, 0.8, 0.2]),
    )
)

green = PreviewSurface(prim_path="/World/Looks/Green", color=np.array([0.2, 0.8, 0.2]))
red = PreviewSurface(prim_path="/World/Looks/Red", color=np.array([0.9, 0.1, 0.1]))

world.reset()

cam = rep.create.camera(position=(4, 4, 3), look_at=(0, 0, 0.5))
rp = rep.create.render_product(cam, (1280, 720))
rep.create.light(light_type="dome", intensity=1500)

rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annot.attach([rp])


def grab(path, frames=30):
    for _ in range(frames):
        world.step(render=True)
    # 렌더 파이프라인이 프레임을 채울 때까지 orchestrator 를 돌린다
    for _ in range(8):
        rep.orchestrator.step(rt_subframes=4)
    data = rgb_annot.get_data()
    arr = np.asarray(data)
    if arr.ndim != 3:
        print(f"  ⚠️ 렌더 미완 (shape={arr.shape}) — 재시도")
        for _ in range(8):
            rep.orchestrator.step(rt_subframes=4)
        arr = np.asarray(rgb_annot.get_data())
    arr = arr[:, :, :3].astype(np.uint8)
    arr.tofile(path + ".raw")
    with open(path + ".meta", "w") as f:
        f.write(f"{arr.shape[1]} {arr.shape[0]}")
    print(f"  캡처 {path}  shape={arr.shape} mean={arr.mean():.1f}")


history = deque(maxlen=100)
rng = np.random.default_rng(0)
anomaly_count = 0

OUT = os.path.join(os.path.dirname(__file__), "..", "_docs", "images", "book", "ch13")
os.makedirs(OUT, exist_ok=True)

# 정상 구간 (초록)
for _ in range(60):
    v = rng.normal(5.0, 0.5)
    history.append(v)
    world.step(render=False)
equipment.apply_visual_material(green)  # [ERR-094] set_color → apply_visual_material
grab(os.path.join(OUT, "ch13_monitor_normal"))

# 이상값 주입 (빨강)
value = 15.0
history.append(value)
arr = np.array(history)
z = (value - arr.mean()) / arr.std()
if abs(z) > 3.0:
    anomaly_count += 1
    equipment.apply_visual_material(red)  # [ERR-094]
    print(f"🚨 이상 감지 #{anomaly_count}: 진동={value:.2f}, Z-score={z:.2f}")
grab(os.path.join(OUT, "ch13_monitor_anomaly"))

print(f"✅ 모니터링 검증 종료. 이상 감지 {anomaly_count}회")
app.close()
