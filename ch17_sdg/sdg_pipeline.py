"""책 17장 [실습] 전체 SDG 통합 파이프라인 (b07) — 동작 검증본.

원문과의 차이:
  [ERR-105] `rep.trigger.on_frame(num_frames=N)` 의 num_frames 는 deprecated
          → `max_execs=N`.
  [ERR-106] `rep.modify.attribute("intensity", rep.distribution.uniform(500,2500))`
          → DomeLight.intensity 는 int 스키마라 float 샘플에서 TypeError(조용히 실패).
            여기서는 조명 세기 랜덤화를 생략(또는 exposure 사용). 나머지 랜덤화는 유지.
  [검증 편의] 출력 /data → 로컬 임시폴더, 프레임 수는 인자(기본 5)로.

검증 결과:
  5프레임 → 31개 파일(rgb·bbox·segmentation) 생성, 세그멘테이션 class 분리 정확.
  캡처: ../_docs/images/book/ch17/ch17_sdg_rgb.png, ch17_sdg_segmentation.png

실행:  python sdg_pipeline.py [프레임수]
"""
import sys
import os
import isaacsim  # noqa
from isaacsim.simulation_app import SimulationApp

app = SimulationApp({"headless": True})

import omni.replicator.core as rep

N = int(sys.argv[1]) if len(sys.argv) > 1 else 5
OUT = os.environ.get("SDG_OUT", "/tmp/ch17_sdg_output")
os.makedirs(OUT, exist_ok=True)

camera = rep.create.camera(position=(0, 0, 2.5), look_at=(0, 0, 0))
render_product = rep.create.render_product(camera, (512, 512))
floor = rep.create.plane(scale=(3, 3, 1), semantics=[("class", "table")])
boxes = rep.create.cube(count=8, semantics=[("class", "box")])
distractors = rep.create.group([
    rep.create.cylinder(count=4),
    rep.create.sphere(count=4),
])
rep.create.light(light_type="dome", intensity=1000)


def randomize_all():
    with boxes:
        rep.randomizer.scatter_2d(surface_prims=[floor], check_for_collisions=True)
        rep.modify.pose(rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)))
        rep.randomizer.color(
            colors=rep.distribution.uniform((0.3, 0.1, 0.1), (0.9, 0.6, 0.3))
        )
    with distractors:
        rep.randomizer.scatter_2d(surface_prims=[floor], check_for_collisions=True)
    # [ERR-106] DomeLight.intensity 랜덤화는 타입 불일치로 제외
    with camera:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.5, -0.5, 2.0), (0.5, 0.5, 3.0))
        )
    return boxes.node


rep.randomizer.register(randomize_all)

# [ERR-105] num_frames → max_execs
with rep.trigger.on_frame(max_execs=N):
    rep.randomizer.randomize_all()

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir=OUT, rgb=True,
    bounding_box_2d_tight=True, semantic_segmentation=True,
)
writer.attach([render_product])

print(f"🎨 SDG 시작 ({N}프레임) → {OUT}")
rep.orchestrator.run()
# orchestrator 완료 대기
import omni.kit.app
for _ in range(300):
    omni.kit.app.get_app().update()
    if rep.orchestrator.get_status() in ("STOPPED", "PAUSED", rep.orchestrator.Status.STOPPED):
        break

n_files = sum(len(f) for _, _, f in os.walk(OUT))
print(f"✅ SDG 완료: {n_files}개 파일")
app.close()
