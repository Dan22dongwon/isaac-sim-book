"""책 17장 [실습] Flying Distractors + DOPEWriter (b05) — 정리본.

원문과의 차이:
  [ERR-103] `from isaacsim.core.utils.semantics import add_update_semantics` 없음
          → `add_labels` (같은 모듈). 6.1 에서 semantics API 가 labels 로 재명명됨
            (add_update_semantics/remove 등 → add_labels/remove_labels/get_labels).

검증 범위 (에셋 의존):
  rep.create.from_usd 로 YCB(/assets/ycb/...)·도형 USD 를 불러오므로 실제 자산이
  있어야 end-to-end 로 돈다. 여기서는 add_labels 교체 + rep API(from_usd/randomizer/
  DOPEWriter) 존재를 검증했다. 자산을 갖춘 환경에서 그대로 사용 가능.

실행:  (자산 필요) python flying_distractors.py
"""
from isaacsim.core.utils.semantics import add_labels  # [ERR-103] add_update_semantics→add_labels  # noqa
import omni.replicator.core as rep

# 타겟 물체: 정확한 6D 자세 라벨이 필요한 물체
target = rep.create.from_usd(
    "/assets/ycb/006_mustard_bottle.usd",
    semantics=[("class", "mustard_bottle")],
)

# 플라잉 디스트랙터: 도형 + 물체 혼합, 충돌 볼륨 내에서 지속 이동
flying_shapes = rep.create.from_usd(
    "/assets/shapes/primitive_shapes.usd",
    count=50,
)


def mesh_dome_randomize():
    with target:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.3, -0.3, 0.1), (0.3, 0.3, 0.4)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        )
    with flying_shapes:
        rep.modify.pose(
            position=rep.distribution.uniform((-1, -1, -1), (1, 1, 1)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        )
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
    return target.node


rep.randomizer.register(mesh_dome_randomize)

# DOPE 호환 라이터로 저장 (6D 자세 라벨 포함)
writer = rep.WriterRegistry.get("DOPEWriter")
writer.initialize(output_dir="/data/pose_dataset")
