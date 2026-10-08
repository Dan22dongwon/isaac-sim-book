"""책 22장 [실습] 3DGS 배경 + 메시 전경 혼합 씬 (b05) — 정리본.

원문과의 차이:
  없음 — stage_utils.add_reference_to_stage 패턴 그대로.

검증 범위 (에셋 의존):
  /data/splats/*.usdz(3DGS)·collider.usd 가 있어야 실제로 불러온다.
  여기서는 `isaacsim.core.utils.stage.add_reference_to_stage` 존재를 검증했다.
  (앞 장들에서 동일 API 로 실제 USD 로드 성공 — Ch4·5·12 참조)

실행:  (3DGS 자산 필요)
"""
import isaacsim.core.utils.stage as stage_utils

# 1. 3DGS 배경 (시각적 사실감)
# stage_utils.add_reference_to_stage(
#     usd_path="/data/splats/factory_floor.usdz",
#     prim_path="/World/Environment/SplatBackground")

# 2. 투명 콜라이더 메시 (물리 충돌용, Ch12.2 참조)
# stage_utils.add_reference_to_stage(
#     usd_path="/data/splats/factory_floor_collider.usd",
#     prim_path="/World/Environment/PhysicsCollider")

# 3. 로봇/동적 물체는 일반 USD 메시로 배치(Ch4~5)
#    → 카메라가 3DGS 배경 + 메시 전경을 함께 렌더 → 학생 정책이 혼합 사실감으로 학습

assert hasattr(stage_utils, "add_reference_to_stage"), "add_reference_to_stage 없음"
